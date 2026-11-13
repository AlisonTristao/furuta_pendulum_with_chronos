#include "embarcado.h"

#include "controlador.h"
#include "controle.h"
#include "planta.h"
#include "saida.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PI_D 3.14159265358979323846

extern const char furuta_ini[];               // pc/furuta.ini embutido (furuta_ini.c, gerado pelo CMake)

void bench_roda(const config *cfg);           // bench.c

static config s_cfg;                        // configuração em uso (cfg -set), copiada no início do ensaio
static TaskHandle_t s_tarefa_mpc;
static QueueHandle_t s_fila_tel;
static volatile int s_pedido = 0;           // 0 = nada, 1 = controle, 2 = a seco, 3 = bench, 4 = sim, -1 = desliga
static volatile bool s_ativo = false;
static volatile fase_t s_fase = FASE_PARADO;
static controlador s_ctl;

bool embarcado_config(const char *par)
{
    if (s_ativo) return false;
    return config_arg_silencioso(&s_cfg, par) == 0;
}

void embarcado_config_padrao(void)
{
    if (s_ativo) return;
    config_le_texto(furuta_ini, strlen(furuta_ini), &s_cfg);
}

static int resumo(const config *c, char *buf, int n)
{
    const mpc_config *m = &c->mpc;
    const ensaio *e = &c->e;
    return snprintf(buf, n,
                    "mpc: Np=%d horizonte=%.3f discretizacao=%s Q=%g,%g,%g,%g Rdu=%g u=[%.3f,%.3f] peso_exp=%.2f "
                    "l_max=%d theta1=[%.3f,%.3f]\n"
                    "fisico: L1=%.4f l2=%.4f m2=%.5f I2=%.4e mu2=%.4e\n"
                    "ensaio: Ts=%.3f duracao=%.1f modo=%s comp_atraso_ms=%.0f impulso=%.0f/%.0f estimador=%d",
                    m->Np, (double)m->horizonte, modelo_disc_nome(m->discretizacao), (double)m->Q[0], (double)m->Q[1], (double)m->Q[2], (double)m->Q[3],
                    (double)m->Rdu, (double)m->u_min, (double)m->u_max, (double)m->peso_exp, m->l_max,
                    (double)m->theta1_min, (double)m->theta1_max, c->f.L1, c->f.l2, c->f.m2, c->f.I2, c->f.mu2, e->Ts,
                    e->duracao, e->equilibrio ? "equilibrio" : "swingup", e->comp_atraso_ms, e->impulso_a,
                    e->impulso_ms, e->estimador);
}

int embarcado_config_texto(char *buf, int n) { return resumo(&s_cfg, buf, n); }

void embarcado_massa(double *m2, double *g)
{
    *m2 = s_cfg.f.m2;
    *g = s_cfg.f.g;
}

bool embarcado_ativo(void) { return s_ativo; }
fase_t embarcado_fase(void) { return s_fase; }

const char *embarcado_fase_nome(fase_t f)
{
    static const char *nomes[] = {"PARADO", "ESPERA", "RODANDO", "SECO", "BENCH", "SIM"};
    return nomes[f];
}

const char *embarcado_liga(int modo)
{
    if (s_ativo) return "já tem um ensaio rodando";
    if (modo == 1 && controle_estado() != EST_PRONTO) return "faça o homing antes (furuta -home)";
    if (modo < 1 || modo > 4) return "modo inválido";
    s_pedido = modo;
    xTaskNotifyGive(s_tarefa_mpc);
    return NULL;
}

void embarcado_desliga(void)
{
    if (s_ativo) s_pedido = -1;
}

bool embarcado_telemetria(telemetria_t *t, unsigned espera_ms)
{
    return xQueueReceive(s_fila_tel, t, pdMS_TO_TICKS(espera_ms)) == pdTRUE;
}

static long long relogio_us(void) { return esp_timer_get_time(); }
#define FOLGA_US 2500   /* do período: estimador, compensação, telemetria (o resto é orçamento do MPC) */

static double wrap(double a) { return a - 2 * PI_D * floor((a + PI_D) / (2 * PI_D)); }

// Um ensaio inteiro (modo 1 ou 2). Devolve o motivo do fim.
static const char *roda_ensaio(int modo)
{
    config cfg = s_cfg;
    ensaio *e = &cfg.e;
    if (modo == 1 && e->usa_limite_homing) {      // restrição de θ1 do MPC = limite do homing menos a margem
        cfg.mpc.theta1_max = controle_limite() - e->margem_theta1;
        cfg.mpc.theta1_min = -cfg.mpc.theta1_max;
    }
    int ts_ms = (int)lround(e->Ts * 1e3);
    controle_assina_amostra(s_tarefa_mpc, ts_ms);
    ulTaskNotifyTake(pdTRUE, 0);

    s_fase = modo == 1 ? FASE_ESPERA : FASE_SECO;
    bool criado = false;
    double a_pend = 0, t_quieto = -1, t_espera = 0;
    int k = 0, K = (int)lround(e->duracao / e->Ts);
    unsigned estouros = 0;
    const char *motivo = "fim da duração";
    if (modo == 1)
        saida_aviso(e->equilibrio ? "esperando o pêndulo perto do topo" : "esperando o pêndulo parado embaixo (1 s)");
    for (;;) {
        if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500))) {
            motivo = "sem amostra da malha";
            break;
        }
        if (s_pedido == -1) {
            motivo = "desligado (furuta -stop)";
            break;
        }
        if (modo == 1) {
            estado_t est = controle_estado();
            if (est != EST_PRONTO && est != EST_RODANDO) {
                motivo = controle_erro()[0] ? controle_erro() : controle_estado_nome(est);
                break;
            }
            controle_acel((float)a_pend);         // o comando calculado no período anterior entra agora
        }
        leitura_t l;
        controle_amostra(&l);
        double x[4] = {l.t1, l.t2, l.dt1, l.dt2};

        if (s_fase == FASE_ESPERA) {              // braço parado no meio até a condição de partida
            double th = wrap(x[1]);
            t_espera += e->Ts;
            bool parte;
            if (e->equilibrio) {
                parte = fabs(th) < e->janela_equilibrio && fabs(x[3]) < 2;
            } else {                              // parado embaixo por 1 s
                bool quieto = fabs(fabs(th) - PI_D) < 0.1 && fabs(x[3]) < 0.1;
                if (!quieto) t_quieto = -1;
                else if (t_quieto < 0) t_quieto = t_espera;
                parte = quieto && t_espera - t_quieto > 1.0;
            }
            if (t_espera > 60) {
                motivo = "60 s sem a condição de partida";
                break;
            }
            if (!parte) continue;
            s_fase = FASE_RODANDO;
        }
        if (!criado) {                            // cria na partida (~0,1 s: DLQR em double); a_pend = 0 até lá
            if (controlador_cria(&s_ctl, &cfg, x) != 0) {
                motivo = "falha ao criar o MPC";
                break;
            }
            criado = true;
            mpc_orcamento(s_ctl.c, relogio_us, ts_ms * 1000LL - FOLGA_US);   /* sem estouro: corta o laço dos rho */
            int perdidas = (int)ulTaskNotifyTake(pdTRUE, 0);                 // amostras perdidas na criação
            char txt[400];
            resumo(&cfg, txt, sizeof(txt));       // o que está sendo usado de fato (θ1 do homing)
            saida_aviso("%s ligado (%d amostras na criação)\n%s", modo == 1 ? "controle" : "a seco", perdidas, txt);
            continue;                             // a amostra já ficou velha: começa na próxima
        }
        int64_t t0 = esp_timer_get_time();
        mpc_info in;
        double a = controlador_passo(&s_ctl, x, k * e->Ts, &in);
        float t_calc = (esp_timer_get_time() - t0) * 1e-3f;
        a_pend = a;
        if (ulTaskNotifyValueClear(NULL, 0) > 0) {   // passou do período: aplica já e conta o estouro
            estouros++;
            if (modo == 1) controle_acel((float)a_pend);
        }
        telemetria_t tl = {(float)(k * e->Ts), {(float)x[0], (float)x[1], (float)x[2], (float)x[3]}, (float)s_ctl.u,
                           (float)a, t_calc, (unsigned char)in.n_qp, (unsigned char)in.n_it,
                           (unsigned short)(estouros > 65535 ? 65535 : estouros)};
        xQueueSend(s_fila_tel, &tl, 0);           // fila cheia: perde a amostra, não atrasa o controle
        k++;
        if (!in.finito) {
            motivo = "controle não finito";
            break;
        }
        if (fabs(x[3]) > e->theta2_aborta) {
            motivo = "|dθ2| acima de dtheta2_aborta";
            break;
        }
        if (k >= K) break;
    }
    controle_assina_amostra(NULL, 0);
    if (criado) controlador_libera(&s_ctl);
    // sem comando novo: o watchdog da malha (200 ms) freia o braço até parar
    saida_aviso("fim do ensaio: %s (%.1f s, %u estouros)", motivo, k * e->Ts, estouros);
    return motivo;
}

// Simulação em tempo real (modo 4, furuta -sim): o mesmo controlador do ensaio, com o orçamento de tempo, contra a
// planta calibrada do sim.exe (comum/planta.c, seção [sim]), um passo a cada Ts. Publica a telemetria como um
// ensaio, e o tópico sensores mostra o que a planta simulada mede. Não mexe no motor.
static portMUX_TYPE s_mux_sim = portMUX_INITIALIZER_UNLOCKED;
static float s_sim_x[4];
static volatile bool s_sim_ativo = false;

bool embarcado_sim_leitura(float x[4])
{
    if (!s_sim_ativo) return false;
    taskENTER_CRITICAL(&s_mux_sim);
    for (int i = 0; i < 4; i++) x[i] = s_sim_x[i];
    taskEXIT_CRITICAL(&s_mux_sim);
    return true;
}

static void sim_publica(const double xm[4])
{
    taskENTER_CRITICAL(&s_mux_sim);
    for (int i = 0; i < 4; i++) s_sim_x[i] = (float)xm[i];
    taskEXIT_CRITICAL(&s_mux_sim);
}

#define SIM_PLANTA_US 3000   /* reserva do período para a planta (double emulado no S3) */

static void roda_sim(void)
{
    static planta p;              // 2,5 KB: fora da pilha
    config cfg = s_cfg;
    const ensaio *e = &cfg.e;
    planta_cria(&p, &cfg);
    sim_publica(p.xm);
    s_sim_ativo = true;
    if (controlador_cria(&s_ctl, &cfg, p.x) != 0) {
        s_sim_ativo = false;
        saida_aviso("sim: falha ao criar o MPC");
        return;
    }
    int ts_ms = (int)lround(e->Ts * 1e3);
    mpc_orcamento(s_ctl.c, relogio_us, ts_ms * 1000LL - FOLGA_US - SIM_PLANTA_US);
    char txt[400];
    resumo(&cfg, txt, sizeof(txt));
    saida_aviso("sim ligado: planta simulada (seção [sim]: atrito seco %g, encoder %d bits, semente %d), %.0f s\n%s",
                e->tc2, e->bits_enc, e->semente, e->duracao, txt);

    int K = (int)lround(e->duracao / e->Ts), k, topo = 0;
    unsigned estouros = 0;
    double t_sobe = -1, calc_soma = 0, calc_max = 0, planta_soma = 0, planta_max = 0;
    const char *motivo = "fim da duração";
    TickType_t t_ini = xTaskGetTickCount();
    for (k = 0; k < K; k++) {
        if (s_pedido == -1) {
            motivo = "desligado (furuta -stop)";
            break;
        }
        double xm[4] = {p.xm[0], p.xm[1], p.xm[2], p.xm[3]};   // o controlador troca dθ2 pelo estimado
        int64_t t0 = esp_timer_get_time();
        mpc_info in;
        double a = controlador_passo(&s_ctl, xm, k * e->Ts, &in);
        int64_t t1 = esp_timer_get_time();
        planta_passo(&p, a);                      // o comando de k entra em k+1 (ensaio.embarcado)
        int64_t t2 = esp_timer_get_time();
        sim_publica(p.xm);
        float t_calc = (t1 - t0) * 1e-3f;
        calc_soma += t_calc, calc_max = fmax(calc_max, t_calc);
        planta_soma += (t2 - t1) * 1e-3, planta_max = fmax(planta_max, (t2 - t1) * 1e-3);
        telemetria_t tl = {(float)(k * e->Ts), {(float)xm[0], (float)xm[1], (float)xm[2], (float)xm[3]},
                           (float)s_ctl.u, (float)a, t_calc, (unsigned char)in.n_qp, (unsigned char)in.n_it,
                           (unsigned short)(estouros > 65535 ? 65535 : estouros)};
        xQueueSend(s_fila_tel, &tl, 0);
        double th = wrap(p.x[1]);
        if (fabs(th) < 0.35) topo++;
        if (t_sobe < 0 && fabs(th) < 0.1 && fabs(p.x[3]) < 0.5) t_sobe = k * e->Ts;
        if (!in.finito) {
            motivo = "controle não finito";
            break;
        }
        if (fabs(p.x[3]) > e->theta2_aborta) {
            motivo = "|dθ2| acima de dtheta2_aborta";
            break;
        }
        if (xTaskDelayUntil(&t_ini, pdMS_TO_TICKS(ts_ms)) == pdFALSE) estouros++;   // já tinha passado do período
    }
    s_sim_ativo = false;
    controlador_libera(&s_ctl);
    int n = k > 0 ? k : 1;
    saida_aviso("fim da sim: %s (%.1f s) | subida %.2f s | topo %d%% | MPC médio %.2f ms, máx %.2f | planta médio "
                "%.2f ms, máx %.2f | %u estouros",
                motivo, k * e->Ts, t_sobe, 100 * topo / n, calc_soma / n, calc_max, planta_soma / n, planta_max,
                estouros);
}

static void tarefa_mpc(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int modo = s_pedido;
        if (modo < 1 || modo > 4) continue;
        s_ativo = true;
        if (modo == 3) {
            s_fase = FASE_BENCH;
            config cfg = s_cfg;
            bench_roda(&cfg);
        } else if (modo == 4) {
            s_fase = FASE_SIM;
            roda_sim();
        } else {
            roda_ensaio(modo);
        }
        s_pedido = 0;
        s_fase = FASE_PARADO;
        s_ativo = false;
    }
}

void embarcado_init(void)
{
    embarcado_config_padrao();
    s_fila_tel = xQueueCreate(64, sizeof(telemetria_t));
    // núcleo 1 só para o MPC
    xTaskCreatePinnedToCore(tarefa_mpc, "mpc", 12288, NULL, configMAX_PRIORITIES - 2, &s_tarefa_mpc, 1);
}
