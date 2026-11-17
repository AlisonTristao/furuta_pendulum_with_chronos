#include "embarcado.h"

#include "controlador.h"
#include "controle.h"
#include "planta.h"
#include "saida.h"
#include "termopar.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PI_D 3.14159265358979323846

extern const char furuta_ini[];               // pc/furuta.ini embutido (furuta_ini.c, gerado pelo CMake)

void bench_roda(const config *cfg);           // bench.c

static config s_cfg;                        // configuração em uso (cfg -set), copiada no início do ensaio
static ensaio_resultado s_ultimo;           // resultado do último ensaio de controle (furuta -lote)
static portMUX_TYPE s_mux_res = portMUX_INITIALIZER_UNLOCKED;
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

// Inclinação da mesa medida pelo furuta -inclina, na flash (NVS "furuta"/"incl": seno e cosseno [rad]). Vale por
// cima do furuta.ini, no boot e no cfg -reset, até a próxima medição.
#define NVS_NS "furuta"
#define NVS_INCL "incl"

static bool inclinacao_salva(double sc[2])
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;   // NVS ainda não inicializada, ou nada salvo
    size_t n = 2 * sizeof(double);
    bool ok = nvs_get_blob(h, NVS_INCL, sc, &n) == ESP_OK && n == 2 * sizeof(double);
    nvs_close(h);
    return ok;
}

void embarcado_inclinacao_carrega(void)
{
    double sc[2];
    if (s_ativo || !inclinacao_salva(sc)) return;
    s_cfg.e.incl_seno = sc[0], s_cfg.e.incl_cosseno = sc[1];
    saida_aviso("inclinação da mesa da flash (furuta -inclina): seno %+.4f, cosseno %+.4f", sc[0], sc[1]);
}

void embarcado_config_padrao(void)
{
    if (s_ativo) return;
    config_le_texto(furuta_ini, strlen(furuta_ini), &s_cfg);
    double sc[2];
    if (inclinacao_salva(sc)) s_cfg.e.incl_seno = sc[0], s_cfg.e.incl_cosseno = sc[1];
    // furuta -sim sem o atrito seco do pêndulo: com ele o equilíbrio fica num ciclo limite (θ1 ±0,24 rad a ~0,8 Hz,
    // θ2 ±2°), como no real; para ver os gráficos limpos. O sim.exe continua com o do furuta.ini. Volta com
    // cfg -set sim.tc2=2e-4
    s_cfg.e.tc2 = 0;
}

static int resumo(const config *c, char *buf, int n)
{
    const mpc_config *m = &c->mpc;
    const ensaio *e = &c->e;
    return snprintf(buf, n,
                    "mpc: Np=%d horizonte=%.3f discretizacao=%s Q=%g,%g,%g,%g Rdu=%g u=[%.3f,%.3f] peso_exp=%.2f "
                    "l_max=%d theta1=[%.3f,%.3f]\n"
                    "fisico: L1=%.4f l2=%.4f m2=%.5f I2=%.4e mu2=%.4e\n"
                    "ensaio: Ts=%.3f duracao=%.1f modo=%s lado=%d lado_auto=%d vies=%+.4f comp_atraso_ms=%.0f impulso=%.0f/%.0f estimador=%d\n"
                    "trim: liga=%d tau=%.1f max=%.3f janela=%.3f valor=%+.4f rampa=%.1f\n"
                    "inclinacao: pontos=%d grava=%.1f amplitude=%.2f ciclos=%d seno=%+.4f cosseno=%+.4f",
                    m->Np, (double)m->horizonte, modelo_disc_nome(m->discretizacao), (double)m->Q[0], (double)m->Q[1], (double)m->Q[2], (double)m->Q[3],
                    (double)m->Rdu, (double)m->u_min, (double)m->u_max, (double)m->peso_exp, m->l_max,
                    (double)m->theta1_min, (double)m->theta1_max, c->f.L1, c->f.l2, c->f.m2, c->f.I2, c->f.mu2, e->Ts,
                    e->duracao, e->equilibrio ? "equilibrio" : "swingup", e->lado, e->lado_auto, e->subida_vies, e->comp_atraso_ms, e->impulso_a,
                    e->impulso_ms, e->estimador, e->trim, e->trim_tau, e->trim_max, e->trim_janela, e->trim_valor, e->trim_rampa,
                    e->incl_pontos, e->incl_grava, e->incl_amplitude, e->incl_ciclos, e->incl_seno, e->incl_cosseno);
}

int embarcado_config_texto(char *buf, int n) { return resumo(&s_cfg, buf, n); }

void embarcado_massa(double *m2, double *g)
{
    *m2 = s_cfg.f.m2;
    *g = s_cfg.f.g;
}

void embarcado_inclinacao_params(inclinacao_params *p)
{
    const ensaio *e = &s_cfg.e;
    const fisico *f = &s_cfg.f;
    p->pontos = e->incl_pontos, p->grava = e->incl_grava;
    p->amplitude = e->incl_amplitude, p->ciclos = e->incl_ciclos;
    p->seno = e->incl_seno, p->cosseno = e->incl_cosseno;
    double Jm = f->I2 / f->m2 + f->l2 * f->l2;    /* J/m2 do pêndulo em torno do pivô */
    p->wn = sqrt(f->g * f->l2 / Jm);
    p->k = f->L1 * f->l2 / Jm;                    /* φ̈ = −ωn² φ − k θ̈1 (pêndulo embaixo, braço acelerando) */
}

bool embarcado_inclinacao_define(double seno, double cosseno)
{
    if (s_ativo) return false;
    s_cfg.e.incl_seno = seno, s_cfg.e.incl_cosseno = cosseno;
    nvs_handle_t h;
    double sc[2] = {seno, cosseno};
    bool gravou = nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK;
    if (gravou) {
        gravou = nvs_set_blob(h, NVS_INCL, sc, sizeof(sc)) == ESP_OK && nvs_commit(h) == ESP_OK;
        nvs_close(h);
    }
    if (!gravou) saida_aviso("ATENÇÃO: não deu para gravar a inclinação na flash (vale só até o reset)");
    return true;                                  // o trim (offset do topo) fica: não é a inclinação da mesa
}

int embarcado_lado_auto(void)
{
    const ensaio *e = &s_cfg.e;
    if (e->lado_auto == 0 || e->trim_valor == 0) return 0;
    return (e->trim_valor > 0 ? 1 : -1) * (e->lado_auto > 0 ? 1 : -1);
}

double embarcado_offset_topo(void) { return s_cfg.e.trim_valor; }

bool embarcado_ultimo_resultado(ensaio_resultado *r)
{
    taskENTER_CRITICAL(&s_mux_res);
    *r = s_ultimo;
    taskEXIT_CRITICAL(&s_mux_res);
    return r->valido;
}

void embarcado_ensaio_atual(int *lado, double *duracao)
{
    *lado = s_cfg.e.lado;
    *duracao = s_cfg.e.duracao;
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

/* Zero do θ2 pela oscilação (ident.c). Se o repouso no -start discordar mais que isso, o encoder foi mexido. */
#define ZERO_TOLERA 0.05
#define PARADO_FAIXA 0.01   /* start: θ2 parado = variou menos que isso em 1 s [rad] (~6 LSB do AS5600) */
static bool s_zero_calibrado = false;
void embarcado_zero_calibrado(void) { s_zero_calibrado = true; }

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
    double a_pend = 0, t_quieto = -1, t_espera = 0, soma_th2 = 0, th2_min = 0, th2_max = 0;
    // resumo do equilíbrio (|θ2| que o controle vê < 0,15 rad): θ1 médio e faixa, rms θ2, quedas
    double eq_t1 = 0, eq_t1_min = 1e9, eq_t1_max = -1e9, eq_th2q = 0;
    int eq_n = 0, eq_quedas = 0;
    double t_topo = -1;                           // 1ª vez com |θ2| < 0,15 rad (a subida) [s]
    double temp_ini = NAN;                        // driver na partida [°C]
    bool solto = false;                           // motor solto no repouso (ensaio.repouso_solta)
    int n_soltas = 0;
    double zero_rep = NAN;                        // repouso − zero em uso no start [rad]
    bool zero_osc = false;                        // o zero veio da oscilação (-calibra/-inclina/-auto)
    bool eq_dentro = false;
    int n_th2 = 0;
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
            } else {
                // parado embaixo por 1 s. Vale qualquer ângulo: o zero sai daqui. Pendurado e parado, o pêndulo aponta
                // para baixo pela gravidade, então θ2 = π (+ a inclinação da mesa neste θ1) é a média dessas leituras
                // (não uma leitura só, como o -zera).
                // Parado = o θ2 variou menos de PARADO_FAIXA no último 1 s. Pelo dθ2 (|dθ2| < 0,1 rad/s) às vezes
                // nunca partia: parado na divisa entre duas contagens do AS5600, a leitura alterna 1 LSB (0,0015
                // rad) e o observador de 40 Hz transforma isso em picos de até ~0,8 rad/s, que zeravam a espera.
                if (t_quieto >= 0) {
                    th2_min = fmin(th2_min, x[1]), th2_max = fmax(th2_max, x[1]);
                    if (th2_max - th2_min > PARADO_FAIXA) t_quieto = -1;   // mexeu: recomeça daqui
                }
                if (t_quieto < 0) {
                    t_quieto = t_espera;
                    th2_min = th2_max = x[1];
                    soma_th2 = 0, n_th2 = 0;
                }
                soma_th2 += x[1], n_th2++;
                parte = t_espera - t_quieto > 1.0;
                if (parte) {
                    double desloc = PI_D + controlador_inclinacao(e, x[0]) - soma_th2 / n_th2;
                    // sem as voltas inteiras: depois de uma queda que deu a volta o θ2 contínuo acumula ±2π, e a
                    // diferença de "−360°" descartava o zero da oscilação a cada rodada (lote 15/11: zero do repouso,
                    // ±1°, em quase todas as subidas)
                    desloc -= 2 * PI_D * floor(desloc / (2 * PI_D) + 0.5);
                    zero_rep = -desloc;
                    zero_osc = s_zero_calibrado && fabs(desloc) < ZERO_TOLERA;
                    if (s_zero_calibrado && fabs(desloc) < ZERO_TOLERA) {   // a oscilação é melhor que o repouso
                        saida_aviso("zero do θ2 da calibração mantido (o repouso difere %+.2f°)", desloc * 180 / PI_D);
                    } else {
                        if (s_zero_calibrado) s_zero_calibrado = false;     // mexeram no encoder: calibração perdida
                        controle_desloca_pendulo((float)desloc);
                        x[1] += desloc;
                        saida_aviso("zero do θ2 pela média de %d leituras parado: corrigido em %+.4f rad (%+.2f°)%s",
                                    n_th2, desloc, desloc * 180 / PI_D,
                                    s_zero_calibrado ? "" : " (furuta -calibra fica mais exato)");
                    }
                }
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
            char txt[512];
            resumo(&cfg, txt, sizeof(txt));       // o que está sendo usado de fato (θ1 do homing)
            temp_ini = termopar_temp();
            saida_aviso("%s ligado (%d amostras na criação), driver %.1f °C\n%s", modo == 1 ? "controle" : "a seco",
                        perdidas, (double)termopar_temp(), txt);
            continue;                             // a amostra já ficou velha: começa na próxima
        }
        int64_t t0 = esp_timer_get_time();
        mpc_info in;
        double a = controlador_passo(&s_ctl, x, k * e->Ts, &in);
        float t_calc = (esp_timer_get_time() - t0) * 1e-3f;
        a_pend = a;
        // repouso com o motor solto: só com o braço já parado (dθ1 comandada ~0); sai na hora em que o repouso sai
        if (modo == 1 && e->repouso_solta) {
            bool solta = s_ctl.repousando && fabs(x[2]) < 0.02;
            if (solta && !solto) n_soltas++;
            solto = solta;
            controle_solta_braco(solta);
        }
        if (ulTaskNotifyValueClear(NULL, 0) > 0) {   // passou do período: aplica já e conta o estouro
            estouros++;
            if (modo == 1) controle_acel((float)a_pend);
        }
        telemetria_t tl = {(float)(k * e->Ts), {(float)x[0], (float)x[1], (float)x[2], (float)x[3]}, (float)s_ctl.u,
                           (float)a, t_calc, (unsigned char)in.n_qp, (unsigned char)in.n_it,
                           (unsigned short)(estouros > 65535 ? 65535 : estouros), (float)s_ctl.trim};
        xQueueSend(s_fila_tel, &tl, 0);           // fila cheia: perde a amostra, não atrasa o controle
        double th_eq = wrap(x[1]);                // x[1]: como o controle viu (sem o offset do topo e a mesa)
        if (fabs(th_eq) < 0.15) {
            if (t_topo < 0) t_topo = k * e->Ts;
            eq_t1 += x[0], eq_th2q += th_eq * th_eq, eq_n++;
            eq_t1_min = fmin(eq_t1_min, x[0]), eq_t1_max = fmax(eq_t1_max, x[0]);
            eq_dentro = true;
        } else if (eq_dentro && fabs(th_eq) > 0.5) {
            eq_quedas++;
            eq_dentro = false;
        }
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
    controle_solta_braco(false);
    // Driver desligado na hora, sem frear: freando e depois parado em READY (até 10 s, DESLIGA_OCIOSO_MS) ele fica
    // com a corrente cheia, que parado não é limitada pela força contraeletromotriz, e o driver dava um degrau até
    // 70 e poucos °C (14/11). O braço fica solto: o próximo start pede o homing de novo, como já pedia.
    bool desligou = modo == 1 && controle_estado() != EST_OCIOSO && controle_estado() != EST_FALHA;
    if (desligou) controle_habilita(false);
    if (criado && e->trim) {                      // o próximo ensaio começa do trim aprendido (cfg -reset zera)
        s_cfg.e.trim_valor = s_ctl.trim;
        saida_aviso("trim do zero do θ2: %+.4f rad (%+.2f°)%s", s_ctl.trim, s_ctl.trim * 180 / PI_D,
                    fabs(s_ctl.trim) >= e->trim_max ? " - no limite trim.max: confira o zero (furuta -calibra)" : "");
    }
    if (criado && !e->equilibrio)                 // o estado em que o impulso entregou o pêndulo ao MPC
        saida_aviso("subida: impulso %.0f ms, energia %.3f, θ1 %+.3f θ2 %+.3f dθ1 %+.2f dθ2 %+.2f na passagem; "
                    "topo em %.2f s; zero %s (repouso %+.2f°)",
                    s_ctl.imp_t * 1e3, s_ctl.imp_e, s_ctl.imp_x[0], s_ctl.imp_x[1], s_ctl.imp_x[2],
                    s_ctl.imp_x[3], t_topo, zero_osc ? "da oscilação" : "do repouso", zero_rep * 180 / PI_D);
    int rerefs = criado ? s_ctl.rerefs : 0;
    double t_repouso = criado ? s_ctl.n_repouso * e->Ts : 0;
    if (criado) controlador_libera(&s_ctl);
    if (eq_n > 0)
        saida_aviso("equilíbrio: %.1f s no topo (%d%%), θ1 médio %+.3f rad (faixa %+.2f..%+.2f), rms θ2 %.2f°, %d quedas, "
                    "%d re-referências, %.1f s em repouso (motor solto %d vezes), driver %.1f °C",
                    eq_n * e->Ts, k ? 100 * eq_n / k : 0, eq_t1 / eq_n, eq_t1_min, eq_t1_max,
                    sqrt(eq_th2q / eq_n) * 180 / PI_D, eq_quedas, rerefs, t_repouso, n_soltas, (double)termopar_temp());
    else if (criado)
        saida_aviso("equilíbrio: não chegou no topo");
    if (modo == 1) {
        ensaio_resultado r = {true, e->lado, criado, t_topo, eq_quedas, k ? 100 * eq_n / k : 0, k * e->Ts,
                              temp_ini, (double)termopar_temp(), motivo};
        taskENTER_CRITICAL(&s_mux_res);
        s_ultimo = r;
        taskEXIT_CRITICAL(&s_mux_res);
    }
    saida_aviso("fim do ensaio: %s (%.1f s, %u estouros)%s", motivo, k * e->Ts, estouros,
                desligou ? "; driver desligado (braço solto: refaça o homing)" : "");
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
    char txt[512];
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
                           (unsigned short)(estouros > 65535 ? 65535 : estouros), (float)s_ctl.trim};
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
