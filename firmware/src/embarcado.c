#include "embarcado.h"

#include "controlador.h"
#include "controle.h"
#include "protocolo.h"

#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PI_D 3.14159265358979323846

typedef struct {
    float t, x[4], u, a, t_calc_ms;
    unsigned char n_qp, n_it;
    unsigned short estouros;
} telemetria_t;

typedef enum { MODO_PARADO, MODO_ESPERA, MODO_RODANDO } fase_t;

static config s_cfg;                        // configuração recebida (P...), copiada no início do ensaio
static TaskHandle_t s_tarefa_mpc;
static QueueHandle_t s_fila_tel;
static volatile int s_pedido = 0;           // 0 = nada, 1 = controle, 2 = a seco, -1 = desliga
static volatile bool s_ativo = false;
static controlador s_ctl;

bool embarcado_config(const char *par)
{
    if (s_ativo) return false;
    char b[160];
    strncpy(b, par, sizeof(b) - 1);
    b[sizeof(b) - 1] = '\0';
    return config_arg_silencioso(&s_cfg, b) == 0;
}

bool embarcado_ativo(void) { return s_ativo; }

const char *embarcado_liga(int modo)
{
    if (s_ativo) return "JA_ATIVO";
    if (modo == 1 && controle_estado() != EST_PRONTO) return "FACA_O_HOMING";
    if (modo != 1 && modo != 2) return "MODO";
    s_pedido = modo;
    xTaskNotifyGive(s_tarefa_mpc);
    return NULL;
}

void embarcado_desliga(void)
{
    if (s_ativo) s_pedido = -1;
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
#ifdef CHRONOS_PERFIL
    chronos_relogio = relogio_us;
    memset(chronos_perfil, 0, sizeof(chronos_perfil));
#endif
    int ts_ms = (int)lround(e->Ts * 1e3);
    controle_assina_amostra(s_tarefa_mpc, ts_ms);
    ulTaskNotifyTake(pdTRUE, 0);

    fase_t fase = modo == 1 ? MODO_ESPERA : MODO_RODANDO;
    bool criado = false;
    double a_pend = 0, t_quieto = -1, t_espera = 0;
    int k = 0, K = (int)lround(e->duracao / e->Ts);
    unsigned estouros = 0;
    const char *motivo = "fim da duração";
    if (modo == 1) protocolo_envia("M,ESPERA");
    for (;;) {
        if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500))) {
            motivo = "sem amostra da malha";
            break;
        }
        if (s_pedido == -1) {
            motivo = "desligado (M0)";
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

        if (fase == MODO_ESPERA) {                // braço parado no meio até a condição de partida
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
            fase = MODO_RODANDO;
        }
        if (!criado) {                            // cria na partida (~0,1 s: DLQR em double); a_pend = 0 até lá
            if (controlador_cria(&s_ctl, &cfg, x) != 0) {
                motivo = "falha ao criar o MPC";
                break;
            }
            criado = true;
            mpc_orcamento(s_ctl.c, relogio_us, ts_ms * 1000LL - FOLGA_US);   /* sem estouro: corta o laço dos rho */
            protocolo_envia("M,LIGADO,%d", (int)ulTaskNotifyTake(pdTRUE, 0));   // amostras perdidas na criação
            {   // o que o ESP32 está usando de fato (confere a configuração mandada com P...)
                const mpc_config *m = &cfg.mpc;
                protocolo_envia("M,CFG,Np=%d,horizonte=%.3f,Q=%g/%g/%g/%g,Rdu=%g,u=%.3f/%.3f,peso_exp=%.2f,l_max=%d,"
                                "theta1=%.3f,l2=%.4f,I2=%.4e,mu2=%.4e,comp=%.0f,Ts=%.3f",
                                m->Np, (double)m->horizonte, (double)m->Q[0], (double)m->Q[1], (double)m->Q[2],
                                (double)m->Q[3], (double)m->Rdu, (double)m->u_min, (double)m->u_max,
                                (double)m->peso_exp, m->l_max, (double)m->theta1_max, cfg.f.l2, cfg.f.I2, cfg.f.mu2,
                                e->comp_atraso_ms, e->Ts);
            }
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
        xQueueSend(s_fila_tel, &tl, 0);           // fila cheia: perde a linha, não atrasa o controle
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
    // sem comando novo: o watchdog da malha (200 ms) freia o braço até parar, como com o PC
    while (uxQueueMessagesWaiting(s_fila_tel)) vTaskDelay(1);
    vTaskDelay(pdMS_TO_TICKS(5));                // a última linha de telemetria sai antes do M,FIM
#ifdef CHRONOS_PERFIL                             // MPC_PERFIL=1: tempo por etapa (µs por período)
    protocolo_envia("M,PERFIL,grad %lld,res_ig %lld,res_des %lld,KKT %lld,Riccati %lld,folgas %lld,lambda2 %lld,"
                    "busca %lld,get_var %lld,ZOH %lld,rollout %lld,rho %lld",
                    chronos_perfil[0] / (k ? k : 1), chronos_perfil[1] / (k ? k : 1), chronos_perfil[2] / (k ? k : 1),
                    chronos_perfil[3] / (k ? k : 1), chronos_perfil[4] / (k ? k : 1), chronos_perfil[5] / (k ? k : 1),
                    chronos_perfil[6] / (k ? k : 1), chronos_perfil[7] / (k ? k : 1), chronos_perfil[8] / (k ? k : 1),
                    chronos_perfil[9] / (k ? k : 1), chronos_perfil[10] / (k ? k : 1), chronos_perfil[11] / (k ? k : 1));
#endif
    protocolo_envia("M,FIM,%u,%s", estouros, motivo);
    return motivo;
}

static void tarefa_mpc(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int modo = s_pedido;
        if (modo != 1 && modo != 2) continue;
        s_ativo = true;
        roda_ensaio(modo);
        s_pedido = 0;
        s_ativo = false;
    }
}

static void tarefa_tel(void *arg)
{
    telemetria_t tl;
    for (;;) {
        if (!xQueueReceive(s_fila_tel, &tl, portMAX_DELAY)) continue;
        protocolo_envia("T,%.3f,%.5f,%.5f,%.4f,%.4f,%.5f,%.3f,%u,%u,%.3f,%u", (double)tl.t, (double)tl.x[0],
                        (double)tl.x[1], (double)tl.x[2], (double)tl.x[3], (double)tl.u, (double)tl.a, tl.n_qp,
                        tl.n_it, (double)tl.t_calc_ms, tl.estouros);
    }
}

void embarcado_init(void)
{
    config_le(NULL, &s_cfg);                      // padrões; o malha.exe manda o furuta.ini com P...
    s_fila_tel = xQueueCreate(64, sizeof(telemetria_t));
    xTaskCreatePinnedToCore(tarefa_tel, "telemetria", 4096, NULL, 4, NULL, 0);
    // núcleo 1 só para o MPC
    xTaskCreatePinnedToCore(tarefa_mpc, "mpc", 12288, NULL, configMAX_PRIORITIES - 2, &s_tarefa_mpc, 1);
}
