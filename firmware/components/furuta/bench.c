// Benchmark do MPC (furuta -bench): malha fechada simulada no núcleo 1 com a configuração em uso (cfg -set), o
// mesmo controlador do ensaio (comum/controlador.c: estimador, compensação de atraso, impulso, MPC em float) e a
// planta em double comandada em θ̈1 com o atraso de um período. Sem o corte de orçamento do ensaio: mede o
// custo inteiro de cada passo. Não mexe no motor.
#include "controlador.h"
#include "saida.h"

#include <math.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PASSOS_MAX 1500   // 30 s a 50 Hz

static int cmp_i32(const void *a, const void *b)
{
    int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
    return (x > y) - (x < y);
}

void bench_roda(const config *cfg)
{
    static int32_t t[PASSOS_MAX];
    const ensaio *e = &cfg->e;
    int n = (int)lround(e->duracao / e->Ts);
    if (n > PASSOS_MAX) n = PASSOS_MAX;
    if (n < 10) n = 10;
    modelo planta = modelo_de_fisico(cfg->f, e->Ts, cfg->mpc.dtheta_max);
    double x[4] = {0, 3.14159265358979 + 0.02, 0, 0};
    saida_aviso("bench: %d passos (%.1f s) de swing-up simulado, núcleo %d, MPC %s", n, n * e->Ts, xPortGetCoreID(),
                MPC_OPT_STR);

    size_t heap0 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int64_t t0 = esp_timer_get_time();
    controlador ctl;
    if (controlador_cria(&ctl, cfg, x) != 0) {
        saida_aviso("bench: falha ao criar o MPC");
        return;
    }
    int64_t t_cria = esp_timer_get_time() - t0;
    size_t heap = heap0 - heap_caps_get_free_size(MALLOC_CAP_8BIT);

    int topo = 0, it_max = 0, nao_finito = 0, estouros = 0, qp_max = 0;
    long it_soma = 0;
    double a_ap = 0, th1_max = 0;
    const int32_t ts_us = (int32_t)lround(e->Ts * 1e6);
    for (int k = 0; k < n; k++) {
        double xm[4] = {x[0], x[1], x[2], x[3]};   // o controlador troca dθ2 pelo estimado
        mpc_info in;
        t0 = esp_timer_get_time();
        double a = controlador_passo(&ctl, xm, k * e->Ts, &in);
        t[k] = (int32_t)(esp_timer_get_time() - t0);
        estouros += t[k] > ts_us;
        it_soma += in.n_it;
        if (in.n_it > it_max) it_max = in.n_it;
        if (in.n_qp > qp_max) qp_max = in.n_qp;
        nao_finito += !in.finito;
        modelo_passo_acel(&planta, x, a_ap, e->Ts, 10);   // o comando de k entra em k+1
        a_ap = a;
        double th = x[1] - 6.283185307179586 * floor((x[1] + 3.14159265358979) / 6.283185307179586);
        topo += fabs(th) < 0.35;
        if (fabs(x[0]) > th1_max) th1_max = fabs(x[0]);
        if ((k & 15) == 15) vTaskDelay(1);   // deixa o IDLE do núcleo 1 rodar
    }
    controlador_libera(&ctl);

    int64_t soma = 0;
    for (int k = 0; k < n; k++) soma += t[k];
    qsort(t, n, sizeof(int32_t), cmp_i32);
    saida_aviso("bench: passo médio %.2f ms, p50 %.2f, p99 %.2f, máx %.2f (Ts %.0f ms, %d acima) | QPs máx %d | "
                "Newton médio %.1f máx %d | cria %.1f ms, heap %u B | topo %d%% | |θ1| máx %.2f rad | não finito %d",
                soma / 1000.0 / n, t[n / 2] / 1000.0, t[n * 99 / 100] / 1000.0, t[n - 1] / 1000.0, e->Ts * 1e3,
                estouros, qp_max, (double)it_soma / n, it_max, t_cria / 1000.0, (unsigned)heap, 100 * topo / n,
                th1_max, nao_finito);
}
