// Benchmark do MPC no ESP32 (ambiente `bench` do platformio.ini): malha fechada simulada aqui mesmo (planta em
// double, comandada em torque) e o tempo de cada mpc_passo no núcleo 1, para várias configurações do MPC.
// Saída em texto na serial (921600). Mesmos parâmetros do pc/furuta.ini.
#include "qlpv_mpc.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART UART_NUM_0
#define PASSOS 750   // 15 s a 50 Hz: swing-up + equilíbrio

static void out(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart_write_bytes(UART, buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1);
}

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static mpc_config config_furuta(int Np, int l_max)
{
    mpc_config c = mpc_config_artigo();
    c.Np = Np, c.l_max = l_max;
    c.Q[0] = 10, c.Q[1] = 100, c.Q[2] = 5, c.Q[3] = 1;
    c.Rdu = 300;
    c.u_min = -0.75f, c.u_max = 0.75f;
    c.dtheta_max = 25;
    c.theta1_min = -1.46f, c.theta1_max = 1.46f;
    c.qv_theta1 = 1e5f;
    c.peso_exp = 3;
    c.peso_exp_estados[0] = 0;
    return c;
}

static void roda(int Np, int l_max, int64_t *t)
{
    fisico f = {.m2 = 0.01410, .L1 = 0.09145, .l2 = 0.09008, .I1 = 0.005, .I2 = 4.3796e-05,
                .mu1 = 0.001, .mu2 = 3.6494e-04, .g = 9.81};
    modelo m = modelo_de_fisico(f, 0.02, 25);
    mpc_config cfg = config_furuta(Np, l_max);
    double x[4] = {0, 3.14159265358979 + 0.02, 0, 0}, ref[4] = {0, 0, 0, 0};
    size_t heap0 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int64_t t0 = esp_timer_get_time();
    mpc *c = mpc_cria(&cfg, &m, x, 0.0);
    int64_t t_cria = esp_timer_get_time() - t0;
    size_t heap = heap0 - heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int topo = 0, it_max = 0, nao_finito = 0;
    long it_soma = 0;
    for (int k = 0; k < PASSOS; k++) {
        mpc_info in;
        t0 = esp_timer_get_time();
        double u = mpc_passo(c, x, ref, &in);
        t[k] = esp_timer_get_time() - t0;
        it_soma += in.n_it;
        if (in.n_it > it_max) it_max = in.n_it;
        nao_finito += !in.finito;
        modelo_passo(&m, x, u, 10);
        double th = x[1] - 6.283185307179586 * floor((x[1] + 3.14159265358979) / 6.283185307179586);
        topo += fabs(th) < 0.35;
        if ((k & 15) == 15) vTaskDelay(1);   // deixa o IDLE do núcleo 1 rodar
    }
    mpc_libera(c);
    int64_t soma = 0;
    for (int k = 0; k < PASSOS; k++) soma += t[k];
    qsort(t, PASSOS, sizeof(int64_t), cmp_i64);
    out("N%-2d l%d | mpc_passo: médio %6.2f ms, p50 %6.2f, p99 %6.2f, máx %6.2f | Newton médio %4.1f máx %2d | "
        "cria %5.1f ms, heap %5u B | topo %3d%% | não finito %d\r\n",
        Np, l_max, soma / 1000.0 / PASSOS, t[PASSOS / 2] / 1000.0, t[PASSOS * 99 / 100] / 1000.0,
        t[PASSOS - 1] / 1000.0, (double)it_soma / PASSOS, it_max, t_cria / 1000.0, (unsigned)heap,
        100 * topo / PASSOS, nao_finito);
}

static void tarefa(void *arg)
{
    static int64_t t[PASSOS];
    static const int cfgs[][2] = {{40, 5}, {20, 5}, {20, 3}, {20, 2}, {15, 2}};
    vTaskDelay(pdMS_TO_TICKS(500));
    for (;;) {
        out("\r\n=== benchmark do MPC (float, %s), núcleo %d, %d passos de swing-up simulado ===\r\n", MPC_OPT_STR,
            xPortGetCoreID(), PASSOS);
        for (size_t i = 0; i < sizeof(cfgs) / sizeof(cfgs[0]); i++) roda(cfgs[i][0], cfgs[i][1], t);
        out("=== fim (repete em 5 s) ===\r\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void bench(void)
{
    uart_config_t cfg = {.baud_rate = 921600, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_driver_install(UART, 1024, 4096, 0, NULL, 0);
    uart_param_config(UART, &cfg);
    xTaskCreatePinnedToCore(tarefa, "bench", 16384, NULL, 5, NULL, 1);
}
