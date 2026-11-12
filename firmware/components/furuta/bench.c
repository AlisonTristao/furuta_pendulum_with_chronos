// Benchmark do MPC (furuta -bench): malha fechada simulada no núcleo 1 com a configuração em uso (cfg -set), o
// mesmo controlador do ensaio (comum/controlador.c: estimador, compensação de atraso, impulso, MPC em float) e a
// planta em double comandada em θ̈1 com o atraso de um período. Sem o corte de orçamento do ensaio: mede o
// custo inteiro de cada passo. Não mexe no motor.
//
// Imprime o tempo por etapa (perfil do CHRONOS, chronos.h, medido em ciclos da CPU): média e máximo por passo e a
// fração do passo. Para comparar variantes, troque a configuração e rode de novo, ex.:
//   cfg -set mpc.discretizacao=rk4   (zoh | rk4 | euler)      cfg -set mpc.Np=10      cfg -set mpc.l_max=3
#include "controlador.h"
#include "saida.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define PASSOS_MAX 1500   // 30 s a 50 Hz
#define MHZ CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ

static unsigned ciclos(void) { return (unsigned)esp_cpu_get_cycle_count(); }

static int cmp_i32(const void *a, const void *b)
{
    int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
    return (x > y) - (x < y);
}

// linhas da tabela: etapa do perfil (0..14), QP (soma 0..8), outros (passo - soma) ou total
enum { L_QP = 100, L_OUTROS, L_TOTAL };
typedef struct {
    int id;
    const char *nome;
} linha_t;

static const linha_t LINHAS[] = {
    {13, "estimador de dθ2"},
    {14, "compensação do atraso"},
    {12, "warm start + ρ inicial"},
    {9, "predição A,B por estágio"},
    {10, "rollout + folgas iniciais"},
    {L_QP, "QP (CHRONOS, todas as iterações)"},
    {0, "  gradiente"},
    {1, "  resíduos de igualdade"},
    {2, "  resíduos de desigualdade"},
    {3, "  elementos do KKT"},
    {4, "  Riccati"},
    {5, "  folgas"},
    {6, "  lambda2"},
    {7, "  busca de linha"},
    {8, "  variáveis (get_var)"},
    {11, "atualização dos ρ"},
    {L_OUTROS, "outros (cópias, testes, u)"},
    {L_TOTAL, "TOTAL do passo"},
};
#define NL ((int)(sizeof(LINHAS) / sizeof(LINHAS[0])))

static long long valor(int id, const long long *d, long long total)
{
    if (id == L_TOTAL) return total;
    long long qp = 0, soma = 0;
    for (int i = 0; i <= 8; i++) qp += d[i];
    if (id == L_QP) return qp;
    for (int i = 0; i < CHRONOS_NPERFIL; i++) soma += d[i];
    if (id == L_OUTROS) return total - soma > 0 ? total - soma : 0;
    return d[id];
}

void bench_roda(const config *cfg)
{
    static int32_t t[PASSOS_MAX];
    static long long soma_l[NL], max_l[NL];
    const ensaio *e = &cfg->e;
    const mpc_config *mc = &cfg->mpc;
    int n = (int)lround(e->duracao / e->Ts);
    if (n > PASSOS_MAX) n = PASSOS_MAX;
    if (n < 10) n = 10;
    modelo planta = modelo_de_fisico(cfg->f, e->Ts, mc->dtheta_max);
    double x[4] = {0, 3.14159265358979 + 0.02, 0, 0};
    saida_aviso("bench: Np %d, horizonte %.2f s, discretização %s, l_max %d, max_iter %d | %d passos (%.1f s) de "
                "swing-up simulado, núcleo %d, MPC %s",
                mc->Np, (double)mc->horizonte, modelo_disc_nome(mc->discretizacao), mc->l_max, mc->op.max_iter, n,
                n * e->Ts, xPortGetCoreID(), MPC_OPT_STR);

    size_t heap0 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    unsigned c0 = ciclos();
    controlador ctl;
    if (controlador_cria(&ctl, cfg, x) != 0) {
        saida_aviso("bench: falha ao criar o MPC");
        return;
    }
    double t_cria = (unsigned)(ciclos() - c0) / (double)MHZ / 1000.0;
    size_t heap = heap0 - heap_caps_get_free_size(MALLOC_CAP_8BIT);

    memset(soma_l, 0, sizeof(soma_l));
    memset(max_l, 0, sizeof(max_l));
    memset(chronos_perfil, 0, sizeof(chronos_perfil));
    long long ant[CHRONOS_NPERFIL] = {0}, d[CHRONOS_NPERFIL];
    chronos_relogio = ciclos;

    int topo = 0, it_max = 0, nao_finito = 0, estouros = 0, qp_max = 0;
    long it_soma = 0, qp_soma = 0;
    double a_ap = 0, th1_max = 0;
    const int32_t ts_us = (int32_t)lround(e->Ts * 1e6);
    for (int k = 0; k < n; k++) {
        double xm[4] = {x[0], x[1], x[2], x[3]};   // o controlador troca dθ2 pelo estimado
        mpc_info in;
        c0 = ciclos();
        double a = controlador_passo(&ctl, xm, k * e->Ts, &in);
        long long total = (unsigned)(ciclos() - c0);
        t[k] = (int32_t)(total / MHZ);
        for (int i = 0; i < CHRONOS_NPERFIL; i++) d[i] = chronos_perfil[i] - ant[i], ant[i] = chronos_perfil[i];
        for (int l = 0; l < NL; l++) {
            long long v = valor(LINHAS[l].id, d, total);
            soma_l[l] += v;
            if (v > max_l[l]) max_l[l] = v;
        }
        estouros += t[k] > ts_us;
        it_soma += in.n_it, qp_soma += in.n_qp;
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
    chronos_relogio = NULL;
    controlador_libera(&ctl);

    long long tot = soma_l[NL - 1] > 0 ? soma_l[NL - 1] : 1;
    saida_printf("%-34s %10s %10s %7s\n", "etapa", "média µs", "máx µs", "%");
    for (int l = 0; l < NL; l++)
        saida_printf("%-34s %10.1f %10.1f %6.1f%%\n", LINHAS[l].nome, (double)soma_l[l] / n / MHZ,
                     (double)max_l[l] / MHZ, 100.0 * soma_l[l] / tot);
    saida_printf("(máx: o maior valor da etapa num passo; QPs por passo: média %.2f, máx %d -> predição, rollout, QP e "
                 "ρ rodam uma vez por QP)\n",
                 (double)qp_soma / n, qp_max);

    int64_t soma = 0;
    for (int k = 0; k < n; k++) soma += t[k];
    qsort(t, n, sizeof(int32_t), cmp_i32);
    saida_aviso("bench: passo médio %.2f ms, p50 %.2f, p99 %.2f, máx %.2f (Ts %.0f ms, %d acima) | QPs médio %.2f "
                "máx %d | Newton médio %.1f máx %d | cria %.1f ms, heap %u B | topo %d%% | |θ1| máx %.2f rad | não "
                "finito %d",
                soma / 1000.0 / n, t[n / 2] / 1000.0, t[n * 99 / 100] / 1000.0, t[n - 1] / 1000.0, e->Ts * 1e3,
                estouros, (double)qp_soma / n, qp_max, (double)it_soma / n, it_max, t_cria, (unsigned)heap,
                100 * topo / n, th1_max, nao_finito);
}
