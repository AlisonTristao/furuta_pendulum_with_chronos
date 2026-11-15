#include "controlador.h"

#include <math.h>

#define PI_D 3.14159265358979323846

double controlador_inclinacao(const ensaio *e, double theta1)
{
    return e->incl_seno * sin(theta1) + e->incl_cosseno * (cos(theta1) - 1);
}

double controlador_energia(const controlador *k, const double x[4])
{
    const modelo *m = &k->m;   /* E = δ θ̇2²/2 + η (1 + cos θ2): 0 embaixo, 2η no topo */
    return (0.5 * m->delta * x[3] * x[3] + m->eta * (1 + cos(x[1]))) / (2 * m->eta);
}

int controlador_cria(controlador *k, const config *cfg, const double x0[4])
{
    k->cfg = *cfg;
    const ensaio *e = &k->cfg.e;
    k->m = modelo_de_fisico(k->cfg.f, e->Ts, k->cfg.mpc.dtheta_max);
    /* θ2 no múltiplo de 2π mais próximo do topo (equilíbrio) ou de embaixo (swing-up); depois fica contínuo */
    k->off2 = e->equilibrio ? 2 * PI_D * floor((x0[1] + PI_D) / (2 * PI_D)) : 2 * PI_D * floor(x0[1] / (2 * PI_D));
    /* lado −1: parte de −π (o MPC sobe pelo outro lado, o espelho do +π) */
    if (!e->equilibrio && e->lado < 0) k->off2 += 2 * PI_D;
    k->trim = e->trim_valor;                      /* offset do topo: vale sempre; trim.liga só decide se aprende */
    k->t_dentro = 0;
    double x[4] = {x0[0], x0[1] - k->off2 - k->trim - controlador_inclinacao(e, x0[0]), x0[2], x0[3]};
    k->c = mpc_cria(&k->cfg.mpc, &k->m, x, 0.0);
    if (!k->c) return -1;
    estimador_cria(&k->est, e->est_L1, e->est_L2);
    k->a_ap = k->a_int = 0;
    k->u = 0;
    k->imp_ativo = !e->equilibrio && e->impulso_ms > 0;
    k->imp_t = 0, k->imp_e = 0;
    k->capturou = 0, k->rerefs = 0;
    k->repousando = 0, k->n_repouso = 0;
    for (int i = 0; i < 4; i++) k->imp_x[i] = 0;
    return 0;
}

void controlador_libera(controlador *k)
{
    mpc_libera(k->c);
    k->c = 0;
}

/* Trim do zero do θ2 (controlador.h). x: o que o controle viu neste passo (θ2 já sem off2 e trim). */
static void trim_passo(controlador *k, const double x[4], double u, int finito)
{
    const ensaio *e = &k->cfg.e;
    const mpc_config *m = &k->cfg.mpc;
    int dentro = finito && fabs(x[1]) < e->trim_janela && fabs(x[3]) < 2.0 && u < 0.95 * (double)m->u_max &&
                 u > 0.95 * (double)m->u_min;
    k->t_dentro = dentro ? k->t_dentro + e->Ts : 0;
    if (k->t_dentro < 1.0) return;                 /* espera assentar depois de pegar o topo */
    k->trim += e->Ts / e->trim_tau * x[1];
    if (k->trim > e->trim_max) k->trim = e->trim_max;
    if (k->trim < -e->trim_max) k->trim = -e->trim_max;
}

double controlador_passo(controlador *k, double x[4], double t, mpc_info *in)
{
    const ensaio *e = &k->cfg.e;
    x[1] -= k->off2 + k->trim + controlador_inclinacao(e, x[0]);
    /* re-referência (controlador.h): caindo (θ2·dθ2 > 0, com o dθ2 do estimador do passo anterior) além de
     * reref_angulo. O estimador pula junto, senão vira um pico de velocidade. */
    if (e->reref && k->capturou && !k->imp_ativo) {
        double d2 = e->estimador ? k->est.x[3] : x[3];
        if (fabs(x[1]) > e->reref_angulo && x[1] * d2 > 0) {
            double s = x[1] > 0 ? 1 : -1;
            k->off2 += 2 * PI_D * s;
            x[1] -= 2 * PI_D * s;
            k->est.x[1] -= 2 * PI_D * s;
            k->rerefs++;
            k->capturou = 0;                      /* arma de novo só no topo novo */
        }
    }
    if (e->estimador) CHRONOS_T(13, estimador_passo(&k->est, &k->m, x, k->a_int, e->Ts));   /* x[3] = estimado */
    double xc[4] = {x[0], x[1], x[2], x[3]};
    if (e->comp_atraso_ms > 0) CHRONOS_T(14, modelo_passo_acel_ctl(&k->m, xc, k->a_ap, e->comp_atraso_ms * 1e-3, 4));
    /* banda morta no equilíbrio: perto do topo o atrito seco do pivô segura o pêndulo PARADO (|θ2| < Tc/(m g l),
     * ~0,013 rad com tc2 2e-4), e corrigir erro dessa ordem só alimenta o ciclo limite do braço. Só com o ângulo E a
     * velocidade pequenos (parado de fato) o MPC vê o pêndulo no topo; mexeu, volta inteiro. Uma banda só no ângulo
     * (θ2 − banda, 15/11) piorou na sim: rms θ2 1,1° -> 2,9° com 0,02 rad, porque com o braço mexendo o pêndulo
     * nunca está parado e a banda só atrasa a correção. Só depois de capturar o topo. */
    if (e->banda_th2 > 0 && e->banda_dth2 > 0 && k->capturou && fabs(xc[1]) < e->banda_th2 &&
        fabs(xc[3]) < e->banda_dth2 && (e->banda_dth1 <= 0 || fabs(xc[2]) < e->banda_dth1))
        xc[1] = 0, xc[3] = 0;
    double ref[4] = {t >= e->t_degrau ? e->theta1_degrau : e->theta1_ref, 0, 0, 0};
    double u = mpc_passo(k->c, xc, ref, in);
    double a = in->finito ? modelo_acel_braco(&k->m, xc, u) : 0.0;
    /* impulso inicial: tira o pêndulo do repouso (parado embaixo o MPC demora a "pegar"). O MPC segue calculando
     * (warm start) e o estimador usa o aplicado. +a na 1ª metade de impulso_ms, -a na 2ª: o braço começa e termina
     * parado. Malha aberta de propósito (14/11): ele quase não injeta energia (0,02-0,07 do topo) e entrega o braço
     * parado, então o MPC recebe sempre o mesmo estado, mesmo com o pêndulo balançando um pouco; um impulso que
     * bombeava energia por realimentação (a = −A sinal(cos θ2 θ̇2)) entregava o braço a 6-13 rad/s e subia em 0,6-3,3 s
     * na simulação, contra 0,76-0,84 s deste. Na passagem guarda o estado (imp_*) para o log da subida. */
    if (k->imp_ativo) {
        double en = controlador_energia(k, x);
        int fim = t >= e->impulso_ms * 1e-3 - 1e-9;
        a = (t < e->impulso_ms * 0.5e-3 - 1e-9 ? e->impulso_a : -e->impulso_a) * (e->lado < 0 ? -1 : 1);
        if (fim) {
            k->imp_ativo = 0;
            k->imp_t = t, k->imp_e = en;
            for (int i = 0; i < 4; i++) k->imp_x[i] = x[i];
            a = in->finito ? modelo_acel_braco(&k->m, xc, u) : 0.0;   /* o MPC assume já neste passo */
        } else {
            u = 0;
        }
    }
    if (!k->capturou && fabs(x[1]) < e->janela_equilibrio && fabs(x[3]) < 2.0) k->capturou = 1;
    /* modo repouso (controlador.h) */
    if (e->repouso && k->capturou && !k->imp_ativo && e->banda_th2 > 0 && e->banda_dth2 > 0) {
        if (k->repousando)
            k->repousando = fabs(x[1]) < 1.5 * e->banda_th2 && fabs(x[3]) < 2 * e->banda_dth2;
        else
            k->repousando = fabs(x[1]) < e->banda_th2 && fabs(x[3]) < e->banda_dth2 && fabs(x[2]) < e->repouso_dth1;
        if (k->repousando) {
            a = -x[2] / e->Ts;                    /* para o braço em um período, no máximo repouso_a */
            if (a > e->repouso_a) a = e->repouso_a;
            if (a < -e->repouso_a) a = -e->repouso_a;
            k->n_repouso++;
        }
    }
    if (e->trim) trim_passo(k, x, u, in->finito);
    k->u = u;
    k->a_int = k->a_ap;
    k->a_ap = a;
    return a;
}
