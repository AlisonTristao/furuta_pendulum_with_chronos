#include "planta.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

/* normal(0,1) (Box-Muller), semente fixa: ensaios simulados repetíveis */
static double unif(planta *p)
{
    p->rng ^= p->rng << 13, p->rng ^= p->rng >> 7, p->rng ^= p->rng << 17;
    return ((p->rng >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}
static double gauss(planta *p) { return sqrt(-2 * log(unif(p))) * cos(2 * PI * unif(p)); }

void planta_cria(planta *p, const config *cfg)
{
    memset(p, 0, sizeof(*p));
    const ensaio *e = &cfg->e;
    p->e = *e;
    p->m = modelo_de_fisico(cfg->f, e->Ts, cfg->mpc.dtheta_max);
    p->m.tc2 = e->tc2;                           /* a planta pode ter atrito seco; o MPC não */
    if (e->mu2_planta > 0) p->m.mu2 = e->mu2_planta;
    p->rng = 88172645463325252ull;
    for (int i = 0; i < 4; i++) p->x[i] = e->x0[i];
    if (e->semente > 0) {                        /* ensaio diferente a cada semente */
        p->rng += 0x9E3779B97F4A7C15ull * (unsigned long long)e->semente;
        p->x[1] += 0.1 * (unif(p) - 0.5);
    }
    for (int i = 0; i < 4; i++) p->xm[i] = p->x[i];
    p->th_est = p->x[1], p->w_est = p->x[3];
    p->q = e->bits_enc > 0 ? 2 * PI / (1 << e->bits_enc) : 0;
    p->ppr = e->passos_rev > 0 ? e->passos_rev / (2 * PI) : 0;
    p->th1_med = p->th1c = p->x[0];
    p->v_fw = p->x[2];
    p->ws = 2 * PI * e->f_motor;                 /* ressonância do rotor (0 = braço rígido) */
}

void planta_passo(planta *p, double a)
{
    const ensaio *e = &p->e;
    double *x = p->x;
    int nms = (int)lround(e->Ts * 1e3), nat = e->embarcado ? nms : (int)lround(e->atraso_ms);
    if (nat > 200) nat = 200;
    for (int j = 0; j < nms; j++) p->fila[(p->tms + nat + j) & 255] = a;
    for (int j = 0; j < nms; j++) {
        /* malha de 1 kHz do firmware: satura a, integra v, freia antes do limite, satura v */
        double ac = p->fila[p->tms++ & 255], v0 = p->v_fw;
        if (e->a_max_fw > 0) ac = fmax(-e->a_max_fw, fmin(e->a_max_fw, ac));
        double v = v0 + ac * 1e-3;
        if (e->limite_fw > 0 && e->a_max_fw > 0) {
            double parada = v * v / (2 * e->a_max_fw);
            if (v > 0 && p->th1_med + parada >= e->limite_fw) v = fmax(v0 - e->a_max_fw * 1e-3, 0);
            if (v < 0 && p->th1_med - parada <= -e->limite_fw) v = fmin(v0 + e->a_max_fw * 1e-3, 0);
        }
        if (e->v_max_fw > 0) v = fmax(-e->v_max_fw, fmin(e->v_max_fw, v));
        p->v_fw = v;
        if (p->ws <= 0) {                        /* braço rígido: segue a velocidade comandada */
            modelo_passo_acel(&p->m, x, (v - v0) * 1e3, 1e-3, 1);
            x[2] = v;
            p->th1c = x[0];
        } else {                                 /* rotor = mola até a posição dos passos (ressonância) */
            for (int i = 0; i < 10; i++) {
                p->th1c += v * 1e-4;
                double alvo = p->ppr > 0 ? floor(p->th1c * p->ppr) / p->ppr : p->th1c;
                /* erro do micropasso: periódico a cada passo inteiro (200/volta) */
                alvo += e->ripple_passo * (2 * PI / 200) * sin(200 * alvo);
                double ar = p->ws * p->ws * (alvo - x[0]) + 2 * e->zeta_motor * p->ws * (v - x[2]);
                modelo_passo_acel(&p->m, x, ar, 1e-4, 1);
            }
        }
        /* θ1 medido = passos inteiros (acumulador do gerador de passos trunca); dθ1 = velocidade comandada */
        p->th1_med = p->ppr > 0 ? floor(p->th1c * p->ppr) / p->ppr : p->th1c;
        double ang = x[1] + (e->ruido_enc > 0 ? e->ruido_enc * p->q * gauss(p) : 0);
        double med = p->q > 0 ? p->q * floor(ang / p->q) : ang;
        if (e->obs_hz > 0) {
            double wn = 2 * PI * e->obs_hz, er = med - p->th_est;
            p->th_est += 1e-3 * (p->w_est + 2 * wn * er);
            p->w_est += 1e-3 * wn * wn * er;
        } else {
            p->w_est = x[3];
        }
        p->xm[0] = p->th1_med, p->xm[1] = med, p->xm[2] = p->v_fw, p->xm[3] = p->w_est;
    }
}
