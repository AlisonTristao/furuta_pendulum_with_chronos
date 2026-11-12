#include "qlpv_mpc.h"

#include "real.h"
#include <stdlib.h>
#include <string.h>

struct mpc {
    mpc_config cfg;
    modelo m;
    int N, primeiro;
    real u_prev;
    chronos *ch;
    real *U, *S;          /* predição: U (N), S (4×N) */
    real *P, *Pn;         /* trajetória dos ρ: 8×N (coluna j = estágio j) */
    real *A, *B, *ref;    /* 4×4×N, 4×N, 4×N */
    real *h, *T;            /* duração do estágio k e instante do fim do estágio k (N); NULL = uniforme */
    real Pterm[16];
};

mpc_config mpc_config_artigo(void)
{
    mpc_config c = {0};
    c.Np = 40;
    c.Q[0] = 10, c.Q[1] = 100, c.Q[2] = 5, c.Q[3] = 1;
    c.Rdu = 100;
    for (int i = 0; i < 4; i++) c.peso_exp_estados[i] = 1;
    c.u_min = -0.6, c.u_max = 0.6;
    c.du_min = -10, c.du_max = 10;
    c.theta1_min = -3.1416, c.theta1_max = 3.1416;
    c.dtheta_max = 30;
    c.qv_theta1 = 1000, c.qv_dtheta = 1000, c.BIG = 1e3;
    c.rho_avg = 1;
    c.terminal_dlqr = 1;
    c.Ru_lqr = 1;
    c.l_max = 5;
    c.eps_rho = 1e-3;
    c.op = chronos_opcoes_padrao();
    return c;
}

/* A(P), B(P) por estágio (ZOH, com ρ médio entre o estágio j e o j+1 se rho_avg). */
static void dinamica(mpc *c, const real *P)
{
    for (int j = 0; j < c->N; j++) {
        real r[8];
        int jn = j + 1 < c->N ? j + 1 : j;
        for (int i = 0; i < 8; i++)
            r[i] = c->cfg.rho_avg ? (real)0.5 * (P[j * 8 + i] + P[jn * 8 + i]) : P[j * 8 + i];
        if (c->h) modelo_zoh_h(&c->m, r, c->h[j], c->A + 16 * j, c->B + 4 * j);
        else modelo_zoh(&c->m, r, c->A + 16 * j, c->B + 4 * j);
    }
}

/* s = rollout de U a partir de x com a dinâmica atual. */
static void rollout(const mpc *c, const real x[4], const real *U, real *s)
{
    real xk[4] = {x[0], x[1], x[2], x[3]}, t[4];
    for (int k = 0; k < c->N; k++) {
        const real *A = c->A + 16 * k, *B = c->B + 4 * k;
        for (int i = 0; i < 4; i++) {
            real acc = B[i] * U[k];
            for (int j = 0; j < 4; j++) acc += A[j * 4 + i] * xk[j];
            t[i] = acc;
        }
        memcpy(xk, t, sizeof(xk));
        memcpy(s + 4 * k, xk, sizeof(xk));
    }
}

/* Warm start com passo variável: U' e S' nos instantes da grade avançados de dt. U constante por estágio
 * (estágio k em [T_{k-1}, T_k)), S linear entre os fins de estágio; além do horizonte, o último valor. */
static void desloca_variavel(mpc *c, real dt)
{
    int N = c->N;
    real *U0 = c->Pn, *S0 = c->Pn + N;                 /* Pn (8×N) como rascunho: U (N) e S (4×N) cabem */
    memcpy(U0, c->U, N * sizeof(real));
    memcpy(S0, c->S, 4 * N * sizeof(real));
    int j = 0;
    for (int k = 0; k < N; k++) {                      /* U'_k = U(T_{k-1} + dt) */
        real t = (k ? c->T[k - 1] : 0) + dt;
        while (j < N - 1 && t >= c->T[j]) j++;
        c->U[k] = U0[j];
    }
    j = 0;
    for (int k = 0; k < N; k++) {                      /* S'_k = S(T_k + dt) */
        real t = c->T[k] + dt;
        while (j < N - 1 && t > c->T[j + 1]) j++;
        if (j >= N - 1 || t >= c->T[N - 1]) {
            memcpy(c->S + 4 * k, S0 + 4 * (N - 1), 4 * sizeof(real));
            continue;
        }
        real a = (t - c->T[j]) / (c->T[j + 1] - c->T[j]);
        for (int i = 0; i < 4; i++) c->S[4 * k + i] = (1 - a) * S0[4 * j + i] + a * S0[4 * (j + 1) + i];
    }
}

/* P = (f_ρ(x), f_ρ(s_1), ..., f_ρ(s_{N-1})). */
static void rho_trajetoria(const mpc *c, const real x[4], const real *s, real *P)
{
    modelo_rho(&c->m, x, P);
    for (int j = 1; j < c->N; j++) modelo_rho(&c->m, s + 4 * (j - 1), P + 8 * j);
}

/* DARE por iteração de Riccati (nu = 1), em double (só na criação; em float a tolerância não fecha): P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA. */
static void dlqr(const double A[16], const double B[4], const double Q[4], double R, double P[16])
{
    memset(P, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; i++) P[i * 5] = Q[i];
    for (int it = 0; it < 200000; it++) {
        double AP[16], PB[4], BPB = R, APB[4], Pn[16];
        for (int i = 0; i < 4; i++) {                 /* PB = P·B, APB = A'·P·B */
            PB[i] = 0;
            for (int k = 0; k < 4; k++) PB[i] += P[k * 4 + i] * B[k];
        }
        for (int i = 0; i < 4; i++) BPB += B[i] * PB[i];
        for (int i = 0; i < 4; i++) {
            APB[i] = 0;
            for (int k = 0; k < 4; k++) APB[i] += A[i * 4 + k] * PB[k];     /* (A')_{ik} = A_{ki} */
        }
        for (int i = 0; i < 4; i++)                   /* AP = A'·P */
            for (int j = 0; j < 4; j++) {
                double acc = 0;
                for (int k = 0; k < 4; k++) acc += A[i * 4 + k] * P[j * 4 + k];
                AP[j * 4 + i] = acc;
            }
        double dif = 0;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                double acc = 0;                       /* (A'PA)_{ij} */
                for (int k = 0; k < 4; k++) acc += AP[k * 4 + i] * A[j * 4 + k];
                double v = (i == j ? Q[i] : 0) + acc - APB[i] * APB[j] / BPB;
                dif = fmax(dif, fabs(v - P[j * 4 + i]));
                Pn[j * 4 + i] = v;
            }
        for (int i = 0; i < 4; i++)                   /* simetriza a cada passo (a assimetria numérica diverge) */
            for (int j = 0; j < 4; j++) P[j * 4 + i] = 0.5 * (Pn[j * 4 + i] + Pn[i * 4 + j]);
        if (dif < 1e-12 * (1 + fabs(P[0]))) break;
    }
}

mpc *mpc_cria(const mpc_config *cfg, const modelo *m, const double x0d[4], double u_prevd)
{
    real x0[4] = {x0d[0], x0d[1], x0d[2], x0d[3]}, u_prev = u_prevd;
    mpc *c = (mpc *)calloc(1, sizeof(mpc));
    int N = cfg->Np;
    c->cfg = *cfg;
    c->m = *m;
    c->N = N;
    c->U = (real *)calloc(N, sizeof(real));
    c->S = (real *)calloc(4 * N, sizeof(real));
    c->P = (real *)calloc(8 * N, sizeof(real));
    c->Pn = (real *)calloc(8 * N, sizeof(real));
    c->A = (real *)calloc(16 * N, sizeof(real));
    c->B = (real *)calloc(4 * N, sizeof(real));
    c->ref = (real *)calloc(4 * N, sizeof(real));

    real Ts = (real)m->Ts, T_tot = N * Ts;
    if (cfg->horizonte > N * Ts * (1 + (real)1e-6) && N > 1) {   /* passo crescente: Σ h_k = horizonte */
        real inc = 2 * (cfg->horizonte - N * Ts) / ((real)N * (N - 1));
        c->h = (real *)calloc(N, sizeof(real));
        c->T = (real *)calloc(N, sizeof(real));
        real t = 0;
        for (int k = 0; k < N; k++) c->h[k] = Ts + k * inc, t += c->h[k], c->T[k] = t;
        T_tot = t;
    }
    real Qe[16] = {0};
    for (int i = 0; i < 4; i++) Qe[i * 5] = cfg->Q[i];
    real vmax = cfg->dtheta_max;
    real smin[4] = {cfg->theta1_min, -cfg->BIG, -vmax, -vmax};
    real smax[4] = {cfg->theta1_max, cfg->BIG, vmax, vmax};
    real qv[4] = {cfg->qv_theta1, cfg->qv_theta1, cfg->qv_dtheta, cfg->qv_dtheta};
    if (cfg->terminal_dlqr) {
        real r0[8], x_topo[4] = {0, 0, 0, 0}, A0[16], B0[4];
        modelo_rho(m, x_topo, r0);
        modelo_zoh(m, r0, A0, B0);
        double A0d[16], B0d[4], Qd[4], Pd[16];
        for (int i = 0; i < 16; i++) A0d[i] = A0[i];
        for (int i = 0; i < 4; i++) B0d[i] = B0[i], Qd[i] = cfg->Q[i];
        dlqr(A0d, B0d, Qd, cfg->Ru_lqr, Pd);
        for (int i = 0; i < 16; i++) c->Pterm[i] = (real)Pd[i];
    }
    chronos_restricoes rs = {1, 1, 1};
    c->ch = chronos_cria(4, 1, N, cfg->op, rs, Qe, &cfg->Rdu, cfg->terminal_dlqr ? c->Pterm : NULL, smin, smax, qv,
                         qv, &cfg->u_min, &cfg->u_max, &cfg->du_min, &cfg->du_max);

    if (cfg->peso_exp != 0 || c->h) {             /* exponencial: longe da referência no começo, perto no fim */
        real *w = (real *)malloc(4 * N * sizeof(real));
        for (int k = 0; k < N; k++) {
            real fr = c->h ? c->T[k] / T_tot : (k + (real)1) / N;   /* fração do horizonte no fim do estágio */
            real esc = c->h ? c->h[k] / Ts : 1;   /* passo variável: peso ∝ duração (custo = integral no tempo) */
            for (int i = 0; i < 4; i++)
                w[4 * k + i] = esc * (cfg->peso_exp_estados[i] && cfg->peso_exp != 0 ? exp(cfg->peso_exp * (fr - 1)) : 1);
        }
        chronos_peso_estagio(c->ch, w);
        free(w);
    }

    /* iterado inicial (build_chronos_mpc): u = u_prev saturado, modelo congelado em x0 */
    real eps = cfg->op.slack_epsilon;
    real u0 = fmin(fmax(u_prev, cfg->u_min + eps), cfg->u_max - eps);
    for (int k = 0; k < N; k++) c->U[k] = u0;
    for (int j = 0; j < N; j++) modelo_rho(m, x0, c->P + 8 * j);
    dinamica(c, c->P);
    chronos_dinamica(c->ch, c->A, c->B);
    chronos_inicializa(c->ch, x0, &u_prev);
    rollout(c, x0, c->U, c->S);
    c->u_prev = u_prev;
    c->primeiro = 1;
    return c;
}

void mpc_libera(mpc *c)
{
    if (!c) return;
    chronos_libera(c->ch);
    free(c->U), free(c->S), free(c->P), free(c->Pn), free(c->A), free(c->B), free(c->ref);
    free(c->h), free(c->T);
    free(c);
}

double mpc_passo(mpc *c, const double xd[4], const double refd[4], mpc_info *info)
{
    real x[4] = {xd[0], xd[1], xd[2], xd[3]}, ref[4] = {refd[0], refd[1], refd[2], refd[3]};
    int N = c->N;
    if (!c->primeiro && !c->h) {                       /* warm start: predição anterior deslocada */
        memmove(c->S, c->S + 4, 4 * (N - 1) * sizeof(real));
        memmove(c->U, c->U + 1, (N - 1) * sizeof(real));
        c->U[N - 1] = c->U[N - 2];
    } else if (!c->primeiro) {                         /* passo variável: a predição anterior Ts à frente no tempo */
        desloca_variavel(c, (real)c->m.Ts);
    }
    if (c->primeiro)
        for (int j = 0; j < N; j++) modelo_rho(&c->m, x, c->P + 8 * j);
    else
        rho_trajetoria(c, x, c->S, c->P);
    for (int k = 0; k < N; k++) memcpy(c->ref + 4 * k, ref, 4 * sizeof(real));

    mpc_info in = {0, 0, INFINITY, 0, 1};
    real *cu = chronos_u(c->ch), *csu = chronos_su(c->ch), *cs = chronos_s(c->ch);
    for (;;) {
        CHRONOS_T(9, dinamica(c, c->P));
        chronos_dinamica(c->ch, c->A, c->B);
        memcpy(cu, c->U, N * sizeof(real));          /* partida: U atual, su = u, s = rollout */
        memcpy(csu, c->U, N * sizeof(real));
        CHRONOS_T(10, rollout(c, x, c->U, cs); chronos_reinicia_folgas(c->ch, x, &c->u_prev));
        real u0;
        in.n_it += chronos_resolve(c->ch, x, &c->u_prev, c->ref, &u0);
        in.n_qp++;
        memcpy(c->U, cu, N * sizeof(real));
        memcpy(c->S, cs, 4 * N * sizeof(real));
        in.finito = 1;
        for (int i = 0; i < 4 * N && in.finito; i++) in.finito = isfinite(c->S[i]);
        for (int k = 0; k < N && in.finito; k++) in.finito = isfinite(c->U[k]);
        if (!in.finito) {
            in.dP = INFINITY;
            break;
        }
        CHRONOS_T(11, rho_trajetoria(c, x, c->S, c->Pn));
        real dP = 0;
        for (int i = 0; i < 8 * N; i++) dP = fmax(dP, fabs(c->Pn[i] - c->P[i]));
        in.dP = dP;
        if (dP < c->cfg.eps_rho || in.n_qp >= c->cfg.l_max) break;
        real *t = c->P;                              /* Picard: P <- T(P) */
        c->P = c->Pn;
        c->Pn = t;
    }
    in.convergiu = in.dP < c->cfg.eps_rho;
    real u = in.finito ? fmin(fmax(c->U[0], c->cfg.u_min), c->cfg.u_max) : 0;
    c->u_prev = u;
    c->primeiro = 0;
    if (info) *info = in;
    return u;
}

const real *mpc_U(const mpc *c) { return c->U; }
const real *mpc_S(const mpc *c) { return c->S; }
const real *mpc_P_terminal(const mpc *c) { return c->Pterm; }
