/* CHRONOS em C: porte linha a linha do solver canônico (ver chronos.h).
 * Cada função abaixo corresponde a um arquivo .m de matlab/external/CHRONOS-MPC/solver, na mesma ordem
 * de operações. Índices 0-based: o estágio interior i (0..N-2) é o k = i+1 do MATLAB; a página i+1 de
 * A, B, Q_hat corresponde à página k+1 do MATLAB. */
#include "chronos.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define M(a, i, j, ld) (a)[(i) + (size_t)(j) * (ld)]

struct chronos {
    int nx, nu, N, nse;
    chronos_opcoes op;
    chronos_restricoes rs;
    int ter;                        /* ter_ingredients */
    int recompute_cost_hess;

    /* linhas das desigualdades (assign_inequalities.m); -1 = desligada */
    int ng0, ngk, nvk, nter;
    int r0_umin, r0_umax, r0_dumin, r0_dumax;
    int rk_smin, rk_smax, rk_umin, rk_umax, rk_dumin, rk_dumax;
    int vk_smin, vk_smax;           /* linhas de v_k (min_row_v_k, max_row_v_k) */
    int rt_smin, rt_smax;
    int *v_rows_k;                  /* linha de g_k de cada folga v_k */

    /* dados */
    double *A, *B, *Qe, *Rdu, *P2;
    double *wq;                 /* peso do rastreamento por estágio e estado (nx×N; coluna N-1 = terminal) */
    double *s_min, *s_max, *u_min, *u_max, *du_min, *du_max;
    double *grad_qv_k, *grad_qv_ter;
    double *r, *r_ter, *xN_ref, *beq_0;

    /* iterado */
    double *s, *u, *su, *du, *s_ter, *err, *err_ter;
    double *g_0, *g_k, *g_ter, *v_k, *v_ter;

    /* gradiente e Hessiana do custo */
    double *grad_u_f0_0, *grad_se_f0_k, *grad_u_f0_k, *grad_se_f0_ter;
    double *R_f0_0, *Q_f0_k, *R_f0_k, *Y_f0_k, *Q_f0_ter;

    /* KKT reduzido */
    double *ru_0, *rse_k, *ru_k, *rse_ter, *R_0, *Q_k, *R_k, *Y_k, *Q_ter;
    double *rp_0, *rp_k;
    double *ri_0, *g2_0, *iS_0, *iS_ri_hat_0;
    double *ri_k, *g2_k, *iS_k, *iS_ri_hat_k, *v2_k, *rv_v2_k;
    double *ri_ter, *g2_ter, *iS_ter, *iS_ri_hat_ter, *v2_ter, *rv_v2_ter;

    /* Riccati */
    double *Q_hat, *R_hat, *Y_hat, *K_ric, *d_ric, *rs_hat, *rp_hat, *ru_hat;
    double *R_hat_0, *d_ric_0, *rp_hat_0, *ru_hat_0, *QA_ric, *G_ric, *rhs, *ldl;

    /* passo */
    double *delta_u, *delta_se, *delta_g_0, *delta_g_k, *delta_g_ter, *delta_v_k, *delta_v_ter;
};

#define WQ(c, i, j, k) sqrt((c)->wq[(size_t)(k) * (c)->nx + (i)] * (c)->wq[(size_t)(k) * (c)->nx + (j)])
static double *zeros(size_t n) { return (double *)calloc(n > 0 ? n : 1, sizeof(double)); }
static double *copia(const double *x, size_t n) { double *y = zeros(n); if (x) memcpy(y, x, n * sizeof(double)); return y; }
static double dmax(double a, double b) { return a > b ? a : b; }
static double dmin(double a, double b) { return a < b ? a : b; }

chronos_opcoes chronos_opcoes_padrao(void)
{
    chronos_opcoes op = {50.0, 0.75, 1e-6, 1e-4, 1e-3, 1e-6, 10};
    return op;
}

/* ---------------------------------------------------------------------------------------------------- */
/* utils/get_mpc_variables.m                                                                             */

static void get_mpc_variables(chronos *c, const double *s_prev, const double *u_prev)
{
    int nx = c->nx, nu = c->nu, N = c->N;
    (void)s_prev;
    for (int i = 0; i < nx; i++) c->s_ter[i] = M(c->s, i, N - 1, nx);
    for (int i = 0; i < nu; i++) c->du[i] = c->u[i] - u_prev[i];
    for (int k = 1; k < N; k++)
        for (int i = 0; i < nu; i++) M(c->du, i, k, nu) = M(c->u, i, k, nu) - M(c->su, i, k - 1, nu);
    /* y = C s com C = I (y_use_s; sem y_0 nem D) */
    for (int k = 0; k < N - 1; k++)
        for (int i = 0; i < nx; i++) M(c->err, i, k, nx) = M(c->r, i, k, nx) - M(c->s, i, k, nx);
    for (int i = 0; i < nx; i++) c->err_ter[i] = c->r_ter[i] - c->s[i + (size_t)(N - 1) * nx];
}

/* ---------------------------------------------------------------------------------------------------- */
/* update_mpc_f0_hess.m (+ f0_hess_*_wrapper.m)                                                          */

static void update_mpc_f0_hess(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse;
    size_t pq = (size_t)nse * nse, pr = (size_t)nu * nu, py = (size_t)nu * nse;
    c->recompute_cost_hess = 0;
    memset(c->R_f0_0, 0, pr * sizeof(double));
    memset(c->Q_f0_k, 0, pq * (N - 1) * sizeof(double));
    memset(c->R_f0_k, 0, pr * (N - 1) * sizeof(double));
    memset(c->Y_f0_k, 0, py * (N - 1) * sizeof(double));
    memset(c->Q_f0_ter, 0, pq * sizeof(double));
    /* rastreamento: Q_E = C'Qe C = Qe */
    for (int k = 0; k < N - 1; k++)
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nx; i++) c->Q_f0_k[k * pq + i + j * nse] += WQ(c, i, j, k) * M(c->Qe, i, j, nx);
    for (int j = 0; j < nx; j++)
        for (int i = 0; i < nx; i++) M(c->Q_f0_ter, i, j, nse) += WQ(c, i, j, N - 1) * M(c->Qe, i, j, nx);
    /* Δu */
    for (int j = 0; j < nu; j++)
        for (int i = 0; i < nu; i++) M(c->R_f0_0, i, j, nu) += M(c->Rdu, i, j, nu);
    for (int k = 0; k < N - 1; k++)
        for (int j = 0; j < nu; j++)
            for (int i = 0; i < nu; i++) {
                double rdu = M(c->Rdu, i, j, nu);
                c->Q_f0_k[k * pq + (nx + i) + (size_t)(nx + j) * nse] += rdu;
                c->R_f0_k[k * pr + i + (size_t)j * nu] += rdu;
                c->Y_f0_k[k * py + i + (size_t)(nx + j) * nu] -= rdu;
            }
    /* terminal */
    if (c->ter)
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nx; i++) M(c->Q_f0_ter, i, j, nse) += M(c->P2, i, j, nx);
}

/* ---------------------------------------------------------------------------------------------------- */
/* grad_f0_MPC.m: reset, Δu, rastreamento, terminal, escala por t                                         */

static void grad_f0_MPC(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse;
    double t = c->op.t;
    memset(c->grad_u_f0_0, 0, nu * sizeof(double));
    memset(c->grad_se_f0_k, 0, (size_t)nse * (N - 1) * sizeof(double));
    memset(c->grad_u_f0_k, 0, (size_t)nu * (N - 1) * sizeof(double));
    memset(c->grad_se_f0_ter, 0, nse * sizeof(double));

    /* grad_f0_control_rate_wrapper.m */
    for (int j = 0; j < nu; j++)
        for (int i = 0; i < nu; i++) c->grad_u_f0_0[i] += M(c->Rdu, i, j, nu) * c->du[j];
    for (int k = 0; k < N - 1; k++)
        for (int i = 0; i < nu; i++) {
            double gdu = 0.0;
            for (int j = 0; j < nu; j++) gdu += M(c->Rdu, i, j, nu) * M(c->du, j, k + 1, nu);
            M(c->grad_se_f0_k, nx + i, k, nse) -= gdu;
            M(c->grad_u_f0_k, i, k, nu) += gdu;
        }
    /* grad_f0_tracking_wrapper.m: grad_s_E = C'Qe = Qe */
    for (int k = 0; k < N - 1; k++)
        for (int i = 0; i < nx; i++) {
            double ge = 0.0;
            for (int j = 0; j < nx; j++) ge += WQ(c, i, j, k) * M(c->Qe, i, j, nx) * M(c->err, j, k, nx);
            M(c->grad_se_f0_k, i, k, nse) -= ge;
        }
    for (int i = 0; i < nx; i++) {
        double ge = 0.0;
        for (int j = 0; j < nx; j++) ge += WQ(c, i, j, N - 1) * M(c->Qe, i, j, nx) * c->err_ter[j];
        c->grad_se_f0_ter[i] -= ge;
    }
    /* grad_f0_terminal_wrapper.m */
    if (c->ter)
        for (int i = 0; i < nx; i++) {
            double ge = 0.0;
            for (int j = 0; j < nx; j++) ge += M(c->P2, i, j, nx) * (c->xN_ref[j] - c->s_ter[j]);
            c->grad_se_f0_ter[i] -= ge;
        }
    /* grad_f0_scale_wrapper.m */
    for (int i = 0; i < nu; i++) c->grad_u_f0_0[i] *= t;
    for (size_t i = 0; i < (size_t)nse * (N - 1); i++) c->grad_se_f0_k[i] *= t;
    for (size_t i = 0; i < (size_t)nu * (N - 1); i++) c->grad_u_f0_k[i] *= t;
    for (int i = 0; i < nse; i++) c->grad_se_f0_ter[i] *= t;
}

/* ---------------------------------------------------------------------------------------------------- */
/* equality_residuals.m                                                                                  */

static void equality_residuals(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse;
    size_t pa = (size_t)nx * nx, pb = (size_t)nx * nu;
    for (int i = 0; i < nx; i++) {
        double bu = 0.0;
        for (int j = 0; j < nu; j++) bu += c->B[i + (size_t)j * nx] * c->u[j];
        c->rp_0[i] = bu - c->s[i] - c->beq_0[i];
    }
    for (int k = 0; k < N - 1; k++) {
        const double *A = c->A + (k + 1) * pa, *B = c->B + (k + 1) * pb;
        for (int i = 0; i < nx; i++) {
            double as = 0.0, bu = 0.0;
            for (int j = 0; j < nx; j++) as += A[i + (size_t)j * nx] * M(c->s, j, k, nx);
            for (int j = 0; j < nu; j++) bu += B[i + (size_t)j * nx] * M(c->u, j, k + 1, nu);
            M(c->rp_k, i, k, nse) = as + bu - M(c->s, i, k + 1, nx);   /* - beq_k (= 0, sem d) */
        }
    }
    /* has_du */
    for (int i = 0; i < nu; i++) c->rp_0[nx + i] = c->u[i] - c->su[i];
    for (int k = 0; k < N - 1; k++)
        for (int i = 0; i < nu; i++) M(c->rp_k, nx + i, k, nse) = M(c->u, i, k + 1, nu) - M(c->su, i, k + 1, nu);
}

/* ---------------------------------------------------------------------------------------------------- */
/* inequality_residuals.m (+ inequality_state/control/control_rate_wrapper.m)                            */

static void inequality_residuals(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, ngk = c->ngk, nvk = c->nvk;
    for (int i = 0; i < c->ng0; i++) c->ri_0[i] = c->g_0[i];
    for (int k = 0; k < N - 1; k++) {
        for (int i = 0; i < ngk; i++) M(c->ri_k, i, k, ngk) = M(c->g_k, i, k, ngk);
        for (int j = 0; j < nvk; j++) M(c->ri_k, c->v_rows_k[j], k, ngk) -= M(c->v_k, j, k, nvk);
    }
    for (int i = 0; i < c->nter; i++) c->ri_ter[i] = c->g_ter[i] - c->v_ter[i];

    if (c->rs.estado) {
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nx; i++) M(c->ri_k, c->rk_smin + i, k, ngk) += c->s_min[i] - M(c->s, i, k, nx);
        for (int i = 0; i < nx; i++) c->ri_ter[c->rt_smin + i] += c->s_min[i] - M(c->s, i, N - 1, nx);
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nx; i++) M(c->ri_k, c->rk_smax + i, k, ngk) += M(c->s, i, k, nx) - c->s_max[i];
        for (int i = 0; i < nx; i++) c->ri_ter[c->rt_smax + i] += M(c->s, i, N - 1, nx) - c->s_max[i];
    }
    if (c->rs.u) {
        for (int i = 0; i < nu; i++) c->ri_0[c->r0_umin + i] += c->u_min[i] - c->u[i];
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) M(c->ri_k, c->rk_umin + i, k, ngk) += c->u_min[i] - M(c->u, i, k + 1, nu);
        for (int i = 0; i < nu; i++) c->ri_0[c->r0_umax + i] += c->u[i] - c->u_max[i];
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) M(c->ri_k, c->rk_umax + i, k, ngk) += M(c->u, i, k + 1, nu) - c->u_max[i];
    }
    if (c->rs.du) {
        for (int i = 0; i < nu; i++) c->ri_0[c->r0_dumin + i] += c->du_min[i] - c->du[i];
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) M(c->ri_k, c->rk_dumin + i, k, ngk) += c->du_min[i] - M(c->du, i, k + 1, nu);
        for (int i = 0; i < nu; i++) c->ri_0[c->r0_dumax + i] += c->du[i] - c->du_max[i];
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) M(c->ri_k, c->rk_dumax + i, k, ngk) += M(c->du, i, k + 1, nu) - c->du_max[i];
    }
}

/* ---------------------------------------------------------------------------------------------------- */
/* reduced_KKT_elements.m (+ reduced_kkt_control/control_rate/state_wrapper.m)                           */

static void reduced_KKT_elements(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse, ngk = c->ngk, nvk = c->nvk;
    size_t pq = (size_t)nse * nse, pr = (size_t)nu * nu, py = (size_t)nu * nse;
    double t = c->op.t;

    /* reduced_kkt_objective_local */
    memcpy(c->ru_0, c->grad_u_f0_0, nu * sizeof(double));
    memcpy(c->rse_k, c->grad_se_f0_k, (size_t)nse * (N - 1) * sizeof(double));
    memcpy(c->ru_k, c->grad_u_f0_k, (size_t)nu * (N - 1) * sizeof(double));
    memcpy(c->rse_ter, c->grad_se_f0_ter, nse * sizeof(double));
    for (size_t i = 0; i < pr; i++) c->R_0[i] = t * c->R_f0_0[i];
    for (size_t i = 0; i < pq * (N - 1); i++) c->Q_k[i] = t * c->Q_f0_k[i];
    for (size_t i = 0; i < pr * (N - 1); i++) c->R_k[i] = t * c->R_f0_k[i];
    for (size_t i = 0; i < py * (N - 1); i++) c->Y_k[i] = t * c->Y_f0_k[i];
    for (size_t i = 0; i < pq; i++) c->Q_ter[i] = t * c->Q_f0_ter[i];

    /* reduced_kkt_0_local (sem folgas suaves no estágio 0) */
    for (int i = 0; i < c->ng0; i++) {
        c->ri_0[i] = c->ri_0[i] + c->g_0[i];
        c->g2_0[i] = c->g_0[i] * c->g_0[i];
        c->iS_0[i] = 1.0 / c->g2_0[i];
        c->iS_ri_hat_0[i] = c->iS_0[i] * c->ri_0[i];
    }
    /* reduced_kkt_k_local */
    for (int k = 0; k < N - 1; k++) {
        for (int i = 0; i < ngk; i++) {
            M(c->ri_k, i, k, ngk) = M(c->ri_k, i, k, ngk) + M(c->g_k, i, k, ngk);
            M(c->g2_k, i, k, ngk) = M(c->g_k, i, k, ngk) * M(c->g_k, i, k, ngk);
        }
        for (int j = 0; j < nvk; j++) {
            double v = M(c->v_k, j, k, nvk);
            M(c->v2_k, j, k, nvk) = v * v;
            M(c->rv_v2_k, j, k, nvk) = t * M(c->grad_qv_k, j, k, nvk) * M(c->v2_k, j, k, nvk) - v;
            M(c->ri_k, c->v_rows_k[j], k, ngk) += M(c->rv_v2_k, j, k, nvk);
        }
        for (int i = 0; i < ngk; i++) M(c->iS_k, i, k, ngk) = M(c->g2_k, i, k, ngk);
        for (int j = 0; j < nvk; j++) M(c->iS_k, c->v_rows_k[j], k, ngk) += M(c->v2_k, j, k, nvk);
        for (int i = 0; i < ngk; i++) {
            M(c->iS_k, i, k, ngk) = 1.0 / M(c->iS_k, i, k, ngk);
            M(c->iS_ri_hat_k, i, k, ngk) = M(c->iS_k, i, k, ngk) * M(c->ri_k, i, k, ngk);
        }
    }
    /* reduced_kkt_terminal_local (todas as linhas terminais são suaves) */
    for (int i = 0; i < c->nter; i++) {
        c->g2_ter[i] = c->g_ter[i] * c->g_ter[i];
        c->ri_ter[i] = c->ri_ter[i] + c->g_ter[i];
        c->v2_ter[i] = c->v_ter[i] * c->v_ter[i];
        c->rv_v2_ter[i] = t * c->grad_qv_ter[i] * c->v2_ter[i] - c->v_ter[i];
        c->ri_ter[i] = c->ri_ter[i] + c->rv_v2_ter[i];
        c->iS_ter[i] = c->g2_ter[i] + c->v2_ter[i];
        c->iS_ter[i] = 1.0 / c->iS_ter[i];
        c->iS_ri_hat_ter[i] = c->iS_ter[i] * c->ri_ter[i];
    }

    /* reduced_kkt_control_wrapper.m */
    if (c->rs.u) {
        for (int i = 0; i < nu; i++) {
            int rmin = c->r0_umin + i, rmax = c->r0_umax + i;
            c->ru_0[i] = c->ru_0[i] + c->iS_ri_hat_0[rmax] - c->iS_ri_hat_0[rmin];
            M(c->R_0, i, i, nu) = M(c->R_0, i, i, nu) + c->iS_0[rmin] + c->iS_0[rmax];
        }
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) {
                int rmin = c->rk_umin + i, rmax = c->rk_umax + i;
                M(c->ru_k, i, k, nu) = M(c->ru_k, i, k, nu) + M(c->iS_ri_hat_k, rmax, k, ngk) - M(c->iS_ri_hat_k, rmin, k, ngk);
                c->R_k[k * pr + i + (size_t)i * nu] = c->R_k[k * pr + i + (size_t)i * nu] + M(c->iS_k, rmin, k, ngk) + M(c->iS_k, rmax, k, ngk);
            }
    }
    /* reduced_kkt_control_rate_wrapper.m */
    if (c->rs.du) {
        for (int i = 0; i < nu; i++) {
            int rmin = c->r0_dumin + i, rmax = c->r0_dumax + i;
            c->ru_0[i] = c->ru_0[i] + c->iS_ri_hat_0[rmax] - c->iS_ri_hat_0[rmin];
            M(c->R_0, i, i, nu) = M(c->R_0, i, i, nu) + c->iS_0[rmin] + c->iS_0[rmax];
        }
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) {
                int rmin = c->rk_dumin + i, rmax = c->rk_dumax + i, su_i = nx + i;
                double delta = M(c->iS_ri_hat_k, rmax, k, ngk) - M(c->iS_ri_hat_k, rmin, k, ngk);
                double soma = M(c->iS_k, rmin, k, ngk) + M(c->iS_k, rmax, k, ngk);
                M(c->rse_k, su_i, k, nse) = M(c->rse_k, su_i, k, nse) - delta;
                M(c->ru_k, i, k, nu) = M(c->ru_k, i, k, nu) + delta;
                c->Q_k[k * pq + su_i + (size_t)su_i * nse] += soma;
                c->R_k[k * pr + i + (size_t)i * nu] += soma;
                c->Y_k[k * py + i + (size_t)su_i * nu] -= soma;
            }
    }
    /* reduced_kkt_state_wrapper.m */
    if (c->rs.estado) {
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nx; i++) {
                int rmin = c->rk_smin + i, rmax = c->rk_smax + i;
                M(c->rse_k, i, k, nse) = M(c->rse_k, i, k, nse) + M(c->iS_ri_hat_k, rmax, k, ngk) - M(c->iS_ri_hat_k, rmin, k, ngk);
                c->Q_k[k * pq + i + (size_t)i * nse] = c->Q_k[k * pq + i + (size_t)i * nse] + M(c->iS_k, rmin, k, ngk) + M(c->iS_k, rmax, k, ngk);
            }
        for (int i = 0; i < nx; i++) {
            int rmin = c->rt_smin + i, rmax = c->rt_smax + i;
            c->rse_ter[i] = c->rse_ter[i] + c->iS_ri_hat_ter[rmax] - c->iS_ri_hat_ter[rmin];
            M(c->Q_ter, i, i, nse) = M(c->Q_ter, i, i, nse) + c->iS_ter[rmin] + c->iS_ter[rmax];
        }
    }
}

/* ---------------------------------------------------------------------------------------------------- */
/* linsolve(R, rhs, SYM): R (n×n) simétrica, LDL' sem pivoteamento; para n = 1 é a divisão                */

static void resolve_sim(int n, const double *R, double *L, double *X, int nrhs)
{
    if (n == 1) {
        for (int j = 0; j < nrhs; j++) X[j] = X[j] / R[0];
        return;
    }
    memcpy(L, R, (size_t)n * n * sizeof(double));
    for (int j = 0; j < n; j++) {
        for (int p = 0; p < j; p++) M(L, j, j, n) -= M(L, j, p, n) * M(L, j, p, n) * M(L, p, p, n);
        for (int i = j + 1; i < n; i++) {
            double a = M(L, i, j, n);
            for (int p = 0; p < j; p++) a -= M(L, i, p, n) * M(L, j, p, n) * M(L, p, p, n);
            M(L, i, j, n) = a / M(L, j, j, n);
        }
    }
    for (int r = 0; r < nrhs; r++) {
        double *x = X + (size_t)r * n;
        for (int i = 0; i < n; i++) for (int p = 0; p < i; p++) x[i] -= M(L, i, p, n) * x[p];
        for (int i = 0; i < n; i++) x[i] /= M(L, i, i, n);
        for (int i = n - 1; i >= 0; i--) for (int p = i + 1; p < n; p++) x[i] -= M(L, p, i, n) * x[p];
    }
}

/* ---------------------------------------------------------------------------------------------------- */
/* riccati_KKT_du.m                                                                                      */

static void riccati_KKT_du(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse;
    size_t pq = (size_t)nse * nse, pr = (size_t)nu * nu, py = (size_t)nu * nse;
    size_t pa = (size_t)nx * nx, pb = (size_t)nx * nu;
    double *G = c->G_ric, *QA = c->QA_ric, *rhs = c->rhs;

    memcpy(c->Q_hat + (N - 1) * pq, c->Q_ter, pq * sizeof(double));
    memcpy(c->rs_hat + (size_t)(N - 1) * nse, c->rse_ter, nse * sizeof(double));

    for (int k = N - 2; k >= 0; k--) {
        const double *A = c->A + (k + 1) * pa, *B = c->B + (k + 1) * pb;
        const double *Qn = c->Q_hat + (k + 1) * pq;      /* Q_hat(:,:,k+1) */
        double *Rh = c->R_hat + k * pr, *Yh = c->Y_hat + k * py, *Kk = c->K_ric + k * py, *dk = c->d_ric + (size_t)k * nu;
        double *Qh = c->Q_hat + k * pq, *rph = c->rp_hat + (size_t)k * nse, *ruh = c->ru_hat + (size_t)k * nu;
        double *rsh = c->rs_hat + (size_t)k * nse;
        const double *rsn = c->rs_hat + (size_t)(k + 1) * nse, *rp = c->rp_k + (size_t)k * nse;

        /* G = B'Q_hat(s,s) + Q_hat(su,s) */
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nu; i++) {
                double a = 0.0;
                for (int p = 0; p < nx; p++) a += M(B, p, i, nx) * M(Qn, p, j, nse);
                M(G, i, j, nu) = a + M(Qn, nx + i, j, nse);
            }
        /* R_hat = R + G B + B'Q_hat(s,su) + Q_hat(su,su) + eps_thknv I */
        for (int j = 0; j < nu; j++)
            for (int i = 0; i < nu; i++) {
                double gb = 0.0, bq = 0.0;
                for (int p = 0; p < nx; p++) gb += M(G, i, p, nu) * M(B, p, j, nx);
                for (int p = 0; p < nx; p++) bq += M(B, p, i, nx) * M(Qn, p, nx + j, nse);
                M(Rh, i, j, nu) = c->R_k[k * pr + i + (size_t)j * nu] + gb + bq + M(Qn, nx + i, nx + j, nse);
            }
        for (int j = 0; j < nu; j++) M(Rh, j, j, nu) = M(Rh, j, j, nu) + c->op.eps_thknv;
        /* Y_hat(:,s) = Y(:,s) + G A ; Y_hat(:,su) = Y(:,su) */
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nu; i++) {
                double ga = 0.0;
                for (int p = 0; p < nx; p++) ga += M(G, i, p, nu) * M(A, p, j, nx);
                M(Yh, i, j, nu) = c->Y_k[k * py + i + (size_t)j * nu] + ga;
            }
        for (int j = nx; j < nse; j++)
            for (int i = 0; i < nu; i++) M(Yh, i, j, nu) = c->Y_k[k * py + i + (size_t)j * nu];
        /* rp_hat = rs_hat(:,k+1) + Q_hat(:,:,k+1) rp */
        for (int i = 0; i < nse; i++) {
            double a = 0.0;
            for (int p = 0; p < nse; p++) a += M(Qn, i, p, nse) * rp[p];
            rph[i] = rsn[i] + a;
        }
        /* ru_hat = ru + B'rp_hat(s) + rp_hat(su) */
        for (int i = 0; i < nu; i++) {
            double a = 0.0;
            for (int p = 0; p < nx; p++) a += M(B, p, i, nx) * rph[p];
            ruh[i] = M(c->ru_k, i, k, nu) + a + rph[nx + i];
        }
        /* [K d] = R_hat \ [Y_hat ru_hat] */
        memcpy(rhs, Yh, py * sizeof(double));
        memcpy(rhs + py, ruh, nu * sizeof(double));
        resolve_sim(nu, Rh, c->ldl, rhs, nse + 1);
        memcpy(Kk, rhs, py * sizeof(double));
        memcpy(dk, rhs + py, nu * sizeof(double));
        /* QA = Q_hat(s,s,k+1) A */
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nx; i++) {
                double a = 0.0;
                for (int p = 0; p < nx; p++) a += M(Qn, i, p, nse) * M(A, p, j, nx);
                M(QA, i, j, nx) = a;
            }
        /* Q_hat = Q ; Q_hat(s,s) += A'QA ; Q_hat -= Y_hat'K */
        memcpy(Qh, c->Q_k + k * pq, pq * sizeof(double));
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nx; i++) {
                double a = 0.0;
                for (int p = 0; p < nx; p++) a += M(A, p, i, nx) * M(QA, p, j, nx);
                M(Qh, i, j, nse) = M(Qh, i, j, nse) + a;
            }
        for (int j = 0; j < nse; j++)
            for (int i = 0; i < nse; i++) {
                double a = 0.0;
                for (int p = 0; p < nu; p++) a += M(Yh, p, i, nu) * M(Kk, p, j, nu);
                M(Qh, i, j, nse) = M(Qh, i, j, nse) - a;
            }
        /* rs_hat = rs ; rs_hat(s) += A'rp_hat(s) ; rs_hat -= Y_hat'd */
        memcpy(rsh, c->rse_k + (size_t)k * nse, nse * sizeof(double));
        for (int i = 0; i < nx; i++) {
            double a = 0.0;
            for (int p = 0; p < nx; p++) a += M(A, p, i, nx) * rph[p];
            rsh[i] = rsh[i] + a;
        }
        for (int i = 0; i < nse; i++) {
            double a = 0.0;
            for (int p = 0; p < nu; p++) a += M(Yh, p, i, nu) * dk[p];
            rsh[i] = rsh[i] - a;
        }
    }

    /* estágio 0 */
    {
        const double *B = c->B, *Qn = c->Q_hat;
        for (int j = 0; j < nx; j++)
            for (int i = 0; i < nu; i++) {
                double a = 0.0;
                for (int p = 0; p < nx; p++) a += M(B, p, i, nx) * M(Qn, p, j, nse);
                M(G, i, j, nu) = a + M(Qn, nx + i, j, nse);
            }
        for (int j = 0; j < nu; j++)
            for (int i = 0; i < nu; i++) {
                double gb = 0.0, bq = 0.0;
                for (int p = 0; p < nx; p++) gb += M(G, i, p, nu) * M(B, p, j, nx);
                for (int p = 0; p < nx; p++) bq += M(B, p, i, nx) * M(Qn, p, nx + j, nse);
                M(c->R_hat_0, i, j, nu) = M(c->R_0, i, j, nu) + gb + bq + M(Qn, nx + i, nx + j, nse);
            }
        for (int j = 0; j < nu; j++) M(c->R_hat_0, j, j, nu) = M(c->R_hat_0, j, j, nu) + c->op.eps_thknv;
        for (int i = 0; i < nse; i++) {
            double a = 0.0;
            for (int p = 0; p < nse; p++) a += M(Qn, i, p, nse) * c->rp_0[p];
            c->rp_hat_0[i] = c->rs_hat[i] + a;
        }
        for (int i = 0; i < nu; i++) {
            double a = 0.0;
            for (int p = 0; p < nx; p++) a += M(B, p, i, nx) * c->rp_hat_0[p];
            c->ru_hat_0[i] = c->ru_0[i] + a + c->rp_hat_0[nx + i];
        }
        memcpy(c->d_ric_0, c->ru_hat_0, nu * sizeof(double));
        resolve_sim(nu, c->R_hat_0, c->ldl, c->d_ric_0, 1);
    }

    /* recursão para frente */
    for (int i = 0; i < nu; i++) c->delta_u[i] = -c->d_ric_0[i];
    for (int i = 0; i < nx; i++) {
        double a = 0.0;
        for (int p = 0; p < nu; p++) a += c->B[i + (size_t)p * nx] * c->delta_u[p];
        c->delta_se[i] = a + c->rp_0[i];
    }
    for (int i = 0; i < nu; i++) c->delta_se[nx + i] = c->delta_u[i] + c->rp_0[nx + i];
    for (int k = 0; k < N - 1; k++) {
        const double *A = c->A + (k + 1) * pa, *B = c->B + (k + 1) * pb;
        const double *Kk = c->K_ric + k * py, *dk = c->d_ric + (size_t)k * nu, *rp = c->rp_k + (size_t)k * nse;
        const double *dse = c->delta_se + (size_t)k * nse;
        double *dun = c->delta_u + (size_t)(k + 1) * nu, *dsen = c->delta_se + (size_t)(k + 1) * nse;
        for (int i = 0; i < nu; i++) {
            double a = 0.0;
            for (int p = 0; p < nse; p++) a += M(Kk, i, p, nu) * dse[p];
            dun[i] = -dk[i] - a;
        }
        for (int i = 0; i < nx; i++) {
            double as = 0.0, bu = 0.0;
            for (int p = 0; p < nx; p++) as += M(A, i, p, nx) * dse[p];
            for (int p = 0; p < nu; p++) bu += M(B, i, p, nx) * dun[p];
            dsen[i] = as + bu + rp[i];
        }
        for (int i = 0; i < nu; i++) dsen[nx + i] = dun[i] + rp[nx + i];
    }
}

/* ---------------------------------------------------------------------------------------------------- */
/* recover_slacks.m (+ recover_slacks_state/control/control_rate_wrapper.m)                              */

static void recover_slacks(chronos *c)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse, ngk = c->ngk, nvk = c->nvk;
    if (c->rs.estado) {
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nx; i++) {
                int rmin = c->rk_smin + i, rmax = c->rk_smax + i;
                int vmin = c->vk_smin + i, vmax = c->vk_smax + i;
                double ds = M(c->delta_se, i, k, nse);
                double mu_min = M(c->iS_ri_hat_k, rmin, k, ngk) - M(c->iS_k, rmin, k, ngk) * ds;
                double mu_max = M(c->iS_ri_hat_k, rmax, k, ngk) + M(c->iS_k, rmax, k, ngk) * ds;
                M(c->delta_g_k, rmin, k, ngk) = M(c->g_k, rmin, k, ngk) - M(c->g2_k, rmin, k, ngk) * mu_min;
                M(c->delta_g_k, rmax, k, ngk) = M(c->g_k, rmax, k, ngk) - M(c->g2_k, rmax, k, ngk) * mu_max;
                M(c->delta_v_k, vmin, k, nvk) = -M(c->rv_v2_k, vmin, k, nvk) + M(c->v2_k, vmin, k, nvk) * mu_min;
                M(c->delta_v_k, vmax, k, nvk) = -M(c->rv_v2_k, vmax, k, nvk) + M(c->v2_k, vmax, k, nvk) * mu_max;
            }
        for (int i = 0; i < nx; i++) {
            int rmin = c->rt_smin + i, rmax = c->rt_smax + i;
            double ds = M(c->delta_se, i, N - 1, nse);
            double mu_min = c->iS_ri_hat_ter[rmin] - c->iS_ter[rmin] * ds;
            double mu_max = c->iS_ri_hat_ter[rmax] + c->iS_ter[rmax] * ds;
            c->delta_g_ter[rmin] = c->g_ter[rmin] - c->g2_ter[rmin] * mu_min;
            c->delta_g_ter[rmax] = c->g_ter[rmax] - c->g2_ter[rmax] * mu_max;
            c->delta_v_ter[rmin] = -c->rv_v2_ter[rmin] + c->v2_ter[rmin] * mu_min;
            c->delta_v_ter[rmax] = -c->rv_v2_ter[rmax] + c->v2_ter[rmax] * mu_max;
        }
    }
    if (c->rs.u) {
        for (int i = 0; i < nu; i++) {
            int rmin = c->r0_umin + i, rmax = c->r0_umax + i;
            c->delta_g_0[rmin] = c->g_0[rmin] - c->g2_0[rmin] * c->iS_ri_hat_0[rmin] + c->delta_u[i];
            c->delta_g_0[rmax] = c->g_0[rmax] - c->g2_0[rmax] * c->iS_ri_hat_0[rmax] - c->delta_u[i];
        }
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) {
                int rmin = c->rk_umin + i, rmax = c->rk_umax + i;
                double dui = M(c->delta_u, i, k + 1, nu);
                M(c->delta_g_k, rmin, k, ngk) = M(c->g_k, rmin, k, ngk) - M(c->g2_k, rmin, k, ngk) * M(c->iS_ri_hat_k, rmin, k, ngk) + dui;
                M(c->delta_g_k, rmax, k, ngk) = M(c->g_k, rmax, k, ngk) - M(c->g2_k, rmax, k, ngk) * M(c->iS_ri_hat_k, rmax, k, ngk) - dui;
            }
    }
    if (c->rs.du) {
        for (int i = 0; i < nu; i++) {
            int rmin = c->r0_dumin + i, rmax = c->r0_dumax + i;
            c->delta_g_0[rmin] = c->g_0[rmin] - c->g2_0[rmin] * c->iS_ri_hat_0[rmin] + c->delta_u[i];
            c->delta_g_0[rmax] = c->g_0[rmax] - c->g2_0[rmax] * c->iS_ri_hat_0[rmax] - c->delta_u[i];
        }
        for (int k = 0; k < N - 1; k++)
            for (int i = 0; i < nu; i++) {
                int rmin = c->rk_dumin + i, rmax = c->rk_dumax + i;
                double dui = M(c->delta_u, i, k + 1, nu) - M(c->delta_se, nx + i, k, nse);
                M(c->delta_g_k, rmin, k, ngk) = M(c->g_k, rmin, k, ngk) - M(c->g2_k, rmin, k, ngk) * M(c->iS_ri_hat_k, rmin, k, ngk) + dui;
                M(c->delta_g_k, rmax, k, ngk) = M(c->g_k, rmax, k, ngk) - M(c->g2_k, rmax, k, ngk) * M(c->iS_ri_hat_k, rmax, k, ngk) - dui;
            }
    }
}

/* ---------------------------------------------------------------------------------------------------- */
/* get_lambda2.m                                                                                         */

static double get_lambda2(const chronos *c)
{
    int nu = c->nu, N = c->N, nse = c->nse, ngk = c->ngk, nvk = c->nvk;
    double t = c->op.t, lambda2 = 0.0, a;
    a = 0.0;
    for (int i = 0; i < nu; i++) a += c->grad_u_f0_0[i] * c->delta_u[i];
    lambda2 = lambda2 - a;
    if (c->ng0) {
        a = 0.0;
        for (int i = 0; i < c->ng0; i++) a += (1.0 / c->g_0[i]) * c->delta_g_0[i];
        lambda2 = lambda2 + a;
    }
    for (int k = 0; k < N - 1; k++) {
        a = 0.0;
        for (int i = 0; i < nse; i++) a += M(c->grad_se_f0_k, i, k, nse) * M(c->delta_se, i, k, nse);
        lambda2 = lambda2 - a;
        a = 0.0;
        for (int i = 0; i < nu; i++) a += M(c->grad_u_f0_k, i, k, nu) * M(c->delta_u, i, k + 1, nu);
        lambda2 = lambda2 - a;
        if (ngk) {
            a = 0.0;
            for (int i = 0; i < ngk; i++) a += (1.0 / M(c->g_k, i, k, ngk)) * M(c->delta_g_k, i, k, ngk);
            lambda2 = lambda2 + a;
        }
        if (nvk) {
            a = 0.0;
            for (int j = 0; j < nvk; j++)
                a += (t * M(c->grad_qv_k, j, k, nvk) - 1.0 / M(c->v_k, j, k, nvk)) * M(c->delta_v_k, j, k, nvk);
            lambda2 = lambda2 - a;
        }
    }
    a = 0.0;
    for (int i = 0; i < nse; i++) a += c->grad_se_f0_ter[i] * M(c->delta_se, i, N - 1, nse);
    lambda2 = lambda2 - a;
    if (c->nter) {
        a = 0.0;
        for (int i = 0; i < c->nter; i++) a += (1.0 / c->g_ter[i]) * c->delta_g_ter[i];
        lambda2 = lambda2 + a;
        a = 0.0;
        for (int i = 0; i < c->nter; i++) a += (t * c->grad_qv_ter[i] - 1.0 / c->v_ter[i]) * c->delta_v_ter[i];
        lambda2 = lambda2 - a;
    }
    return lambda2;
}

/* ---------------------------------------------------------------------------------------------------- */
/* line_search_local (mpc_solve.m): busca de passo só de viabilidade                                      */

static int todos_positivos(const double *x, const double *dx, size_t n, double l)
{
    for (size_t i = 0; i < n; i++) if (!(x[i] + l * dx[i] > 0)) return 0;
    return 1;
}

static int line_search_local(chronos *c, int continue_Newton)
{
    int nx = c->nx, nu = c->nu, N = c->N, nse = c->nse;
    size_t n0 = c->ng0, nk = (size_t)c->ngk * (N - 1), nv = (size_t)c->nvk * (N - 1), nt = c->nter;
    double l = 1.0;
#define FEAS(l) (todos_positivos(c->g_0, c->delta_g_0, n0, l) && todos_positivos(c->g_k, c->delta_g_k, nk, l) && \
                 todos_positivos(c->g_ter, c->delta_g_ter, nt, l) && todos_positivos(c->v_k, c->delta_v_k, nv, l) && \
                 todos_positivos(c->v_ter, c->delta_v_ter, nt, l))
    if (!FEAS(l)) {
        /* No original o laço não tem saída se o passo tiver NaN (l*Beta para no menor subnormal e nunca
         * chega a zero); aqui para em l < 1e-300, como o qp.py. O critério min_l abaixo encerra o Newton. */
        while (!FEAS(l) && l >= 1e-300) l = l * c->op.Beta;
        if (l < c->op.min_l) continue_Newton = 0;
    }
#undef FEAS
    for (size_t i = 0; i < n0; i++) c->g_0[i] = c->g_0[i] + l * c->delta_g_0[i];
    for (size_t i = 0; i < nk; i++) c->g_k[i] = c->g_k[i] + l * c->delta_g_k[i];
    for (size_t i = 0; i < nt; i++) c->g_ter[i] = c->g_ter[i] + l * c->delta_g_ter[i];
    for (size_t i = 0; i < nv; i++) c->v_k[i] = c->v_k[i] + l * c->delta_v_k[i];
    for (size_t i = 0; i < nt; i++) c->v_ter[i] = c->v_ter[i] + l * c->delta_v_ter[i];
    for (size_t i = 0; i < (size_t)nu * N; i++) c->u[i] = c->u[i] + l * c->delta_u[i];
    for (int k = 0; k < N; k++) {
        for (int i = 0; i < nx; i++) M(c->s, i, k, nx) = M(c->s, i, k, nx) + l * M(c->delta_se, i, k, nse);
        for (int i = 0; i < nu; i++) M(c->su, i, k, nu) = M(c->su, i, k, nu) + l * M(c->delta_se, nx + i, k, nse);
    }
    return continue_Newton;
}

/* ---------------------------------------------------------------------------------------------------- */
/* mpc_solve.m                                                                                           */

int chronos_resolve(chronos *c, const double *s_prev, const double *u_prev, const double *ref, double *u0)
{
    int nx = c->nx, N = c->N;
    /* r = ref(:,1:N-1); r_ter = ref(:,N); xN_ref = r_ter (xN_ref_is_y) */
    memcpy(c->r, ref, (size_t)nx * (N - 1) * sizeof(double));
    memcpy(c->r_ter, ref + (size_t)nx * (N - 1), nx * sizeof(double));
    if (c->ter) memcpy(c->xN_ref, c->r_ter, nx * sizeof(double));
    if (c->recompute_cost_hess) update_mpc_f0_hess(c);

    /* update_mpc_beq: beq_0 = -A(:,:,1) s_prev */
    for (int i = 0; i < nx; i++) {
        double a = 0.0;
        for (int j = 0; j < nx; j++) a += M(c->A, i, j, nx) * s_prev[j];
        c->beq_0[i] = -a;
    }

    int continue_Newton = 1, iter = 0;
    get_mpc_variables(c, s_prev, u_prev);
    double lambda2 = 1.0;
    while (c->op.eps <= lambda2 * 0.5 && continue_Newton && iter < c->op.max_iter) {
        grad_f0_MPC(c);
        equality_residuals(c);
        if (c->ng0 || c->ngk || c->nter) inequality_residuals(c);
        reduced_KKT_elements(c);
        riccati_KKT_du(c);
        if (c->ng0 || c->ngk || c->nter) recover_slacks(c);
        lambda2 = get_lambda2(c);
        continue_Newton = line_search_local(c, continue_Newton);
        get_mpc_variables(c, s_prev, u_prev);
        iter++;
    }
    if (u0) memcpy(u0, c->u, c->nu * sizeof(double));
    return iter;
}

/* ---------------------------------------------------------------------------------------------------- */
/* build_chronos_mpc.m: initialize_inequality_slacks e rollstates                                         */

static void inicializa_folgas(chronos *c)
{
    int N = c->N, ngk = c->ngk, nvk = c->nvk;
    double e = c->op.slack_epsilon;
    if (!(c->ng0 || c->ngk || c->nter)) return;
    memset(c->g_0, 0, c->ng0 * sizeof(double));
    memset(c->g_k, 0, (size_t)ngk * (N - 1) * sizeof(double));
    memset(c->g_ter, 0, c->nter * sizeof(double));
    memset(c->v_k, 0, (size_t)nvk * (N - 1) * sizeof(double));
    memset(c->v_ter, 0, c->nter * sizeof(double));
    inequality_residuals(c);
    for (int i = 0; i < c->ng0; i++) c->g_0[i] = dmax(-c->ri_0[i], e);
    for (int k = 0; k < N - 1; k++) {
        for (int i = 0; i < ngk; i++) M(c->g_k, i, k, ngk) = dmax(-M(c->ri_k, i, k, ngk), e);
        for (int j = 0; j < nvk; j++) {
            double ri = M(c->ri_k, c->v_rows_k[j], k, ngk);
            M(c->g_k, c->v_rows_k[j], k, ngk) = e + dmax(-ri, 0.0);
            M(c->v_k, j, k, nvk) = e + dmax(ri, 0.0);
        }
    }
    for (int i = 0; i < c->nter; i++) {
        c->g_ter[i] = e + dmax(-c->ri_ter[i], 0.0);
        c->v_ter[i] = e + dmax(c->ri_ter[i], 0.0);
    }
    inequality_residuals(c);
}

void chronos_reinicia_folgas(chronos *c, const double *s_prev, const double *u_prev)
{
    get_mpc_variables(c, s_prev, u_prev);
    inicializa_folgas(c);
}

void chronos_inicializa(chronos *c, const double *s_prev, const double *u_prev)
{
    int nx = c->nx, nu = c->nu, N = c->N;
    size_t pa = (size_t)nx * nx, pb = (size_t)nx * nu;
    double e = c->op.slack_epsilon;
    double *x = zeros(nx), *xn = zeros(nx), *up = copia(u_prev, nu), *uk = zeros(nu);
    memcpy(x, s_prev, nx * sizeof(double));
    /* rollstates (sem x_ref: u_raw = u_prev) */
    for (int k = 0; k < N; k++) {
        for (int i = 0; i < nu; i++) {
            double v = up[i];
            if (c->rs.du) {
                v = dmax(up[i] + (c->du_min[i] + e), v);
                v = dmin(up[i] + (c->du_max[i] - e), v);
            }
            if (c->rs.u) {
                v = dmax(c->u_min[i] + e, v);
                v = dmin(c->u_max[i] - e, v);
            }
            uk[i] = v;
        }
        for (int i = 0; i < nx; i++) {
            double as = 0.0, bu = 0.0;
            for (int j = 0; j < nx; j++) as += c->A[k * pa + i + (size_t)j * nx] * x[j];
            for (int j = 0; j < nu; j++) bu += c->B[k * pb + i + (size_t)j * nx] * uk[j];
            xn[i] = as + bu;
            if (c->rs.estado) {
                xn[i] = dmax(c->s_min[i] + e, xn[i]);
                xn[i] = dmin(c->s_max[i] - e, xn[i]);
            }
        }
        for (int i = 0; i < nu; i++) { M(c->u, i, k, nu) = uk[i]; M(c->su, i, k, nu) = up[i]; up[i] = uk[i]; }
        for (int i = 0; i < nx; i++) { M(c->s, i, k, nx) = xn[i]; x[i] = xn[i]; }
    }
    free(x); free(xn); free(up); free(uk);
    get_mpc_variables(c, s_prev, u_prev);
    inicializa_folgas(c);
}

/* ---------------------------------------------------------------------------------------------------- */
/* criação (init_mpc*.m, assign_inequalities.m, init_costs.m)                                            */

chronos *chronos_cria(int nx, int nu, int N, chronos_opcoes op, chronos_restricoes rs,
                      const double *Qe, const double *Rdu, const double *P,
                      const double *s_min, const double *s_max, const double *qv_min, const double *qv_max,
                      const double *u_min, const double *u_max, const double *du_min, const double *du_max)
{
    chronos *c = (chronos *)calloc(1, sizeof(chronos));
    int nse = nx + nu, n1 = N - 1, row;
    c->nx = nx; c->nu = nu; c->N = N; c->nse = nse; c->op = op; c->rs = rs;

    /* assign_inequalities.m */
    row = 0;
    c->r0_umin = c->r0_umax = c->r0_dumin = c->r0_dumax = -1;
    if (rs.u) { c->r0_umin = row; row += nu; c->r0_umax = row; row += nu; }
    if (rs.du) { c->r0_dumin = row; row += nu; c->r0_dumax = row; row += nu; }
    c->ng0 = row;
    row = 0;
    c->rk_smin = c->rk_smax = c->rk_umin = c->rk_umax = c->rk_dumin = c->rk_dumax = -1;
    if (rs.estado) { c->rk_smin = row; c->vk_smin = 0; row += nx; c->rk_smax = row; c->vk_smax = nx; row += nx; }
    if (rs.u) { c->rk_umin = row; row += nu; c->rk_umax = row; row += nu; }
    if (rs.du) { c->rk_dumin = row; row += nu; c->rk_dumax = row; row += nu; }
    c->ngk = row;
    c->nvk = rs.estado ? 2 * nx : 0;
    c->v_rows_k = (int *)calloc(c->nvk > 0 ? c->nvk : 1, sizeof(int));
    for (int j = 0; j < c->nvk; j++) c->v_rows_k[j] = j;      /* folgas de estado nas primeiras linhas */
    c->nter = rs.estado ? 2 * nx : 0;
    c->rt_smin = rs.estado ? 0 : -1;
    c->rt_smax = rs.estado ? nx : -1;

    /* dados */
    c->A = zeros((size_t)nx * nx * N); c->B = zeros((size_t)nx * nu * N);
    c->Qe = copia(Qe, (size_t)nx * nx); c->Rdu = copia(Rdu, (size_t)nu * nu);
    c->wq = zeros((size_t)nx * N);
    for (int k = 0; k < nx * N; k++) c->wq[k] = 1.0;
    c->P2 = zeros((size_t)nx * nx);
    c->ter = P != NULL;
    if (P) for (int i = 0; i < nx * nx; i++) c->P2[i] = 2.0 * P[i];
    c->s_min = copia(s_min, nx); c->s_max = copia(s_max, nx);
    c->u_min = copia(u_min, nu); c->u_max = copia(u_max, nu);
    c->du_min = copia(du_min, nu); c->du_max = copia(du_max, nu);
    c->grad_qv_k = zeros((size_t)c->nvk * n1); c->grad_qv_ter = zeros(c->nter);
    if (rs.estado) {
        for (int k = 0; k < n1; k++)
            for (int i = 0; i < nx; i++) {
                M(c->grad_qv_k, c->vk_smin + i, k, c->nvk) = qv_min[i];
                M(c->grad_qv_k, c->vk_smax + i, k, c->nvk) = qv_max[i];
            }
        for (int i = 0; i < nx; i++) { c->grad_qv_ter[c->rt_smin + i] = qv_min[i]; c->grad_qv_ter[c->rt_smax + i] = qv_max[i]; }
    }
    c->r = zeros((size_t)nx * n1); c->r_ter = zeros(nx); c->xN_ref = zeros(nx); c->beq_0 = zeros(nx);

    /* iterado */
    c->s = zeros((size_t)nx * N); c->u = zeros((size_t)nu * N); c->su = zeros((size_t)nu * N); c->du = zeros((size_t)nu * N);
    c->s_ter = zeros(nx); c->err = zeros((size_t)nx * n1); c->err_ter = zeros(nx);
    c->g_0 = zeros(c->ng0); c->g_k = zeros((size_t)c->ngk * n1); c->g_ter = zeros(c->nter);
    c->v_k = zeros((size_t)c->nvk * n1); c->v_ter = zeros(c->nter);

    /* custo */
    c->grad_u_f0_0 = zeros(nu); c->grad_se_f0_k = zeros((size_t)nse * n1); c->grad_u_f0_k = zeros((size_t)nu * n1);
    c->grad_se_f0_ter = zeros(nse);
    c->R_f0_0 = zeros((size_t)nu * nu); c->Q_f0_k = zeros((size_t)nse * nse * n1); c->R_f0_k = zeros((size_t)nu * nu * n1);
    c->Y_f0_k = zeros((size_t)nu * nse * n1); c->Q_f0_ter = zeros((size_t)nse * nse);

    /* KKT */
    c->ru_0 = zeros(nu); c->rse_k = zeros((size_t)nse * n1); c->ru_k = zeros((size_t)nu * n1); c->rse_ter = zeros(nse);
    c->R_0 = zeros((size_t)nu * nu); c->Q_k = zeros((size_t)nse * nse * n1); c->R_k = zeros((size_t)nu * nu * n1);
    c->Y_k = zeros((size_t)nu * nse * n1); c->Q_ter = zeros((size_t)nse * nse);
    c->rp_0 = zeros(nse); c->rp_k = zeros((size_t)nse * n1);
    c->ri_0 = zeros(c->ng0); c->g2_0 = zeros(c->ng0); c->iS_0 = zeros(c->ng0); c->iS_ri_hat_0 = zeros(c->ng0);
    c->ri_k = zeros((size_t)c->ngk * n1); c->g2_k = zeros((size_t)c->ngk * n1); c->iS_k = zeros((size_t)c->ngk * n1);
    c->iS_ri_hat_k = zeros((size_t)c->ngk * n1); c->v2_k = zeros((size_t)c->nvk * n1); c->rv_v2_k = zeros((size_t)c->nvk * n1);
    c->ri_ter = zeros(c->nter); c->g2_ter = zeros(c->nter); c->iS_ter = zeros(c->nter); c->iS_ri_hat_ter = zeros(c->nter);
    c->v2_ter = zeros(c->nter); c->rv_v2_ter = zeros(c->nter);

    /* Riccati */
    c->Q_hat = zeros((size_t)nse * nse * N); c->R_hat = zeros((size_t)nu * nu * n1); c->Y_hat = zeros((size_t)nu * nse * n1);
    c->K_ric = zeros((size_t)nu * nse * n1); c->d_ric = zeros((size_t)nu * n1); c->rs_hat = zeros((size_t)nse * N);
    c->rp_hat = zeros((size_t)nse * n1); c->ru_hat = zeros((size_t)nu * n1);
    c->R_hat_0 = zeros((size_t)nu * nu); c->d_ric_0 = zeros(nu); c->rp_hat_0 = zeros(nse); c->ru_hat_0 = zeros(nu);
    c->QA_ric = zeros((size_t)nx * nx); c->G_ric = zeros((size_t)nu * nx); c->rhs = zeros((size_t)nu * (nse + 1));
    c->ldl = zeros((size_t)nu * nu);

    /* passo */
    c->delta_u = zeros((size_t)nu * N); c->delta_se = zeros((size_t)nse * N);
    c->delta_g_0 = zeros(c->ng0); c->delta_g_k = zeros((size_t)c->ngk * n1); c->delta_g_ter = zeros(c->nter);
    c->delta_v_k = zeros((size_t)c->nvk * n1); c->delta_v_ter = zeros(c->nter);

    c->recompute_cost_hess = 1;
    return c;
}

void chronos_dinamica(chronos *c, const double *A, const double *B)
{
    memcpy(c->A, A, (size_t)c->nx * c->nx * c->N * sizeof(double));
    memcpy(c->B, B, (size_t)c->nx * c->nu * c->N * sizeof(double));
}

void chronos_limites_du(chronos *c, const double *du_min, const double *du_max)
{
    memcpy(c->du_min, du_min, (size_t)c->nu * sizeof(double));
    memcpy(c->du_max, du_max, (size_t)c->nu * sizeof(double));
}

void chronos_peso_estagio(chronos *c, const double *w)
{
    memcpy(c->wq, w, (size_t)c->nx * c->N * sizeof(double));
    c->recompute_cost_hess = 1;
}

void chronos_terminal(chronos *c, const double *P)
{
    if (!c->ter) return;                     /* update_mpc_ter_ingredients: só se houver terminal */
    for (int i = 0; i < c->nx * c->nx; i++) c->P2[i] = 2.0 * P[i];
    c->recompute_cost_hess = 1;
}

double *chronos_s(chronos *c) { return c->s; }
double *chronos_u(chronos *c) { return c->u; }
double *chronos_su(chronos *c) { return c->su; }
double *chronos_g_0(chronos *c) { return c->g_0; }
double *chronos_g_k(chronos *c) { return c->g_k; }
double *chronos_g_ter(chronos *c) { return c->g_ter; }
double *chronos_v_k(chronos *c) { return c->v_k; }
double *chronos_v_ter(chronos *c) { return c->v_ter; }
void chronos_tamanhos(const chronos *c, int *n) { n[0] = c->ng0; n[1] = c->ngk; n[2] = c->nvk; n[3] = c->nter; }

void chronos_libera(chronos *c)
{
    if (!c) return;
    double **p[] = {&c->wq, &c->A, &c->B, &c->Qe, &c->Rdu, &c->P2, &c->s_min, &c->s_max, &c->u_min, &c->u_max, &c->du_min,
                    &c->du_max, &c->grad_qv_k, &c->grad_qv_ter, &c->r, &c->r_ter, &c->xN_ref, &c->beq_0, &c->s, &c->u,
                    &c->su, &c->du, &c->s_ter, &c->err, &c->err_ter, &c->g_0, &c->g_k, &c->g_ter, &c->v_k, &c->v_ter,
                    &c->grad_u_f0_0, &c->grad_se_f0_k, &c->grad_u_f0_k, &c->grad_se_f0_ter, &c->R_f0_0, &c->Q_f0_k,
                    &c->R_f0_k, &c->Y_f0_k, &c->Q_f0_ter, &c->ru_0, &c->rse_k, &c->ru_k, &c->rse_ter, &c->R_0, &c->Q_k,
                    &c->R_k, &c->Y_k, &c->Q_ter, &c->rp_0, &c->rp_k, &c->ri_0, &c->g2_0, &c->iS_0, &c->iS_ri_hat_0,
                    &c->ri_k, &c->g2_k, &c->iS_k, &c->iS_ri_hat_k, &c->v2_k, &c->rv_v2_k, &c->ri_ter, &c->g2_ter,
                    &c->iS_ter, &c->iS_ri_hat_ter, &c->v2_ter, &c->rv_v2_ter, &c->Q_hat, &c->R_hat, &c->Y_hat,
                    &c->K_ric, &c->d_ric, &c->rs_hat, &c->rp_hat, &c->ru_hat, &c->R_hat_0, &c->d_ric_0, &c->rp_hat_0,
                    &c->ru_hat_0, &c->QA_ric, &c->G_ric, &c->rhs, &c->ldl, &c->delta_u, &c->delta_se, &c->delta_g_0,
                    &c->delta_g_k, &c->delta_g_ter, &c->delta_v_k, &c->delta_v_ter};
    for (size_t i = 0; i < sizeof(p) / sizeof(p[0]); i++) free(*p[i]);
    free(c->v_rows_k);
    free(c);
}
