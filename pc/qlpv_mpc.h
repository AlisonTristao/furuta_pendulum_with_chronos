/* NMPC qLPV do pêndulo de Furuta com o CHRONOS em C (artigo ICIT 27, seções II-III e V):
 * modelo ZOH com ρ médio no estágio, estimativa iterativa da trajetória dos ρ (até l_max QPs, ‖ΔP‖∞ < eps_rho),
 * terminal DLQR opcional.
 *
 * A cada QP do laço dos ρ o iterado do CHRONOS parte de u = su = U (último U do laço; no 1º QP, o U anterior
 * deslocado) e s = rollout de U com a dinâmica do QP, com as folgas recalculadas (reinit_slacks). Isso é o
 * controlador 'icit_chronos' do qlpv_nn_furuta (python/nmpc), usado como referência na validação. */
#ifndef QLPV_MPC_H
#define QLPV_MPC_H

#include "chronos.h"
#include "modelo.h"

typedef struct {
    int Np;
    double Q[4], Rdu;
    double u_min, u_max, du_min, du_max;
    double theta1_min, theta1_max;     /* restrição suave de θ1 */
    double dtheta_max;                 /* restrição suave de |dθ1|, |dθ2| */
    double qv_theta1, qv_dtheta, BIG;  /* pesos L1 das folgas; BIG = "sem limite" em θ2 */
    double peso_exp;                   /* α: peso do estágio k = exp(α((k+1)/N - 1)); 0 = uniforme */
    double peso_exp_estados[4];        /* 1 = o estado recebe o peso exponencial, 0 = peso constante */
    int rho_avg;                       /* 1: ρ médio no estágio (trapézio, eq. 13) */
    int terminal_dlqr;                 /* 1: custo terminal DLQR no topo */
    double Ru_lqr;
    int l_max;
    double eps_rho;
    chronos_opcoes op;
} mpc_config;

/* Configuração do artigo (seção V). */
mpc_config mpc_config_artigo(void);

typedef struct {
    int n_qp, n_it;        /* QPs do laço dos ρ e iterações de Newton somadas */
    double dP;             /* ‖T(P) - P‖∞ do último QP */
    int convergiu, finito;
} mpc_info;

typedef struct mpc mpc;

mpc *mpc_cria(const mpc_config *cfg, const modelo *m, const double x0[4], double u_prev);
void mpc_libera(mpc *c);
/* Troca o limite de |Δu| sem recriar o controlador (mantém o warm start, projetado no limite novo). */
void mpc_limite_du(mpc *c, double du_max);

/* Um passo: x medido, ref (4, constante no horizonte). Devolve o torque u (saturado; 0 se não finito). */
double mpc_passo(mpc *c, const double x[4], const double ref[4], mpc_info *info);

/* Predição do último passo: U (Np) e s (4×Np, coluna k = x(k+1|k)). */
const double *mpc_U(const mpc *c);
const double *mpc_S(const mpc *c);
/* Matriz terminal usada (4×4, coluna; zeros sem terminal). */
const double *mpc_P_terminal(const mpc *c);

#endif
