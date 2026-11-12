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

/* Configuração e predição no tipo do controlador (real.h: double no PC, float no ESP32); estado e u em double. */
typedef struct {
    int Np;
    real Q[4], Rdu;
    real u_min, u_max, du_min, du_max;
    real theta1_min, theta1_max;     /* restrição suave de θ1 */
    real dtheta_max;                 /* restrição suave de |dθ1|, |dθ2| */
    real qv_theta1, qv_dtheta, BIG;  /* pesos L1 das folgas; BIG = "sem limite" em θ2 */
    real peso_exp;                   /* α: peso do estágio k = exp(α((k+1)/N - 1)); 0 = uniforme */
    real peso_exp_estados[4];        /* 1 = o estado recebe o peso exponencial, 0 = peso constante */
    real horizonte;                  /* > Np·Ts: passo do estágio k cresce em reta, h_k = Ts + k·inc, até somar
                                        horizonte [s] (u constante em cada estágio = blocagem); 0 = uniforme */
    int rho_avg;                       /* 1: ρ médio no estágio (trapézio, eq. 13) */
    int terminal_dlqr;                 /* 1: custo terminal DLQR no topo */
    real Ru_lqr;
    int l_max;
    real eps_rho;
    chronos_opcoes op;
} mpc_config;

/* Configuração do artigo (seção V). */
mpc_config mpc_config_artigo(void);

typedef struct {
    int n_qp, n_it;        /* QPs do laço dos ρ e iterações de Newton somadas */
    real dP;             /* ‖T(P) - P‖∞ do último QP */
    int convergiu, finito;
    int cortado;           /* 1: o laço dos ρ parou pelo orçamento de tempo */
} mpc_info;

typedef struct mpc mpc;

mpc *mpc_cria(const mpc_config *cfg, const modelo *m, const double x0[4], double u_prev);
void mpc_libera(mpc *c);

/* Um passo: x medido, ref (4, constante no horizonte). Devolve o torque u (saturado; 0 se não finito). */
double mpc_passo(mpc *c, const double x[4], const double ref[4], mpc_info *info);

/* Predição do último passo: U (Np) e s (4×Np, coluna k = x(k+1|k)). */
const real *mpc_U(const mpc *c);
const real *mpc_S(const mpc *c);
/* Matriz terminal usada (4×4, coluna; zeros sem terminal). */
const real *mpc_P_terminal(const mpc *c);
/* Orçamento de tempo por passo (embarcado): depois de cada QP, só faz outro se o tempo gasto mais o do último
 * QP couber em orcamento_us. relogio_us: relógio em µs; NULL desliga. */
void mpc_orcamento(mpc *c, long long (*relogio_us)(void), long long orcamento_us);

#endif
