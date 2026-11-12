/* CHRONOS em C: porte do solver canônico (github.com/arielmb94/CHRONOS-MPC, commit 8bfab7c,
 * matlab/external/CHRONOS-MPC/solver/mpc_solve.m), restrito aos ingredientes do NMPC qLPV do Furuta:
 *
 *   dinâmica    s_{k+1} = A_k s_k + B_k u_k   (A_k, B_k por estágio: qLPV)
 *   custo       rastreamento dos estados (y = s, peso Qe), Δu (Rdu), terminal 2P (DLQR; P = 0 desliga)
 *   restrições  u e Δu duras; estados suaves (folga L1, peso qv)
 *
 * Cada ingrediente de restrição liga/desliga como os has_* do original. Mesmas variáveis (s, su, u por
 * estágio), mesma recursão de Riccati (riccati_KKT_du.m), mesma busca de passo e mesmo critério de parada.
 * Matrizes em ordem de coluna (como no MATLAB); "página" k de um array 3D = bloco contíguo k.
 *
 * Não portado (não usado no projeto): custo de controle, saídas y ≠ s, custos e restrições customizados,
 * entradas conhecidas d. */
#ifndef CHRONOS_H
#define CHRONOS_H

#ifdef _WIN32
#define CHRONOS_API __declspec(dllexport)
#else
#define CHRONOS_API
#endif

typedef struct chronos chronos;

/* Opções do solver (init_mpc.m): t = 50, Beta = 0,75, min_l = 1e-6, eps = 1e-4, max_iter = 10,
 * slack_epsilon = 1e-3, eps_thknv = 1e-6. */
typedef struct {
    double t, Beta, min_l, eps, slack_epsilon, eps_thknv;
    int max_iter;
} chronos_opcoes;

/* Liga/desliga das restrições (sempre com os dois limites, mín. e máx.). */
typedef struct {
    int estado;   /* s_min <= s <= s_max, suave (qv_min, qv_max) */
    int u;        /* u_min <= u <= u_max, dura */
    int du;       /* du_min <= Δu <= du_max, dura */
} chronos_restricoes;

CHRONOS_API chronos_opcoes chronos_opcoes_padrao(void);

/* Cria o problema. Qe (nx×nx), Rdu (nu×nu), P (nx×nx, terminal; NULL = sem custo terminal).
 * Limites: vetores de tamanho nx (estado, qv) e nu (u, Δu); NULL nos desligados. */
CHRONOS_API chronos *chronos_cria(int nx, int nu, int N, chronos_opcoes op, chronos_restricoes rs,
                                  const double *Qe, const double *Rdu, const double *P,
                                  const double *s_min, const double *s_max,
                                  const double *qv_min, const double *qv_max,
                                  const double *u_min, const double *u_max,
                                  const double *du_min, const double *du_max);
CHRONOS_API void chronos_libera(chronos *c);

/* update_mpc_dynamics: A (nx×nx×N), B (nx×nu×N), página k = estágio k. */
CHRONOS_API void chronos_dinamica(chronos *c, const double *A, const double *B);
/* Peso do rastreamento por estágio e estado (nx×N, coluna k = estágio k, a última é o terminal; padrão 1).
 * O elemento (i, j) de Qe no estágio k é multiplicado por sqrt(w_ik w_jk). */
CHRONOS_API void chronos_peso_estagio(chronos *c, const double *w);
/* update_mpc_ter_ingredients: troca P (nx×nx); P = 0 desliga o custo terminal. */
CHRONOS_API void chronos_terminal(chronos *c, const double *P);

/* build_chronos_mpc: chute inicial (rollstates com u = u_prev saturado) e folgas iniciais. */
CHRONOS_API void chronos_inicializa(chronos *c, const double *s_prev, const double *u_prev);
/* initialize_inequality_slacks (reinit_slacks.m): folgas g, v coerentes com o u, su, s atuais. */
CHRONOS_API void chronos_reinicia_folgas(chronos *c, const double *s_prev, const double *u_prev);

/* mpc_solve: ref (nx×N), coluna k = referência do estágio k (a última também é a terminal).
 * Devolve o número de iterações; u0 (nu) recebe o primeiro controle. */
CHRONOS_API int chronos_resolve(chronos *c, const double *s_prev, const double *u_prev,
                                const double *ref, double *u0);

/* Acesso ao iterado (ordem de coluna): s (nx×N), u (nu×N), su (nu×N), g_0 (ng0), g_k (ngk×(N-1)),
 * g_ter (nter), v_k (nvk×(N-1)), v_ter (nter). */
CHRONOS_API double *chronos_s(chronos *c);
CHRONOS_API double *chronos_u(chronos *c);
CHRONOS_API double *chronos_su(chronos *c);
CHRONOS_API double *chronos_g_0(chronos *c);
CHRONOS_API double *chronos_g_k(chronos *c);
CHRONOS_API double *chronos_g_ter(chronos *c);
CHRONOS_API double *chronos_v_k(chronos *c);
CHRONOS_API double *chronos_v_ter(chronos *c);
/* Tamanhos: [ng0, ngk, nvk, nter]. */
CHRONOS_API void chronos_tamanhos(const chronos *c, int *n);

#endif
