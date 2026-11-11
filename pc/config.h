/* Configuração em arquivo .ini: [fisico], [mpc], [ensaio], [sim]. Chave ausente = valor padrão do artigo. */
#ifndef CONFIG_H
#define CONFIG_H

#include "modelo.h"
#include "qlpv_mpc.h"

typedef struct {
    double Ts, duracao;
    double theta1_ref, t_degrau, theta1_degrau;   /* referência de θ1: theta1_ref, e theta1_degrau a partir de t_degrau */
    double t_disturbio, u_disturbio, dur_disturbio; /* só na simulação: torque somado à entrada */
    double x0[4];                                 /* só na simulação */
    int rk4_sub;                                  /* só na simulação: subpassos da planta */
    double tc2;                                   /* só na simulação: atrito seco do pêndulo [N·m] */
    int planta_acel;                              /* só na simulação: 1 = planta comandada em θ̈1, como o hardware */
    double obs_hz;                                /* só na simulação: dθ2 pelo observador do firmware (0 = ideal) */
    double atraso_ms;                             /* só na simulação: atraso entre a leitura e o novo comando */
    int bits_enc;                                 /* só na simulação: quantização de θ2 (0 = ideal) */
    char porta[32];
    double margem_theta1;                         /* restrição de θ1 do MPC = ±(limite do homing − margem) */
    int usa_limite_homing;                        /* 1: θ1_min/max do MPC vêm do homing */
    double theta2_aborta;                         /* encerra se |dθ2| passar disso (rad/s) */
    int equilibrio;                               /* modo = equilibrio: o controle só assume perto do topo */
    double comp_atraso_ms;                        /* prediz o estado este tempo à frente (com o último comando) antes do MPC */
    double troca_equilibrio;                      /* |θ2| (rad) em que passa ao controle de equilíbrio (volta em 2×) */
    double janela_equilibrio;                     /* |θ2| (rad) em que o modo equilíbrio assume */
} ensaio;

typedef struct {
    fisico f;
    mpc_config mpc;
    double u_max_eq, Rdu_eq;                      /* controle de equilíbrio (u ±u_max_eq); 0 = o mesmo do swing-up */
    ensaio e;
} config;

/* Lê o arquivo (NULL = só padrões). Devolve 0 se ok; imprime chaves desconhecidas em stderr. */
int config_le(const char *arquivo, config *c);
void config_imprime(const config *c);
/* "secao.chave=valor" da linha de comando (ex.: mpc.u_max=1.25). Devolve 0 se ok. */
int config_arg(config *c, const char *arg);
/* Configuração do controle de equilíbrio (a do swing-up com u ±u_max_eq e Rdu_eq). */
mpc_config config_mpc_equilibrio(const config *c);
/* Lê o .ini e aplica os argumentos com '=' a partir de argv[2]; os outros vão para *livre (o 1º). */
int config_le_args(int argc, char **argv, config *c, const char **livre);

#endif
