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
    double ruido_enc;                             /* só na simulação: ruído de θ2 (desvio, em LSB do encoder) */
    double passos_rev;                            /* só na simulação: micropassos por volta (θ1 em passos inteiros; 0 = ideal) */
    double a_max_fw, v_max_fw, limite_fw;
    double f_motor, zeta_motor;
    double mu2_planta;                            /* só na simulação: atrito viscoso do pêndulo na planta (0 = o do modelo) */
    double ripple_passo;                          /* só na simulação: erro do micropasso, fração do passo inteiro (com f_motor > 0) */
    int semente;                                  /* só na simulação: > 0 sorteia ruído e perturba θ2 inicial (±0,05 rad) */
    char porta[32];
    double margem_theta1;                         /* restrição de θ1 do MPC = ±(limite do homing − margem) */
    int usa_limite_homing;                        /* 1: θ1_min/max do MPC vêm do homing */
    double theta2_aborta;                         /* encerra se |dθ2| passar disso (rad/s) */
    int equilibrio;                               /* modo = equilibrio: o controle só assume perto do topo */
    double comp_atraso_ms;                        /* prediz o estado este tempo à frente (com o último comando) antes do MPC */
    int estimador;                                /* 1: dθ2 pelo estimador com modelo no PC (estimador.h) */
    double est_L1, est_L2;
    double impulso_a, impulso_ms;                 /* impulso inicial (controlador.c): +a na 1ª metade, -a na 2ª [rad/s², ms] */
    int embarcado;                                /* 1: controle no ESP32 (malha.exe) / como no ESP32 (sim.exe):
                                                     atraso fixo de um período, controlador.h */
    double janela_equilibrio;                     /* |θ2| (rad) em que o modo equilíbrio assume */
} ensaio;

typedef struct {
    fisico f;
    mpc_config mpc;
    ensaio e;
} config;

/* Lê o arquivo (NULL = só padrões). Devolve 0 se ok; imprime chaves desconhecidas em stderr. */
int config_le(const char *arquivo, config *c);
void config_imprime(const config *c);
/* "secao.chave=valor" da linha de comando (ex.: mpc.u_max=1.25). Devolve 0 se ok. */
int config_arg(config *c, const char *arg);
/* Idem sem imprimir nada (firmware: o stderr sai na UART do protocolo). */
int config_arg_silencioso(config *c, const char *arg);
/* Lê o .ini e aplica os argumentos com '=' a partir de argv[2]; os outros vão para *livre (o 1º). */
int config_le_args(int argc, char **argv, config *c, const char **livre);
/* Os pares "secao.chave=valor" do arquivo (seções em `secoes`, ex.: "fisico mpc ensaio") e depois os argumentos
 * com '=' de argv[2..], na ordem: cb recebe cada um (para repassar a configuração ao firmware). */
int config_pares(const char *arquivo, int argc, char **argv, const char *secoes,
                 int (*cb)(const char *par, void *ctx), void *ctx);

#endif
