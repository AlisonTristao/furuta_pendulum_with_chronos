/* Configuração em arquivo .ini: [fisico], [mpc], [ensaio], [sim]. Chave ausente = valor padrão do artigo. */
#ifndef CONFIG_H
#define CONFIG_H

#include <stddef.h>

#include "modelo.h"
#include "qlpv_mpc.h"

typedef struct {
    double Ts, duracao;
    double theta1_ref, t_degrau, theta1_degrau;   /* referência de θ1: theta1_ref, e theta1_degrau a partir de t_degrau */
    double t_disturbio, u_disturbio, dur_disturbio; /* só na simulação: torque somado à entrada */
    double x0[4];                                 /* só na simulação */
    int rk4_sub;                                  /* só na simulação: subpassos da planta */
    double tc2;                                   /* só na simulação: atrito seco do pêndulo [N·m] */
    double offset_th2;                            /* só na simulação: erro do zero do encoder do pêndulo [rad] */
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
    int lado;                                     /* swing-up: θ2 inicial +π (1) ou −π (−1, espelho: impulso invertido) */
    int lado_auto;                                /* furuta -auto: lado = lado_auto·sinal(trim.valor); 0 sorteia */
    double impulso_a, impulso_ms;                 /* impulso inicial (controlador.c): +a na 1ª metade, -a na 2ª [rad/s², ms] */
    int reref;                                    /* 1: depois de uma queda, o alvo pula para o topo à frente (±2π) */
    double reref_angulo;                          /* |θ2| a partir do topo, caindo, em que pula [rad] */
    double banda_th2, banda_dth2;                 /* banda morta no equilíbrio: com |θ2| < banda_th2 [rad] e |dθ2| <
                                                     banda_dth2 [rad/s] o MPC vê o pêndulo parado no topo (0 desliga) */
    double banda_dth1;                            /* idem, exige também |dθ1| < banda_dth1 [rad/s] (0: não olha) */
    int repouso;                                  /* 1: no topo, pêndulo parado na banda (banda_th2/_dth2) e braço lento
                                                     (|dθ1| < repouso_dth1): para o braço e não corrige (controlador.h) */
    double repouso_dth1, repouso_a;               /* [rad/s] para entrar; |θ̈1| máx. para parar o braço [rad/s²] */
    int repouso_solta;                            /* 1: no repouso, com o braço parado, corta a corrente do motor (o
                                                     zumbido do motor parado desfaz o atrito que segura o pêndulo) */
    int estatico;                                 /* só na simulação: atrito estático no pêndulo (Karnopp, com tc2) */
    int embarcado;                                /* 1: controle no ESP32 (malha.exe) / como no ESP32 (sim.exe):
                                                     atraso fixo de um período, controlador.h */
    double janela_equilibrio;                     /* |θ2| (rad) em que o modo equilíbrio assume */
    /* trim do zero do θ2 no equilíbrio, em tempo real (controlador.c, seção [trim]) */
    int trim;                                     /* 1: aprende (trim_valor vale sempre, como offset fixo) */
    double trim_tau;                              /* constante de tempo [s]: bem mais lenta que o equilíbrio */
    double trim_max;                              /* |trim| máximo [rad] */
    double trim_janela;                           /* só aprende com |θ2| abaixo disso [rad] */
    double trim_valor;                            /* trim inicial [rad]; o firmware guarda aqui o aprendido */
    /* inclinação da mesa (seção [inclinacao]): o topo verdadeiro fica em θ2 = seno·sin θ1 + cosseno·(cos θ1 − 1)
     * (controlador_inclinacao). O firmware mede com a varredura do furuta -inclina (inclina.c) */
    int incl_pontos;                              /* posições do braço na varredura (0: o -auto não varre) */
    double incl_grava;                            /* oscilação gravada em cada posição [s] */
    double incl_amplitude;                        /* amplitude nominal (sem atrito) da excitação ressonante [rad] */
    int incl_ciclos;                              /* ciclos da excitação (θ̈1 = A cos(ωn t)) */
    double incl_seno, incl_cosseno;               /* coeficientes medidos [rad] */
} ensaio;

typedef struct {
    fisico f;
    mpc_config mpc;
    ensaio e;
} config;

/* Lê o arquivo (NULL = só padrões). Devolve 0 se ok; imprime chaves desconhecidas em stderr. */
int config_le(const char *arquivo, config *c);
/* Idem com o conteúdo do .ini na memória (firmware: o pc/furuta.ini embutido), sem imprimir nada; devolve -1 se
 * alguma chave não foi reconhecida (as outras são aplicadas). */
int config_le_texto(const char *texto, size_t n, config *c);
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
