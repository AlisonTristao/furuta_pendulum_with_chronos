/* Controlador completo de um período, igual no ESP32 (embarcado) e no sim.exe (ensaio.embarcado = 1):
 * estimador de dθ2, compensação do atraso, NMPC qLPV e u -> θ̈1.
 *
 * Atraso fixo de um período: o θ̈1 calculado com a amostra k é aplicado no instante k+1 (até k+2). Então, na
 * amostra k: o estimador usa o comando que valeu em [k-1, k] (calculado em k-2), a compensação prediz o estado
 * comp_atraso_ms à frente com o comando que vale em [k, k+1] (calculado em k-1; comp_atraso_ms = Ts é exato). */
#ifndef CONTROLADOR_H
#define CONTROLADOR_H

#include "config.h"
#include "estimador.h"

typedef struct {
    config cfg;           /* cópia (θ1_min/max já com o limite do homing) */
    modelo m;
    mpc *c;
    estimador est;
    double a_ap, a_int;   /* θ̈1 aplicado agora (calculado no passo anterior) e no intervalo que terminou */
    double off2;          /* múltiplo de 2π tirado de θ2 na partida */
    double u;             /* último torque do MPC */
    /* Trim do zero do θ2 no equilíbrio (seção [trim]). Com o zero errado em δ, o MPC regula o θ2 medido, mas o
     * pêndulo de verdade fica inclinado e o braço precisa acelerar para um lado; como o braço tem curso e velocidade
     * limitados, a média da inclinação verdadeira no equilíbrio é zero, então a média do θ2 que o controle vê é
     * δ − trim. O integrador trim += (Ts/τ)·θ2 leva o trim a δ com constante de tempo τ. Aprende só no equilíbrio
     * (|θ2| < janela, |dθ2| < 2 rad/s, há pelo menos 1 s, u sem saturar), com |trim| ≤ max. Parte de trim.valor, que
     * vale mesmo com trim.liga = 0 (offset fixo do topo): o topo verdadeiro fica ~1,4° fora do zero medido embaixo
     * (14/11: o trim aprendeu +1,46° e +1,28° com a inclinação da mesa já corrigida; embaixo, só −0,17°). */
    double trim;          /* somado ao zero do θ2 [rad] */
    /* impulso inicial (ensaio.impulso_a/_ms) e o estado em que ele passou o pêndulo ao MPC */
    int imp_ativo;        /* 1 enquanto o impulso comanda */
    double imp_t;         /* duração [s] */
    double imp_x[4];      /* estado na passagem (θ2 como o controle vê: 0 = topo) */
    double imp_e;         /* energia na passagem: 0 = parado embaixo, 1 = parado no topo */
    /* Re-referência (ensaio.reref): no topo, se o pêndulo sai mais de reref_angulo e está se afastando (caindo), o
     * alvo pula para o topo à frente no sentido da queda (off2 += 2π·sinal(θ2)): o MPC deixa ele cair e dar a volta
     * em vez de frear a queda e voltar pelo mesmo lado. Pula na hora, num ângulo fixo: decidir em tempo real por
     * uma simulação rápida ("o braço ainda segura?") perdeu na simulação (15/11: 17,2 s contra 15,2 s em 18 quedas),
     * porque não prevê um empurrão que continua agindo. Uma vez por queda: só arma de novo quando captura o topo
     * novo. No swing-up não age (ainda não capturou). */
    int capturou;         /* está no topo desde a última troca (|θ2| < janela_equilibrio, |dθ2| < 2 rad/s) */
    int rerefs;           /* quantas vezes pulou */
    /* Modo repouso (ensaio.repouso): com o controle desligado o pêndulo fica em pé sozinho, preso pelo atrito seco
     * do pivô, se estiver parado na faixa |θ2| < Tc/(m g l). No topo, com o pêndulo parado nessa faixa (|θ2| <
     * banda_th2, |dθ2| < banda_dth2) e o braço lento (|dθ1| < repouso_dth1), o controle para o braço com |θ̈1| <=
     * repouso_a e o mantém parado, ignorando o MPC (que segue calculando, warm start); sai quando o pêndulo sai da
     * faixa (1,5× banda_th2 ou 2× banda_dth2). Parar o braço tira a aceleração que tirava o pêndulo do atrito. */
    int repousando;
    long n_repouso;       /* períodos em repouso */
    double t_dentro;      /* há quanto tempo está no equilíbrio [s] */
} controlador;

/* Energia do pêndulo (θ2 como o controle vê, 0 = topo): 0 = parado embaixo, 1 = parado no topo. */
double controlador_energia(const controlador *k, const double x[4]);

/* Inclinação da mesa: com o eixo do braço fora da vertical, a vertical verdadeira vista no plano do pêndulo (que
 * gira com o braço) sai do zero do θ2 por um ângulo que varia com θ1 como uma senoide. Com o zero do encoder
 * acertado em θ1 = 0 (inclina.c), o topo verdadeiro fica em θ2 = controlador_inclinacao(e, θ1) [rad]. */
double controlador_inclinacao(const ensaio *e, double theta1);

/* x0: medida na partida (θ2 do firmware: π = embaixo). Devolve 0 se ok. */
int controlador_cria(controlador *k, const config *cfg, const double x0[4]);
void controlador_libera(controlador *k);
/* Um período. x: medida da amostra k; sai como o controle viu (θ2 sem o múltiplo de 2π, dθ2 estimado).
 * t: tempo desde a partida (referência de θ1). Devolve o θ̈1 para aplicar no instante k+1. */
double controlador_passo(controlador *k, double x[4], double t, mpc_info *in);

#endif
