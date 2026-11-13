/* Planta simulada do pêndulo real, calibrada nos ensaios (seção [sim] do .ini): comandada em θ̈1 como o hardware,
 * com a malha de 1 kHz do firmware (satura a e v, freia antes do limite de software), atraso do comando, atrito
 * seco, ressonância do rotor, θ1 em passos inteiros, θ2 quantizado (encoder) com ruído e o observador de dθ2.
 * Usada pelo sim.exe e pelo furuta -sim do firmware. Tudo em double. */
#ifndef PLANTA_H
#define PLANTA_H

#include "config.h"
#include "modelo.h"

typedef struct {
    ensaio e;
    modelo m;                 /* o do controlador com o atrito da planta (tc2, mu2_planta) */
    double x[4];              /* estado verdadeiro */
    double xm[4];             /* o que o firmware mede: θ1 em passos, θ2 do encoder, dθ1 comandada, dθ2 do observador */
    double fila[256];         /* comando por ms (atraso) */
    long tms;
    double th_est, w_est, q, ppr, th1_med, th1c, v_fw, ws;
    unsigned long long rng;
} planta;

/* Estado inicial = sim.x0 (com sim.semente > 0: θ2 perturbado e o ruído sorteado com essa semente). */
void planta_cria(planta *p, const config *cfg);
/* Avança ensaio.Ts, 1 ms por vez, com o comando a [rad/s²] calculado agora. Com ensaio.embarcado ele entra após um
 * período inteiro (como o firmware); sem, após sim.atraso_ms. Exige sim.planta_acel. */
void planta_passo(planta *p, double a);

#endif
