// Identificação do pêndulo pela oscilação livre (ident.c), chamada pelo terminal: furuta -ident <l2>.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Bloqueia até ~50 s (espera a soltura + 20 s de gravação) e imprime o resultado. NULL se deu certo, ou o motivo.
const char *ident_roda(double l2);
// furuta -calibra: mesma oscilação livre, só o zero do θ2 (centro da oscilação). Até ~50 s.
const char *ident_calibra(void);
// Centro de uma oscilação livre já gravada (φ = θ2 − π, uma amostra a cada 10 ms; o 1º meio-ciclo é descartado),
// pelos extremos (cabeçalho do ident.c). NULL se deu certo, ou o motivo. Usado pela varredura do inclina.c.
const char *ident_centro(const float *phi, int n, double *c, int *ntrincas);

#ifdef __cplusplus
}
#endif
