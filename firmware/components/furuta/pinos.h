// Teste de bancada dos pinos (pinos.c), chamado pelo terminal do TraceView.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void pinos_entradas(void);
void pinos_i2c(void);
const char *pinos_motor(void);   // NULL se rodou, ou o motivo da recusa

#ifdef __cplusplus
}
#endif
