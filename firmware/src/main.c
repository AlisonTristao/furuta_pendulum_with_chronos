// Pêndulo de Furuta: ESP32 como interface entre o MPC e o hardware.
//   núcleo 1: malha interna de 1 kHz (controle.c) + ISR do gerador de passos (stepper.c)
//   núcleo 0: protocolo serial (protocolo.c)
// Com -DTESTE_PINOS (ambiente teste_pinos) roda só o teste de bancada dos pinos (teste_pinos.c); com -DBENCH
// (ambiente bench), só o benchmark do MPC (bench.c).
#include "controle.h"
#include "protocolo.h"

void teste_pinos(void);
void bench(void);

void app_main(void)
{
#if defined(TESTE_PINOS)
    teste_pinos();
#elif defined(BENCH)
    bench();
#else
    controle_init();
    protocolo_init();
#endif
}
