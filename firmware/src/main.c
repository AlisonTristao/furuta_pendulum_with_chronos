// Pêndulo de Furuta: ESP32 com o MPC embarcado (ou como interface do MPC no PC).
//   núcleo 0: malha interna de 1 kHz (controle.c) + ISR do gerador de passos (stepper.c), protocolo serial
//             (protocolo.c) e telemetria; núcleo 1: só o MPC embarcado (embarcado.c)
// Com -DTESTE_PINOS (ambiente teste_pinos) roda só o teste de bancada dos pinos (teste_pinos.c); com -DBENCH
// (ambiente bench), só o benchmark do MPC (bench.c).
#include "controle.h"
#include "embarcado.h"
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
    embarcado_init();
    protocolo_init();
#endif
}
