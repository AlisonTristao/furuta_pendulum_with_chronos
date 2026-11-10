// Pêndulo de Furuta: ESP32 como interface entre o MPC (PC, Python) e o hardware.
//   núcleo 1: malha interna de 1 kHz (controle.c) + ISR do gerador de passos (stepper.c)
//   núcleo 0: protocolo serial (protocolo.c)
#include "controle.h"
#include "protocolo.h"

void app_main(void)
{
    controle_init();
    protocolo_init();
}
