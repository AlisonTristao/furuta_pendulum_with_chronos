// Busca dos fins de curso: vai até o negativo, depois até o positivo, e para no meio (theta1 = 0).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum { HOM_ANDANDO, HOM_OK, HOM_ERRO } hom_status_t;

typedef struct {
    float v_alvo;      // rad/s pedida para a rampa
    bool imediato;     // true: aplicar v_alvo sem rampa (parada na chave)
} hom_saida_t;

void homing_inicia(int32_t passos);
hom_status_t homing_passo(int32_t passos, bool fim_neg, bool fim_pos, float v, hom_saida_t *saida);

int32_t homing_centro(void);             // passos no meio do curso
float homing_meio_curso(void);           // rad, metade do curso entre as chaves
const char *homing_erro(void);
