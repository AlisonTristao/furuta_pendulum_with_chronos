#include "config.h"
#include "homing.h"
#include "stepper.h"

#include <math.h>
#include <stdlib.h>

typedef enum { LIBERA, BUSCA_NEG, SAI_NEG, BUSCA_POS, CENTRO } fase_t;

static fase_t s_fase;
static int32_t s_inicio_fase, s_neg, s_pos;
static const char *s_erro = "";

static void muda(fase_t f, int32_t passos)
{
    s_fase = f;
    s_inicio_fase = passos;
}

void homing_inicia(int32_t passos)
{
    s_erro = "";
    muda(LIBERA, passos);
}

int32_t homing_centro(void) { return (s_neg + s_pos) / 2; }
float homing_meio_curso(void) { return (float)(s_pos - s_neg) / 2.0f / PASSOS_POR_RAD; }
const char *homing_erro(void) { return s_erro; }

static hom_status_t erro(const char *msg, hom_saida_t *saida)
{
    s_erro = msg;
    saida->v_alvo = 0.0f;
    saida->imediato = true;
    return HOM_ERRO;
}

hom_status_t homing_passo(int32_t passos, bool fim_neg, bool fim_pos, float v, hom_saida_t *saida)
{
    saida->imediato = false;
    float andado = fabsf((float)(passos - s_inicio_fase)) / PASSOS_POR_RAD;
    if (andado > HOMING_CURSO_MAX) return erro("CURSO_MAX_SEM_CHAVE", saida);

    switch (s_fase) {
    case LIBERA:                       // começa com uma chave apertada: sai dela devagar
        if (fim_neg && fim_pos) return erro("DUAS_CHAVES_ATIVAS", saida);
        if (fim_neg) saida->v_alvo = HOMING_V / 2;
        else if (fim_pos) saida->v_alvo = -HOMING_V / 2;
        else muda(BUSCA_NEG, passos);
        break;
    case BUSCA_NEG:
        if (fim_pos) return erro("CHAVE_POS_ANDANDO_NEG(DIR_SINAL ou pinos trocados?)", saida);
        saida->v_alvo = -HOMING_V;
        if (fim_neg) {
            s_neg = passos;
            saida->v_alvo = 0.0f;
            saida->imediato = true;
            muda(SAI_NEG, passos);
        }
        break;
    case SAI_NEG:
        saida->v_alvo = HOMING_V / 2;
        if (!fim_neg) muda(BUSCA_POS, passos);
        break;
    case BUSCA_POS:
        if (fim_neg) return erro("CHAVE_NEG_ANDANDO_POS", saida);
        saida->v_alvo = HOMING_V;
        if (fim_pos) {
            s_pos = passos;
            saida->v_alvo = 0.0f;
            saida->imediato = true;
            muda(CENTRO, passos);
        }
        break;
    case CENTRO: {                     // perfil trapezoidal até o meio do curso
        if (fim_neg || fim_pos) {
            if (fabsf((float)(passos - s_inicio_fase)) / PASSOS_POR_RAD > 0.2f)
                return erro("CHAVE_ATIVA_INDO_AO_CENTRO", saida);
        }
        float e = (float)(homing_centro() - passos) / PASSOS_POR_RAD;
        float vmax = sqrtf(2.0f * HOMING_A * fabsf(e));
        if (vmax > HOMING_V) vmax = HOMING_V;
        saida->v_alvo = copysignf(vmax, e);
        if (abs(homing_centro() - passos) <= 1 && fabsf(v) < 0.05f) {
            saida->v_alvo = 0.0f;
            saida->imediato = true;
            return HOM_OK;
        }
        break;
    }
    }
    return HOM_ANDANDO;
}
