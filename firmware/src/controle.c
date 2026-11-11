#include "config.h"
#include "controle.h"
#include "encoder.h"
#include "homing.h"
#include "stepper.h"

#include <math.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PI_F 3.14159265359f

// ------------------------------------------------------------- estado compartilhado (núcleo 0 <-> núcleo 1)
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static leitura_t s_leitura;
static volatile estado_t s_estado = EST_OCIOSO;
static const char *volatile s_erro = "";
static volatile bool s_fim_neg, s_fim_pos, s_flt;
static volatile float s_meio_curso = 0.0f, s_limite = 0.0f;

// pedidos da tarefa de comunicação, consumidos pela malha
static volatile float s_a_cmd = 0.0f;
static volatile int64_t s_t_cmd = 0;
static volatile bool s_novo_cmd = false;
static volatile bool s_ped_homing, s_ped_origem, s_ped_zera;
static volatile int s_ped_hab = -1;
static volatile bool s_pronto_init = false;

// ------------------------------------------------------------- estado da malha (só núcleo 1)
static float s_v = 0.0f;               // velocidade comandada do braço (rad/s)
static int64_t s_t_pronto = 0;         // última vez que a malha não estava em PRONTO (us)
static int32_t s_offset = 0;           // passos em theta1 = 0
static int s_erros_enc = 0;

typedef struct {
    gpio_num_t pino;
    bool estado;
    int cont;
} chave_t;

static chave_t s_chave_neg = {PIN_FIM_NEG, false, 0};
static chave_t s_chave_pos = {PIN_FIM_POS, false, 0};

static bool debounce(chave_t *c)
{
    bool bruto = gpio_get_level(c->pino) == FIM_NIVEL_ATIVO;
    if (bruto == c->estado) c->cont = 0;
    else if (++c->cont >= FIM_DEBOUNCE_MS * CTRL_HZ / 1000) {
        c->estado = bruto;
        c->cont = 0;
    }
    return c->estado;
}

static float rampa(float v, float alvo, float dv)
{
    if (alvo > v + dv) return v + dv;
    if (alvo < v - dv) return v - dv;
    return alvo;
}

static void falha(const char *msg)
{
    s_erro = msg;
    s_estado = EST_FALHA;
    s_v = 0.0f;
    stepper_habilita(false);
}

// Integra a aceleração pedida, freando antes dos limites de software (distância de parada com A_MAX).
static float malha_rodando(float theta1, float dt)
{
    float a = s_a_cmd;
    if (a > A_MAX) a = A_MAX;
    if (a < -A_MAX) a = -A_MAX;
    float v = s_v + a * dt;
    float parada = v * v / (2.0f * A_MAX);
    if (v > 0 && theta1 + parada >= s_limite) v = fmaxf(s_v - A_MAX * dt, 0.0f);
    if (v < 0 && theta1 - parada <= -s_limite) v = fminf(s_v + A_MAX * dt, 0.0f);
    return v;
}

static void processa_pedidos(int32_t passos)
{
    if (s_ped_hab >= 0) {
        bool ligar = s_ped_hab;
        s_ped_hab = -1;
        s_v = 0.0f;
        stepper_habilita(ligar);
        s_estado = EST_OCIOSO;          // com o driver desligado o braço pode ter sido movido: refazer homing
        s_erro = "";
    }
    if (s_ped_zera) {
        s_ped_zera = false;
        encoder_define(PI_F);
    }
    if (s_ped_origem) {
        s_ped_origem = false;
        if (s_estado != EST_HOMING && s_estado != EST_RODANDO) {
            s_offset = passos;
            s_meio_curso = LIM_SEM_HOMING_RAD;
            s_limite = LIM_SEM_HOMING_RAD;
            s_v = 0.0f;
            s_erro = "";
            stepper_habilita(true);
            s_estado = EST_PRONTO;
        }
    }
    if (s_ped_homing) {
        s_ped_homing = false;
        s_v = 0.0f;
        s_erro = "";
        stepper_habilita(true);
        homing_inicia(passos);
        s_estado = EST_HOMING;
    }
}

static void tarefa(void *arg)
{
    stepper_init();
    encoder_init();
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_FIM_NEG) | (1ULL << PIN_FIM_POS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = FIM_PULL_UP,
        .pull_down_en = !FIM_PULL_UP,
    };
    gpio_config(&in);
    gpio_config_t flt = {.pin_bit_mask = 1ULL << PIN_FLT, .mode = GPIO_MODE_INPUT};
    gpio_config(&flt);
    s_pronto_init = true;

    const float dt = 1.0f / CTRL_HZ;
    TickType_t ultimo = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS(1000 / CTRL_HZ));
        int32_t passos = stepper_passos();
        processa_pedidos(passos);

        // sensores
        if (encoder_ok()) {
            if (encoder_atualiza(dt)) s_erros_enc = 0;
            else if (++s_erros_enc >= ENC_ERROS_MAX && s_estado != EST_FALHA) falha("ENCODER_I2C");
        }
        bool fneg = debounce(&s_chave_neg), fpos = debounce(&s_chave_pos);
        bool flt = USA_PINO_FALHA && gpio_get_level(PIN_FLT) == 0;
        s_fim_neg = fneg;
        s_fim_pos = fpos;
        s_flt = flt;
        float theta1 = (float)(passos - s_offset) / PASSOS_POR_RAD;

        if (flt && stepper_habilitado()) falha("DRV8825_FAULT");
        int64_t agora = esp_timer_get_time();
        if (s_estado != EST_PRONTO) s_t_pronto = agora;   // conta o tempo parado em PRONTO

        switch (s_estado) {
        case EST_HOMING: {
            hom_saida_t h;
            hom_status_t st = homing_passo(passos, fneg, fpos, s_v, &h);
            s_v = h.imediato ? h.v_alvo : rampa(s_v, h.v_alvo, HOMING_A * dt);
            if (st == HOM_OK) {
                s_offset = homing_centro();
                s_meio_curso = homing_meio_curso();
                s_limite = s_meio_curso - FIM_MARGEM_RAD;
                s_estado = s_limite > 0 ? EST_PRONTO : EST_FALHA;
                if (s_limite <= 0) s_erro = "CURSO_MENOR_QUE_MARGEM";
            } else if (st == HOM_ERRO) {
                falha(homing_erro());
            }
            break;
        }
        case EST_RODANDO:
            s_novo_cmd = false;
            if (esp_timer_get_time() - s_t_cmd > WATCHDOG_MS * 1000LL) {
                s_estado = EST_PRONTO;      // sem comando: freia em PRONTO
                s_v = rampa(s_v, 0.0f, A_MAX * dt);
            } else {
                s_v = malha_rodando(theta1, dt);
            }
            break;
        case EST_PRONTO:
            if (s_novo_cmd) {               // primeiro comando da malha externa
                s_novo_cmd = false;
                s_estado = EST_RODANDO;
                s_v = malha_rodando(theta1, dt);
                break;
            }
            s_v = rampa(s_v, 0.0f, A_MAX * dt);
            if (agora - s_t_pronto > DESLIGA_OCIOSO_MS * 1000LL && s_v == 0.0f) {
                stepper_habilita(false);    // parado sem uso: desliga o driver (esquenta); refazer o homing
                s_estado = EST_OCIOSO;
            }
            break;
        case EST_OCIOSO:
            s_v = rampa(s_v, 0.0f, A_MAX * dt);
            break;
        case EST_FALHA:
            s_v = 0.0f;
            break;
        }
        // chave acionada fora do homing = os limites de software falharam
        if ((s_estado == EST_RODANDO || s_estado == EST_PRONTO) && (fneg || fpos))
            falha(fneg ? "FIM_DE_CURSO_NEG" : "FIM_DE_CURSO_POS");

        if (s_v > V_MAX) s_v = V_MAX;
        if (s_v < -V_MAX) s_v = -V_MAX;
        stepper_velocidade(s_v);

        leitura_t x = {theta1, encoder_angulo(), s_v, encoder_velocidade()};
        taskENTER_CRITICAL(&s_mux);
        s_leitura = x;
        taskEXIT_CRITICAL(&s_mux);
    }
}

void controle_init(void)
{
    xTaskCreatePinnedToCore(tarefa, "controle", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    while (!s_pronto_init) vTaskDelay(1);
}

estado_t controle_estado(void) { return s_estado; }

const char *controle_estado_nome(estado_t e)
{
    static const char *nomes[] = {"IDLE", "HOMING", "READY", "RUN", "FAULT"};
    return nomes[e];
}

const char *controle_erro(void) { return s_erro; }
bool controle_fim_neg(void) { return s_fim_neg; }
bool controle_fim_pos(void) { return s_fim_pos; }
bool controle_falha_driver(void) { return s_flt; }
float controle_meio_curso(void) { return s_meio_curso; }
float controle_limite(void) { return s_limite; }

void controle_leitura(leitura_t *x)
{
    taskENTER_CRITICAL(&s_mux);
    *x = s_leitura;
    taskEXIT_CRITICAL(&s_mux);
}

bool controle_acel(float a)
{
    estado_t e = s_estado;
    if (e != EST_PRONTO && e != EST_RODANDO) return false;
    s_a_cmd = a;
    s_t_cmd = esp_timer_get_time();
    s_novo_cmd = true;                    // a malha passa de PRONTO para RODANDO
    return true;
}

bool controle_homing(void)
{
    if (s_estado == EST_RODANDO) return false;
    s_ped_homing = true;
    while (s_ped_homing) vTaskDelay(1);   // espera a malha aceitar (estado vira HOMING)
    return true;
}

static void espera(volatile bool *ped)
{
    while (*ped) vTaskDelay(1);
}

void controle_origem_aqui(void)
{
    s_ped_origem = true;
    espera(&s_ped_origem);
}

void controle_zera_pendulo(void)
{
    s_ped_zera = true;
    espera(&s_ped_zera);
}

void controle_habilita(bool ligado)
{
    s_ped_hab = ligado;
    while (s_ped_hab >= 0) vTaskDelay(1);
}
