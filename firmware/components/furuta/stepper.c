#include "config.h"
#include "stepper.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

// Incremento do acumulador em Q31 (2^31 = 1 passo por tick). O pulso de STEP fica alto um tick e baixo
// no mínimo um tick, então o incremento é limitado a < 0,5 passo/tick (2^30).
#define UM_PASSO   (1u << 31)
#define INC_MAX    ((int32_t)(1u << 30) - 1)

static volatile int32_t s_passos = 0;
static volatile int32_t s_inc = 0;        // passos/tick em Q31, com sinal
static bool s_habilitado = false;

static bool IRAM_ATTR tick(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *ctx)
{
    static uint32_t acc = 0;
    static int dir = 1;
    static bool alto = false;

    int32_t inc = s_inc;
    if (alto) {                                     // fecha o pulso do tick anterior
        gpio_ll_set_level(&GPIO, PIN_STEP, 0);
        alto = false;
    } else if (inc != 0 && (inc > 0 ? 1 : -1) != dir) {
        dir = -dir;                                 // troca o DIR num tick sem pulso (setup >= 1 tick)
        gpio_ll_set_level(&GPIO, PIN_DIR, dir * DIR_SINAL > 0);
        return false;
    }
    acc += (uint32_t)(inc >= 0 ? inc : -inc);
    if (acc >= UM_PASSO) {                          // nunca acontece num tick com pulso alto (inc < 2^30)
        acc -= UM_PASSO;
        gpio_ll_set_level(&GPIO, PIN_STEP, 1);
        alto = true;
        s_passos += dir;
    }
    return false;
}

static void configura_micropasso(void)
{
    int m = 0;                                      // tabela do DRV8825: M2 M1 M0 = log2(micropasso)
    for (int k = MICROPASSO; k > 1; k >>= 1) m++;
    gpio_set_level(PIN_M0, m & 1);
    gpio_set_level(PIN_M1, (m >> 1) & 1);
    gpio_set_level(PIN_M2, (m >> 2) & 1);
}

void stepper_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_EN) |
                        (1ULL << PIN_M0) | (1ULL << PIN_M1) | (1ULL << PIN_M2),
        .mode = GPIO_MODE_INPUT_OUTPUT,             // lê o nível de volta (pinos -motor devolve o DIR igual)
    };
    gpio_config(&out);
    gpio_set_level(PIN_EN, 1);                      // começa desligado
    gpio_set_level(PIN_STEP, 0);
    gpio_set_level(PIN_DIR, DIR_SINAL > 0);         // dir = +1
    configura_micropasso();

    gptimer_handle_t timer;
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    gptimer_new_timer(&cfg, &timer);
    gptimer_alarm_config_t alarme = {
        .alarm_count = 1000000 / PASSO_ISR_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(timer, &alarme);
    gptimer_event_callbacks_t cbs = {.on_alarm = tick};
    gptimer_register_event_callbacks(timer, &cbs, NULL);
    gptimer_enable(timer);
    gptimer_start(timer);
}

void stepper_habilita(bool ligado)
{
    if (!ligado) s_inc = 0;
    gpio_set_level(PIN_EN, ligado ? 0 : 1);
    s_habilitado = ligado;
}

bool stepper_habilitado(void) { return s_habilitado; }

float stepper_v_limite(void)
{
    return (float)INC_MAX / (float)UM_PASSO * PASSO_ISR_HZ / PASSOS_POR_RAD;
}

void stepper_velocidade(float rad_s)
{
    double inc = (double)rad_s * PASSOS_POR_RAD / PASSO_ISR_HZ * (double)UM_PASSO;
    if (inc > INC_MAX) inc = INC_MAX;
    if (inc < -INC_MAX) inc = -INC_MAX;
    s_inc = s_habilitado ? (int32_t)inc : 0;
}

int32_t stepper_passos(void) { return s_passos; }
