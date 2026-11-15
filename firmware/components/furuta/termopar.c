/* Termopar do driver: MAX6675 por SPI feito à mão (bit-bang), só leitura.
 *
 * Quadro de 16 bits, MSB primeiro, válido depois de CS cair: D15 = 0, D14..D3 = temperatura (12 bits, 0,25 °C),
 * D2 = 1 se o termopar está aberto, D1 = id, D0 = estado. O bit sai na borda de descida do SCK; lê com o SCK baixo e
 * sobe. Subir o CS começa uma conversão nova (~220 ms): ler mais rápido que isso devolve a mesma e a interrompe.
 * Sem módulo, o SO fica no pull-up e vem 0xFFFF (o D15 do MAX6675 é sempre 0).
 *
 * Por que importa: a varredura da mesa (furuta -inclina) deixa o driver ligado ~1 min, o motor perde torque e o
 * swing-up logo depois sai ruim (14/11). Com a temperatura no tópico e no log dá para ver quanto. */
#include "config.h"
#include "termopar.h"

#include <math.h>
#include <stdint.h>

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PERIODO_MS 250

static volatile float s_temp = NAN;
static const char *volatile s_estado = "iniciando";

static uint16_t le_quadro(void)
{
    uint16_t v = 0;
    gpio_set_level(PIN_TC_CS, 0);
    esp_rom_delay_us(2);
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(PIN_TC_SCK, 0);
        esp_rom_delay_us(2);   // MAX6675: SCK até 4,3 MHz; folga para fio comprido
        if (gpio_get_level(PIN_TC_SO)) v |= (uint16_t)(1U << i);
        gpio_set_level(PIN_TC_SCK, 1);
        esp_rom_delay_us(2);
    }
    gpio_set_level(PIN_TC_SCK, 0);
    gpio_set_level(PIN_TC_CS, 1);   // começa a próxima conversão
    return v;
}

static void tarefa(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(300));   // primeira conversão depois do CS subir no init
    TickType_t ultimo = xTaskGetTickCount();
    for (;;) {
        uint16_t q = le_quadro();
        if (q == 0xFFFF || (q & 0x8000)) {
            s_temp = NAN, s_estado = "sem módulo";
        } else if (q & 0x0004) {
            s_temp = NAN, s_estado = "termopar aberto";
        } else {
            s_temp = (float)(q >> 3) * 0.25f, s_estado = "ok";
        }
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_MS));
    }
}

void termopar_init(void)
{
    gpio_config_t out = {.pin_bit_mask = (1ULL << PIN_TC_SCK) | (1ULL << PIN_TC_CS), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&out);
    gpio_set_level(PIN_TC_SCK, 0);
    gpio_set_level(PIN_TC_CS, 1);
    gpio_config_t in = {.pin_bit_mask = 1ULL << PIN_TC_SO, .mode = GPIO_MODE_INPUT,
                        .pull_up_en = GPIO_PULLUP_ENABLE};   // sem módulo: 0xFFFF
    gpio_config(&in);
    // núcleo 0, abaixo da comms (10) e da telemetria (4): ~70 µs de bit-bang a cada 250 ms
    xTaskCreatePinnedToCore(tarefa, "termopar", 2560, NULL, 2, NULL, 0);
}

float termopar_temp(void) { return s_temp; }
const char *termopar_estado(void) { return s_estado; }
