// Teste de bancada dos pinos (ambiente `teste_pinos` do platformio.ini). Saída em texto no monitor serial.
//
//  1. scan do I2C (D21/D22) e modelo do encoder detectado
//  2. níveis das entradas: fins de curso (D18/D19) e nFAULT (D34)
//  3. motor: liga o driver (EN = D27) e anda 1/8 de volta do motor para cada lado em cada micropasso
//     (M0..M2 = D32/D33/D13), por bit-bang direto no STEP (D25) e DIR (D26). Se os pinos M estiverem certos, o
//     motor anda o MESMO ângulo em todos os modos. Para na hora se um fim de curso acionar.
//  4. monitor: 5x por segundo imprime encoder, fins de curso e nFAULT (gire o pêndulo e aperte as chaves com a mão)
//
// Comandos no monitor: 'm' repete o teste do motor, 'i' repete o scan do I2C, 'z' zera o ângulo do encoder.
#include "config.h"
#include "encoder.h"

#include <stdarg.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART UART_NUM_0
#define RAD2DEG 57.2957795f

static void out(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart_write_bytes(UART, buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1);
}

static bool fim_neg(void) { return gpio_get_level(PIN_FIM_NEG) == FIM_NIVEL_ATIVO; }
static bool fim_pos(void) { return gpio_get_level(PIN_FIM_POS) == FIM_NIVEL_ATIVO; }

static void le_encoder(int n)
{
    for (int i = 0; i < n; i++) {             // algumas leituras para o observador assentar
        encoder_atualiza(1e-3f);
        esp_rom_delay_us(1000);
    }
}

static void scan_i2c(void)
{
    uint8_t end[16];
    int n = encoder_scan(end, 16);
    out("\r\n[I2C] SDA=D%d SCL=D%d: %d dispositivo(s):", PIN_SDA, PIN_SCL, n);
    for (int i = 0; i < n; i++) out(" 0x%02X", end[i]);
    out("\r\n[I2C] encoder detectado: %s%s\r\n", encoder_nome(),
        encoder_ok() ? "" : "  (esperado 0x36 = AS5600 ou 0x06 = MT6701; sem nada: confira fios/alimentação)");
}

static void entradas(void)
{
    out("[ENTRADAS] FIM_NEG(D%d)=%d%s  FIM_POS(D%d)=%d%s  nFAULT(D%d)=%d%s\r\n",
        PIN_FIM_NEG, gpio_get_level(PIN_FIM_NEG), fim_neg() ? " ACIONADO" : "",
        PIN_FIM_POS, gpio_get_level(PIN_FIM_POS), fim_pos() ? " ACIONADO" : "",
        PIN_FLT, gpio_get_level(PIN_FLT), gpio_get_level(PIN_FLT) == 0 ? " FALHA/sem pull-up" : " ok");
}

static void micropasso(int m)
{
    int k = 0;
    for (int v = m; v > 1; v >>= 1) k++;
    gpio_set_level(PIN_M0, k & 1);
    gpio_set_level(PIN_M1, (k >> 1) & 1);
    gpio_set_level(PIN_M2, (k >> 2) & 1);
}

// Anda `passos` (com sinal) com rampa linear de velocidade. Devolve os passos dados (para antes numa chave).
static int anda(int passos, int m)
{
    int dir = passos >= 0 ? 1 : -1;
    int neg0 = gpio_get_level(PIN_FIM_NEG), pos0 = gpio_get_level(PIN_FIM_POS);   // para se alguma MUDAR (NA ou NF)
    int total = passos * dir;
    gpio_set_level(PIN_DIR, dir * DIR_SINAL > 0);
    esp_rom_delay_us(5);
    // velocidade em passos inteiros/s: começa em 100 e vai até 400 (≈ 2 volta/s), escalada pelo micropasso
    const float v0 = 100.0f * m, v1 = 400.0f * m, a = 1500.0f * m;
    float v = v0;
    int dados = 0;
    for (; dados < total; dados++) {
        if (gpio_get_level(PIN_FIM_NEG) != neg0 || gpio_get_level(PIN_FIM_POS) != pos0) {
            out("  !! fim de curso mudou de estado, parei (FIM_NEG=%d FIM_POS=%d)\r\n", gpio_get_level(PIN_FIM_NEG),
                gpio_get_level(PIN_FIM_POS));
            break;
        }
        int resta = total - dados;
        float v_freio = v0 + (v1 - v0) * (float)resta / (float)(total / 2 + 1);
        float alvo = v1 < v_freio ? v1 : v_freio;
        v = v < alvo ? v + a / v : alvo;
        int periodo = (int)(1e6f / v);
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(3);
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(periodo > 3 ? periodo - 3 : 1);
    }
    return dados * dir;
}

static void teste_motor(void)
{
    static const int MODOS[] = {1, 2, 4, 8, 16, 32};
    out("\r\n[MOTOR] STEP=D%d DIR=D%d EN=D%d M0=D%d M1=D%d M2=D%d\r\n", PIN_STEP, PIN_DIR, PIN_EN, PIN_M0, PIN_M1,
        PIN_M2);
    entradas();
    if (gpio_get_level(PIN_FLT) == 0 && USA_PINO_FALHA)
        out("[MOTOR] nFAULT em 0 antes de ligar: falha no driver ou falta pull-up no D34\r\n");
    gpio_set_level(PIN_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    out("[MOTOR] driver ligado. Cada modo: +1/8 de volta do motor e volta. O angulo tem que ser o mesmo.\r\n");
    for (int i = 0; i < (int)(sizeof(MODOS) / sizeof(MODOS[0])); i++) {
        int m = MODOS[i];
        micropasso(m);
        vTaskDelay(pdMS_TO_TICKS(20));
        int n = MOTOR_PASSOS_VOLTA * m / 8;
        le_encoder(30);
        float e0 = encoder_angulo();
        int ida = anda(n, m);
        le_encoder(150);
        float e1 = encoder_angulo();
        vTaskDelay(pdMS_TO_TICKS(300));
        int volta = anda(-ida, m);
        le_encoder(150);
        float e2 = encoder_angulo();
        out("  1/%-2d: %5d passos ida, %5d volta | encoder %+7.2f -> %+7.2f -> %+7.2f graus | nFAULT=%d\r\n", m,
            ida, volta, e0 * RAD2DEG, e1 * RAD2DEG, e2 * RAD2DEG, gpio_get_level(PIN_FLT));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    micropasso(MICROPASSO);
    gpio_set_level(PIN_EN, 1);
    out("[MOTOR] driver desligado (o encoder mede o pendulo em relacao ao braco: ele so muda se o pendulo balancar)\r\n");
}

void teste_pinos(void)
{
    uart_config_t cfg = {
        .baud_rate = SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART, 1024, 2048, 0, NULL, 0);
    uart_param_config(UART, &cfg);

    gpio_config_t o = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_EN) | (1ULL << PIN_M0) |
                        (1ULL << PIN_M1) | (1ULL << PIN_M2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&o);
    gpio_set_level(PIN_EN, 1);
    gpio_set_level(PIN_STEP, 0);
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_FIM_NEG) | (1ULL << PIN_FIM_POS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = FIM_PULL_UP,
        .pull_down_en = !FIM_PULL_UP,
    };
    gpio_config(&in);
    gpio_config_t flt = {.pin_bit_mask = 1ULL << PIN_FLT, .mode = GPIO_MODE_INPUT};
    gpio_config(&flt);

    vTaskDelay(pdMS_TO_TICKS(500));
    out("\r\n===== TESTE DE PINOS - pendulo de Furuta =====\r\n");
    encoder_init();
    scan_i2c();
    entradas();
    vTaskDelay(pdMS_TO_TICKS(1000));
    teste_motor();
    out("\r\n[MONITOR] gire o pendulo e aperte as chaves. 'm' motor, 'i' I2C, 'z' zera encoder\r\n");

    int erros = 0;
    for (int k = 0;; k++) {
        if (encoder_ok() && !encoder_atualiza(1e-3f)) erros++;
        vTaskDelay(1);
        if (k % 200 == 0) {
            out("enc %+9.2f graus  %+8.1f graus/s  erros_i2c=%d | FIM_NEG=%d FIM_POS=%d nFAULT=%d\r\n",
                encoder_angulo() * RAD2DEG, encoder_velocidade() * RAD2DEG, erros, fim_neg(), fim_pos(),
                gpio_get_level(PIN_FLT));
        }
        uint8_t c;
        if (uart_read_bytes(UART, &c, 1, 0) == 1) {
            if (c == 'm') teste_motor();
            else if (c == 'i') scan_i2c();
            else if (c == 'z') encoder_define(0.0f);
        }
    }
}
