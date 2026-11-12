// Teste de bancada dos pinos (módulo `pinos` do terminal, src/main.cpp). Para conferir a fiação da placa nova.
//
//   pinos -entradas  níveis crus dos fins de curso e do nFAULT (aperte as chaves com a mão)
//   pinos -i2c       scan do I2C e modelo do encoder detectado
//   pinos -motor     só em IDLE: liga o driver e anda 1/8 de volta do motor para cada lado em cada micropasso, por
//                    bit-bang direto no STEP/DIR. Se os pinos M0..M2 estiverem certos, o motor anda o MESMO ângulo
//                    em todos os modos. Para na hora se um fim de curso mudar de estado.
// O encoder (θ2) e as chaves já aparecem ao vivo nos tópicos sensores e maquina.
#include "config.h"
#include "controle.h"
#include "embarcado.h"
#include "encoder.h"
#include "pinos.h"
#include "saida.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool ativa(gpio_num_t p) { return gpio_get_level(p) == FIM_NIVEL_ATIVO; }

void pinos_entradas(void)
{
    saida_printf("FIM_NEG(GPIO%d)=%d%s  FIM_POS(GPIO%d)=%d%s  nFAULT(GPIO%d)=%d%s\n", PIN_FIM_NEG,
                 gpio_get_level(PIN_FIM_NEG), ativa(PIN_FIM_NEG) ? " ACIONADO" : "", PIN_FIM_POS,
                 gpio_get_level(PIN_FIM_POS), ativa(PIN_FIM_POS) ? " ACIONADO" : "", PIN_FLT, gpio_get_level(PIN_FLT),
                 gpio_get_level(PIN_FLT) == 0 ? " FALHA/sem pull-up" : " ok");
}

void pinos_i2c(void)
{
    uint8_t end[16];
    int n = encoder_scan(end, 16);
    saida_printf("I2C SDA=GPIO%d SCL=GPIO%d: %d dispositivo(s):", PIN_SDA, PIN_SCL, n);
    for (int i = 0; i < n; i++) saida_printf(" 0x%02X", end[i]);
    saida_printf("\nencoder: %s%s\n", encoder_nome(),
                 encoder_ok() ? "" : " (esperado 0x36 = AS5600 ou 0x06 = MT6701; sem nada: confira fios/alimentação)");
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
    // velocidade em passos inteiros/s: começa em 100 e vai até 400 (≈ 2 voltas/s), escalada pelo micropasso
    const float v0 = 100.0f * m, v1 = 400.0f * m, a = 1500.0f * m;
    float v = v0;
    int dados = 0;
    for (; dados < total; dados++) {
        if (gpio_get_level(PIN_FIM_NEG) != neg0 || gpio_get_level(PIN_FIM_POS) != pos0) {
            saida_printf("  !! fim de curso mudou de estado, parei (FIM_NEG=%d FIM_POS=%d)\n",
                         gpio_get_level(PIN_FIM_NEG), gpio_get_level(PIN_FIM_POS));
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
        if ((dados & 63) == 63) vTaskDelay(1);   // não prende o núcleo 0 (malha de 1 kHz, BTP)
    }
    return dados * dir;
}

const char *pinos_motor(void)
{
    if (embarcado_ativo()) return "tem um ensaio rodando";
    if (controle_estado() != EST_OCIOSO) return "só em IDLE (furuta -driver 0 antes)";
    static const int MODOS[] = {1, 2, 4, 8, 16, 32};
    saida_printf("STEP=GPIO%d DIR=GPIO%d EN=GPIO%d M0=GPIO%d M1=GPIO%d M2=GPIO%d\n", PIN_STEP, PIN_DIR, PIN_EN, PIN_M0,
                 PIN_M1, PIN_M2);
    pinos_entradas();
    if (gpio_get_level(PIN_FLT) == 0 && USA_PINO_FALHA)
        saida_printf("nFAULT em 0 antes de ligar: falha no driver ou falta pull-up\n");
    int dir0 = gpio_get_level(PIN_DIR);          // o gerador de passos guarda o sentido: devolve igual no fim
    gpio_set_level(PIN_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    saida_printf("driver ligado. Cada modo: +1/8 de volta do motor e volta. O ângulo tem que ser o mesmo.\n");
    for (int i = 0; i < (int)(sizeof(MODOS) / sizeof(MODOS[0])); i++) {
        int m = MODOS[i];
        micropasso(m);
        vTaskDelay(pdMS_TO_TICKS(20));
        int ida = anda(MOTOR_PASSOS_VOLTA * m / 8, m);
        vTaskDelay(pdMS_TO_TICKS(300));
        int volta = anda(-ida, m);
        saida_printf("  1/%-2d: %5d passos ida, %5d volta | nFAULT=%d\n", m, ida, volta, gpio_get_level(PIN_FLT));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    micropasso(MICROPASSO);
    gpio_set_level(PIN_DIR, dir0);
    gpio_set_level(PIN_EN, 1);
    saida_printf("driver desligado\n");
    return NULL;
}
