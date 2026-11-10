#include "config.h"
#include "encoder.h"

#include <math.h>

#include "driver/i2c_master.h"

#define DOIS_PI 6.28318530718f

typedef struct {
    const char *nome;
    uint8_t endereco;
    uint8_t registro;
    uint32_t resolucao;
} modelo_t;

static const modelo_t MODELOS[] = {
    {"AS5600", 0x36, 0x0C, 4096},     // RAW ANGLE (0x0C..0x0D), 12 bits
    {"MT6701", 0x06, 0x03, 16384},    // ANGLE (0x03..0x04), 14 bits
};

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static const modelo_t *s_modelo = NULL;

static int32_t s_bruto_ant;           // última leitura crua
static int32_t s_contagem;            // contagem desenrolada
static int32_t s_zero;                // contagem que corresponde a s_ang_zero
static float s_ang_zero = 0.0f;
static float s_th_est, s_w_est;       // observador
static bool s_primeira = true;

static bool le_bruto(int32_t *bruto)
{
    uint8_t b[2];
    if (i2c_master_transmit_receive(s_dev, &s_modelo->registro, 1, b, 2, 2) != ESP_OK) return false;
    if (s_modelo->resolucao == 4096)
        *bruto = ((b[0] & 0x0F) << 8) | b[1];
    else
        *bruto = (b[0] << 6) | (b[1] >> 2);
    return true;
}

esp_err_t encoder_init(void)
{
    i2c_master_bus_config_t bus = {
        .i2c_port = -1,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus, &s_bus);
    if (err != ESP_OK) return err;
    for (int i = 0; i < (int)(sizeof(MODELOS) / sizeof(MODELOS[0])); i++) {
        if (i2c_master_probe(s_bus, MODELOS[i].endereco, 20) == ESP_OK) {
            s_modelo = &MODELOS[i];
            break;
        }
    }
    if (!s_modelo) return ESP_ERR_NOT_FOUND;
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_modelo->endereco,
        .scl_speed_hz = ENC_I2C_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dev, &s_dev);
}

const char *encoder_nome(void) { return s_modelo ? s_modelo->nome : "NENHUM"; }
bool encoder_ok(void) { return s_modelo != NULL; }

int encoder_scan(uint8_t *enderecos, int max)
{
    int n = 0;
    if (!s_bus) return 0;
    for (uint8_t a = 0x03; a < 0x78 && n < max; a++)
        if (i2c_master_probe(s_bus, a, 5) == ESP_OK) enderecos[n++] = a;
    return n;
}

static float angulo_medido(void)
{
    return ENC_SINAL * (float)(s_contagem - s_zero) * DOIS_PI / (float)s_modelo->resolucao + s_ang_zero;
}

bool encoder_atualiza(float dt)
{
    if (!s_modelo) return false;
    int32_t bruto;
    if (!le_bruto(&bruto)) return false;
    int32_t res = (int32_t)s_modelo->resolucao;
    if (s_primeira) {
        s_bruto_ant = s_contagem = s_zero = bruto;
        s_th_est = angulo_medido();
        s_w_est = 0.0f;
        s_primeira = false;
        return true;
    }
    int32_t d = bruto - s_bruto_ant;                  // desenrola: passo menor que meia volta por amostra
    if (d > res / 2) d -= res;
    else if (d < -res / 2) d += res;
    s_contagem += d;
    s_bruto_ant = bruto;

    // observador de 2a ordem (rastreador): th_est segue a medida com banda wn, w_est = velocidade filtrada
    const float wn = DOIS_PI * ENC_BANDA_VEL_HZ;
    float e = angulo_medido() - s_th_est;
    s_th_est += dt * (s_w_est + 2.0f * wn * e);       // amortecimento 1
    s_w_est += dt * wn * wn * e;
    return true;
}

void encoder_define(float angulo)
{
    float desloc = angulo - angulo_medido();
    s_zero = s_contagem;
    s_ang_zero = angulo;
    s_th_est += desloc;
}

float encoder_angulo(void) { return s_modelo ? angulo_medido() : 0.0f; }
float encoder_velocidade(void) { return s_w_est; }
