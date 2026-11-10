// Encoder magnético absoluto do pêndulo (I2C): AS5600 (0x36, 12 bits) ou MT6701 (0x06, 14 bits),
// detectado pelo endereço no boot. Ângulo desenrolado (multivolta) e velocidade por observador.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t encoder_init(void);           // ESP_ERR_NOT_FOUND se nenhum encoder conhecido responder
const char *encoder_nome(void);          // "AS5600", "MT6701" ou "NENHUM"
bool encoder_ok(void);

// Lista os endereços que respondem no barramento (diagnóstico). Devolve quantos achou.
int encoder_scan(uint8_t *enderecos, int max);

// Uma leitura + atualização do observador (chamar a CTRL_HZ). Devolve false se a leitura I2C falhar.
bool encoder_atualiza(float dt);

// Define o zero: o ângulo atual passa a valer `angulo` (pêndulo parado embaixo -> pi).
void encoder_define(float angulo);

float encoder_angulo(void);              // rad, contínuo
float encoder_velocidade(void);          // rad/s
