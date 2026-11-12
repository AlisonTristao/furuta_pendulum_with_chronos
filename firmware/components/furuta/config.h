// Pinos e parâmetros do firmware. Tudo que depende do hardware fica aqui.
#pragma once

#include "driver/gpio.h"

// ---------------------------------------------------------------- pinos (ESP32-S3)
// PROVISÓRIOS: a placa nova ainda não foi montada; trocar pelos pinos definitivos. Evitar no S3: 19/20 (USB
// nativa, é o link BTP), 0/3/45/46 (strapping), 26-32 (flash) e 33-37 (PSRAM octal dos módulos R8).
// No ESP32 clássico eram: STEP 25, DIR 26, EN 27, FLT 34, M0 32, M1 33, M2 13, FIM 18/19, SDA 21, SCL 22.
#define PIN_STEP     GPIO_NUM_4
#define PIN_DIR      GPIO_NUM_5
#define PIN_EN       GPIO_NUM_6    // DRV8825 nENABLE: 0 = driver ligado
#define PIN_FLT      GPIO_NUM_7    // DRV8825 nFAULT: 0 = falha (SEM pull-up interno)
#define PIN_M0       GPIO_NUM_15
#define PIN_M1       GPIO_NUM_16
#define PIN_M2       GPIO_NUM_17
#define PIN_FIM_NEG  GPIO_NUM_10   // fim de curso do lado theta1 < 0
#define PIN_FIM_POS  GPIO_NUM_11   // fim de curso do lado theta1 > 0
#define PIN_SDA      GPIO_NUM_8
#define PIN_SCL      GPIO_NUM_9
// SLP = RST = 3,3 V (sempre acordado).

// ---------------------------------------------------------------- motor
#define MOTOR_PASSOS_VOLTA 200      // passos inteiros por volta
#define MICROPASSO         16       // 1, 2, 4, 8, 16 ou 32 (pinos M0..M2)
#define REDUCAO            1.0f     // voltas do motor por volta do braço
#define DIR_SINAL          1        // -1 inverte o sentido positivo de theta1
#define PASSO_ISR_HZ       40000    // taxa do gerador de passos (máx. PASSO_ISR_HZ/2 passos/s)
#define V_MAX              25.0f    // rad/s, saturação da velocidade do braço
#define A_MAX              300.0f   // rad/s², saturação do comando e frenagem nos limites

// ---------------------------------------------------------------- fins de curso
#define FIM_NIVEL_ATIVO    1        // solta = 0, apertada = 1 (fio solto lê 1 = acionado: seguro)
#define FIM_PULL_UP        1        // pull-up interno nos dois pinos
#define FIM_DEBOUNCE_MS    3
#define FIM_MARGEM_RAD     0.15f    // limite de software = meio curso - margem
#define HOMING_V           1.5f     // rad/s
#define HOMING_A           10.0f    // rad/s²
#define HOMING_CURSO_MAX   7.0f     // rad: desiste se não achar a chave nesse curso
#define LIM_SEM_HOMING_RAD 0.5f     // limite de software quando a origem é definida sem homing (furuta -origem)

// ---------------------------------------------------------------- encoder
#define ENC_SINAL          1        // -1 inverte o sentido de theta2
#define ENC_I2C_HZ         100000   // 400 kHz dá timeout só com os pull-ups internos
#define ENC_BANDA_VEL_HZ   40.0f    // banda do observador de velocidade de theta2
#define ENC_ERROS_MAX      20       // leituras I2C seguidas com erro até entrar em falha

// ---------------------------------------------------------------- malha interna
#define CTRL_HZ            1000
#define WATCHDOG_MS        200      // sem comando por esse tempo: freia até parar
#define DESLIGA_OCIOSO_MS  10000    // parado em READY por esse tempo: desliga o driver e volta a IDLE
#define USA_PINO_FALHA     1        // 0 se o nFAULT não tiver pull-up externo
