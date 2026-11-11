// Protocolo serial (UART0, SERIAL_BAUD, linhas terminadas em '\n'). Ver README.md.
//
//   <a>   aceleração do braço [rad/s²]   -> "t1,t2,dt1,dt2"   (ou "ERR,<estado>" fora de READY/RUN)
//   H     homing                          -> "OK,<meio_curso>,<limite>" | "ERR,<motivo>"
//   X     só lê o estado                  -> "t1,t2,dt1,dt2"   (amostra no início do período)
//   O     origem aqui, sem homing         -> "OK,<meio_curso>,<limite>"
//   Z     pêndulo parado embaixo: t2 = pi -> "OK"
//   S     estado                          -> "S,<estado>,<erro>,<encoder>,<fim_neg>,<fim_pos>,<flt>"
//   I     scan I2C                        -> "I,<0xAA>,..."
//   E0/E1 desliga/liga o driver           -> "OK"
#include "config.h"
#include "controle.h"
#include "encoder.h"
#include "protocolo.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART UART_NUM_0

static void envia(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 2) n = sizeof(buf) - 2;
    buf[n++] = '\n';
    uart_write_bytes(UART, buf, n);
}

static void responde_curso(void)
{
    envia("OK,%.5f,%.5f", controle_meio_curso(), controle_limite());
}

static void executa(char *linha)
{
    char c = linha[0];
    if (c == '\0') return;

    if (isdigit((unsigned char)c) || c == '-' || c == '+' || c == '.') {
        char *fim;
        float a = strtof(linha, &fim);
        if (fim == linha) {
            envia("ERR,NUMERO");
        } else if (controle_acel(a)) {
            leitura_t x;
            controle_leitura(&x);
            envia("%.5f,%.5f,%.4f,%.4f", x.t1, x.t2, x.dt1, x.dt2);
        } else {
            envia("ERR,%s", controle_estado_nome(controle_estado()));
        }
        return;
    }

    switch (toupper((unsigned char)c)) {
    case 'H':
        if (!controle_homing()) {
            envia("ERR,RODANDO");
            break;
        }
        while (controle_estado() == EST_HOMING) vTaskDelay(pdMS_TO_TICKS(10));
        if (controle_estado() == EST_PRONTO) responde_curso();
        else envia("ERR,%s", controle_erro());
        break;
    case 'O':
        controle_origem_aqui();
        if (controle_estado() == EST_PRONTO) responde_curso();
        else envia("ERR,%s", controle_estado_nome(controle_estado()));
        break;
    case 'X': {                              // só lê o estado (não mexe no comando nem no watchdog)
        leitura_t x;
        controle_leitura(&x);
        envia("%.5f,%.5f,%.4f,%.4f", x.t1, x.t2, x.dt1, x.dt2);
        break;
    }
    case 'Z':
        controle_zera_pendulo();
        envia("OK");
        break;
    case 'S':
        envia("S,%s,%s,%s,%d,%d,%d", controle_estado_nome(controle_estado()), controle_erro(), encoder_nome(),
              controle_fim_neg(), controle_fim_pos(), controle_falha_driver());
        break;
    case 'I': {
        uint8_t end[16];
        int n = encoder_scan(end, 16);
        char buf[100] = "I";
        for (int i = 0; i < n; i++) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), ",0x%02X", end[i]);
        envia("%s", buf);
        break;
    }
    case 'E':
        if (linha[1] == '0' || linha[1] == '1') {
            controle_habilita(linha[1] == '1');
            envia("OK");
        } else {
            envia("ERR,CMD");
        }
        break;
    default:
        envia("ERR,CMD");
    }
}

static void tarefa(void *arg)
{
    char linha[SERIAL_LINHA_MAX];
    int n = 0;
    uint8_t buf[64];
    envia("BOOT,furuta,%s", encoder_nome());
    for (;;) {
        int lidos = uart_read_bytes(UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < lidos; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                linha[n] = '\0';
                if (n > 0 && linha[n - 1] == '\r') linha[n - 1] = '\0';
                executa(linha);
                n = 0;
            } else if (n < SERIAL_LINHA_MAX - 1) {
                linha[n++] = c;
            } else {
                n = 0;                       // linha longa demais: descarta
            }
        }
    }
}

void protocolo_init(void)
{
    uart_config_t cfg = {
        .baud_rate = SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART, 1024, 1024, 0, NULL, 0);
    uart_param_config(UART, &cfg);
    uart_set_pin(UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreatePinnedToCore(tarefa, "protocolo", 4096, NULL, 5, NULL, 0);
}
