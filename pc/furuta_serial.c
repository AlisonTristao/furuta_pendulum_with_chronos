#include "furuta_serial.h"

#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct furuta {
    serial *s;
    char resp[128];
};

furuta *furuta_conecta(const char *porta, int baud)
{
    serial *s = serial_abre(porta, baud);
    if (!s) return NULL;
    furuta *f = (furuta *)calloc(1, sizeof(furuta));
    f->s = s;
    return f;
}

void furuta_desconecta(furuta *f)
{
    if (!f) return;
    serial_fecha(f->s);
    free(f);
}

const char *furuta_ultima_resposta(const furuta *f) { return f->resp; }

int furuta_le_estado(const char *linha, double x[4])
{
    char *p = (char *)linha, *fim;
    for (int i = 0; i < 4; i++) {
        x[i] = strtod(p, &fim);
        if (fim == p) return -1;
        p = fim;
        if (i < 3) {
            if (*p != ',') return -1;
            p++;
        }
    }
    return *p == '\0' ? 0 : -1;
}

static int classifica(const char *r)
{
    if (strncmp(r, "BOOT", 4) == 0) return FURUTA_REINICIOU;
    if (strncmp(r, "ERR", 3) == 0) return FURUTA_ERR;
    return FURUTA_OK;
}

int furuta_comando(furuta *f, const char *cmd, char *resp, int cap, int timeout_ms)
{
    char linha[64];
    int n = snprintf(linha, sizeof(linha), "%s\n", cmd);
    if (serial_escreve(f->s, linha, n)) return FURUTA_PRAZO;
    if (serial_le_linha(f->s, f->resp, sizeof(f->resp), timeout_ms) < 0) {
        f->resp[0] = '\0';
        return FURUTA_PRAZO;
    }
    if (resp && cap > 0) {
        strncpy(resp, f->resp, cap - 1);
        resp[cap - 1] = '\0';
    }
    return classifica(f->resp);
}

int furuta_acel(furuta *f, double a, double x[4], int timeout_ms)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "%.5f", a);
    int r = furuta_comando(f, cmd, NULL, 0, timeout_ms);
    if (r != FURUTA_OK) return r;
    return furuta_le_estado(f->resp, x) == 0 ? FURUTA_OK : FURUTA_FORMATO;
}

int furuta_estado(furuta *f, double x[4], int timeout_ms)
{
    int r = furuta_comando(f, "X", NULL, 0, timeout_ms);
    if (r != FURUTA_OK) return r;
    return furuta_le_estado(f->resp, x) == 0 ? FURUTA_OK : FURUTA_FORMATO;
}

void furuta_limpa(furuta *f) { serial_limpa(f->s); }
