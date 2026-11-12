#include "saida.h"

#include <stdarg.h>
#include <stdio.h>

static saida_fn volatile s_fn = NULL;

void saida_registra(saida_fn fn) { s_fn = fn; }

static void envia(int aviso, const char *fmt, va_list ap)
{
    saida_fn fn = s_fn;
    if (!fn) return;
    char buf[512];
    if (vsnprintf(buf, sizeof(buf), fmt, ap) < 0) return;
    fn(aviso, buf);
}

void saida_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    envia(0, fmt, ap);
    va_end(ap);
}

void saida_aviso(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    envia(1, fmt, ap);
    va_end(ap);
}
