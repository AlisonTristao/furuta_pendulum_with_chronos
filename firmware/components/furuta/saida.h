// Saída de texto do núcleo em C, chamável de qualquer tarefa. Quem a entrega é o src/main.cpp (BTP/TraceView),
// que registra a função no boot; antes disso (ou sem registro) o texto é descartado.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*saida_fn)(int aviso, const char *texto);   // aviso = 1: evento (painel de logs), 0: terminal
void saida_registra(saida_fn fn);

// Texto no terminal do TraceView (dentro de um comando, vira a resposta dele).
void saida_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
// Evento (início/fim de ensaio etc.): painel de logs do TraceView e terminal.
void saida_aviso(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
