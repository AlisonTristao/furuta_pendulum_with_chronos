#pragma once

void protocolo_init(void);   // UART0 + tarefa de comunicação (núcleo 0)
// Manda uma linha (acrescenta '\n'); pode ser chamada de qualquer tarefa (a UART serializa as linhas).
void protocolo_envia(const char *fmt, ...);
