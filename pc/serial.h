/* Porta serial mínima (Windows e POSIX): abre 8N1 sem controle de fluxo, escreve, lê uma linha com prazo.
 * Na abertura DTR e RTS ficam desligados: nas placas ESP32 DevKit eles ligam o EN/BOOT pelo circuito de
 * auto-reset, e mexer neles reinicia a placa. */
#ifndef SERIAL_H
#define SERIAL_H

typedef struct serial serial;

/* porta: "COM7" (Windows) ou "/dev/ttyUSB0". NULL se não abrir. */
serial *serial_abre(const char *porta, int baud);
void serial_fecha(serial *s);
/* Descarta o que estiver no buffer de entrada. */
void serial_limpa(serial *s);
/* Escreve n bytes; devolve 0 se escreveu tudo. */
int serial_escreve(serial *s, const char *buf, int n);
/* Lê até '\n' (descarta '\r'), no máximo cap-1 caracteres, esperando até timeout_ms no total.
 * Devolve o tamanho da linha (>= 0) ou -1 no prazo esgotado. */
int serial_le_linha(serial *s, char *linha, int cap, int timeout_ms);

#endif
