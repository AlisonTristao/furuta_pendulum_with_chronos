/* Cliente do protocolo serial do firmware do pêndulo (C:\git\furuta_pendulum_with_chronos, firmware/src/protocolo.c).
 * Texto, linhas terminadas em '\n', 921600 baud. O PC é o mestre: cada comando tem uma resposta.
 *
 *   <a>    aceleração do braço [rad/s²]  -> "t1,t2,dt1,dt2"  (ou "ERR,<estado>" fora de PRONTO/RODANDO)
 *   H      homing                         -> "OK,<meio_curso>,<limite>" | "ERR,<motivo>"
 *   X      só lê o estado              -> "t1,t2,dt1,dt2"
 *   O      origem aqui, sem homing        -> "OK,<meio_curso>,<limite>"
 *   Z      pêndulo parado embaixo: t2 = pi -> "OK"
 *   S      estado                         -> "S,<estado>,<erro>,<encoder>,<fim_neg>,<fim_pos>,<flt>"
 *   E0/E1  desliga/liga o driver          -> "OK"
 * Ao reiniciar o firmware manda "BOOT,furuta,<encoder>". Sem comando por 200 ms o firmware freia o braço. */
#ifndef FURUTA_SERIAL_H
#define FURUTA_SERIAL_H

#define FURUTA_BAUD 921600

typedef struct furuta furuta;

enum {
    FURUTA_OK = 0,
    FURUTA_ERR = 1,          /* o firmware respondeu "ERR,..." (texto em furuta_ultima_resposta) */
    FURUTA_PRAZO = -1,       /* sem resposta no prazo */
    FURUTA_FORMATO = -2,     /* resposta fora do formato esperado */
    FURUTA_REINICIOU = -3,   /* chegou "BOOT,...": a ESP32 reiniciou */
};

furuta *furuta_conecta(const char *porta, int baud);
void furuta_desconecta(furuta *f);

/* Manda a aceleração a [rad/s²] e lê o estado x = (θ1, θ2, dθ1, dθ2) que o firmware tinha ao receber o
 * comando. Devolve FURUTA_OK ou um código de erro. */
int furuta_acel(furuta *f, double a, double x[4], int timeout_ms);

/* Só lê o estado (comando X): não mexe no comando de aceleração nem no watchdog. */
int furuta_estado(furuta *f, double x[4], int timeout_ms);
/* Descarta o que estiver no buffer de entrada (ex.: "BOOT" de um reinício). */
void furuta_limpa(furuta *f);

/* Comando de texto qualquer (H, O, Z, S, E0, E1, I); resp recebe a linha de resposta. */
int furuta_comando(furuta *f, const char *cmd, char *resp, int cap, int timeout_ms);

const char *furuta_ultima_resposta(const furuta *f);

/* Controle embarcado: manda um comando sem esperar resposta; lê uma linha qualquer (telemetria "T,...",
 * avisos "M,..."). furuta_linha devolve FURUTA_OK, FURUTA_ERR, FURUTA_REINICIOU ou FURUTA_PRAZO. */
int furuta_envia(furuta *f, const char *cmd);
int furuta_linha(furuta *f, char *linha, int cap, int timeout_ms);

/* "t1,t2,dt1,dt2" -> x; 0 se o formato estiver certo. */
int furuta_le_estado(const char *linha, double x[4]);

#endif
