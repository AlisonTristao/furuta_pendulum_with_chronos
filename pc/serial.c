#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUF_CAP 512

#ifdef _WIN32
#include <windows.h>

struct serial {
    HANDLE h;
    char buf[BUF_CAP];
    int ini, fim;              /* bytes recebidos ainda não entregues: buf[ini..fim) */
};

static double agora_ms(void)
{
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return 1e3 * (double)c.QuadPart / (double)f.QuadPart;
}

serial *serial_abre(const char *porta, int baud)
{
    char nome[64];
    snprintf(nome, sizeof(nome), "\\\\.\\%s", porta);
    HANDLE h = CreateFileA(nome, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) return NULL;

    DCB dcb;
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;     /* não reinicia a ESP32 */
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return NULL; }
    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, CLRRTS);

    /* ReadFile volta assim que houver qualquer byte, ou depois de 1 ms sem nada */
    COMMTIMEOUTS to = {MAXDWORD, MAXDWORD, 1, 0, 100};
    SetCommTimeouts(h, &to);
    SetupComm(h, 4096, 4096);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    serial *s = (serial *)calloc(1, sizeof(serial));
    s->h = h;
    return s;
}

void serial_fecha(serial *s)
{
    if (!s) return;
    CloseHandle(s->h);
    free(s);
}

void serial_limpa(serial *s)
{
    PurgeComm(s->h, PURGE_RXCLEAR);
    s->ini = s->fim = 0;
}

int serial_escreve(serial *s, const char *buf, int n)
{
    DWORD escritos = 0;
    if (!WriteFile(s->h, buf, (DWORD)n, &escritos, NULL)) return -1;
    return (int)escritos == n ? 0 : -1;
}

/* lê o que houver (espera no máximo ~1 ms); devolve bytes lidos */
static int recebe(serial *s)
{
    if (s->ini > 0) {                          /* compacta */
        memmove(s->buf, s->buf + s->ini, s->fim - s->ini);
        s->fim -= s->ini;
        s->ini = 0;
    }
    if (s->fim >= BUF_CAP) s->ini = s->fim = 0; /* lixo sem '\n': descarta */
    DWORD lidos = 0;
    if (!ReadFile(s->h, s->buf + s->fim, (DWORD)(BUF_CAP - s->fim), &lidos, NULL)) return -1;
    s->fim += (int)lidos;
    return (int)lidos;
}

#else /* POSIX */
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

struct serial {
    int fd;
    char buf[BUF_CAP];
    int ini, fim;
};

static double agora_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return 1e3 * t.tv_sec + 1e-6 * t.tv_nsec;
}

static speed_t velocidade(int baud)
{
    switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
    }
}

serial *serial_abre(const char *porta, int baud)
{
    int fd = open(porta, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return NULL;
    struct termios t;
    tcgetattr(fd, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, velocidade(baud));
    cfsetospeed(&t, velocidade(baud));
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CRTSCTS | HUPCL);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &t);
    int bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &bits);
    tcflush(fd, TCIOFLUSH);
    serial *s = (serial *)calloc(1, sizeof(serial));
    s->fd = fd;
    return s;
}

void serial_fecha(serial *s)
{
    if (!s) return;
    close(s->fd);
    free(s);
}

void serial_limpa(serial *s)
{
    tcflush(s->fd, TCIFLUSH);
    s->ini = s->fim = 0;
}

int serial_escreve(serial *s, const char *buf, int n)
{
    int feito = 0;
    while (feito < n) {
        ssize_t r = write(s->fd, buf + feito, n - feito);
        if (r < 0) return -1;
        feito += (int)r;
    }
    return 0;
}

static int recebe(serial *s)
{
    if (s->ini > 0) {
        memmove(s->buf, s->buf + s->ini, s->fim - s->ini);
        s->fim -= s->ini;
        s->ini = 0;
    }
    if (s->fim >= BUF_CAP) s->ini = s->fim = 0;
    struct pollfd p = {s->fd, POLLIN, 0};
    if (poll(&p, 1, 1) <= 0) return 0;
    ssize_t r = read(s->fd, s->buf + s->fim, BUF_CAP - s->fim);
    if (r < 0) return 0;
    s->fim += (int)r;
    return (int)r;
}
#endif

int serial_le_linha(serial *s, char *linha, int cap, int timeout_ms)
{
    double prazo = agora_ms() + timeout_ms;
    for (;;) {
        for (int i = s->ini; i < s->fim; i++) {
            if (s->buf[i] == '\n') {
                int n = i - s->ini;
                if (n > 0 && s->buf[i - 1] == '\r') n--;
                if (n > cap - 1) n = cap - 1;
                memcpy(linha, s->buf + s->ini, n);
                linha[n] = '\0';
                s->ini = i + 1;
                return n;
            }
        }
        if (agora_ms() >= prazo) return -1;
        if (recebe(s) < 0) return -1;
    }
}
