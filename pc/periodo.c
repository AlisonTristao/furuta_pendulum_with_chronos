#include "periodo.h"

#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

struct periodo {
    double T, prox;
    HANDLE timer;
};

double periodo_agora(void)
{
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}

periodo *periodo_cria(double T_s)
{
    periodo *p = (periodo *)calloc(1, sizeof(periodo));
    p->T = T_s;
    p->timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    if (!p->timer) p->timer = CreateWaitableTimerExW(NULL, NULL, 0, TIMER_ALL_ACCESS);   /* Windows antigo */
    timeBeginPeriod(1);
    p->prox = periodo_agora() + T_s;
    return p;
}

void periodo_libera(periodo *p)
{
    if (!p) return;
    timeEndPeriod(1);
    CloseHandle(p->timer);
    free(p);
}

double periodo_espera(periodo *p)
{
    double falta = p->prox - periodo_agora();
    if (falta > 1e-3) {
        LARGE_INTEGER t;
        t.QuadPart = -(LONGLONG)((falta - 5e-4) * 1e7);          /* relativo, em 100 ns */
        SetWaitableTimer(p->timer, &t, 0, NULL, NULL, FALSE);
        WaitForSingleObject(p->timer, INFINITE);
    }
    double t;
    while ((t = periodo_agora()) < p->prox) { /* espera ativa no último trecho */ }
    double atraso = t - p->prox;
    p->prox += p->T;
    if (t > p->prox) p->prox += p->T * (double)(long)((t - p->prox) / p->T + 1.0);   /* pula os perdidos */
    return atraso;
}

int periodo_prioridade_alta(void)
{
    int ok = SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS) &&
             SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    return ok ? 0 : -1;
}

#else /* POSIX */
#include <sched.h>
#include <time.h>

struct periodo {
    double T, prox;
};

double periodo_agora(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + 1e-9 * t.tv_nsec;
}

periodo *periodo_cria(double T_s)
{
    periodo *p = (periodo *)calloc(1, sizeof(periodo));
    p->T = T_s;
    p->prox = periodo_agora() + T_s;
    return p;
}

void periodo_libera(periodo *p) { free(p); }

double periodo_espera(periodo *p)
{
    struct timespec ts;
    ts.tv_sec = (time_t)p->prox;
    ts.tv_nsec = (long)((p->prox - (double)ts.tv_sec) * 1e9);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    double t = periodo_agora(), atraso = t - p->prox;
    p->prox += p->T;
    if (t > p->prox) p->prox += p->T * (double)(long)((t - p->prox) / p->T + 1.0);
    return atraso > 0 ? atraso : 0.0;
}

int periodo_prioridade_alta(void)
{
    struct sched_param sp = {sched_get_priority_max(SCHED_FIFO)};
    return sched_setscheduler(0, SCHED_FIFO, &sp);
}
#endif
