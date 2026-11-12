#include "modelo.h"

#include <string.h>

modelo modelo_de_fisico(fisico f, double Ts, double vmax)
{
    modelo m;
    m.alpha = f.I1 + f.m2 * f.L1 * f.L1;
    m.beta = f.m2 * f.l2 * f.l2;
    m.gamma = f.m2 * f.L1 * f.l2;
    m.delta = f.I2 + f.m2 * f.l2 * f.l2;
    m.eta = f.m2 * f.g * f.l2;
    m.mu1 = f.mu1;
    m.mu2 = f.mu2;
    m.Ts = Ts;
    m.vmax = vmax;
    m.tc2 = 0;
    return m;
}

void modelo_ode(const modelo *m, const double x[4], double u, double dx[4])
{
    double s = sin(x[1]), c = cos(x[1]), s2 = sin(2 * x[1]);
    double d1 = x[2], d2 = x[3];
    double M11 = m->alpha + m->beta * s * s, M12 = m->gamma * c, M22 = m->delta;
    double N11 = m->beta * d2 * s2, N12 = -m->gamma * d2 * s, N21 = -0.5 * m->beta * d1 * s2;
    double r1 = u - m->mu1 * d1 - (N11 * d1 + N12 * d2);
    double r2 = -m->mu2 * d2 - N21 * d1 + m->eta * s - m->tc2 * tanh(d2 / 0.01);   /* Coulomb suavizado */
    double det = M11 * M22 - M12 * M12;
    dx[0] = d1;
    dx[1] = d2;
    dx[2] = (M22 * r1 - M12 * r2) / det;
    dx[3] = (-M12 * r1 + M11 * r2) / det;
}

double modelo_acel_braco(const modelo *m, const double x[4], double u)
{
    double dx[4];
    modelo_ode(m, x, u, dx);
    return dx[2];
}

void modelo_passo(const modelo *m, double x[4], double u, int nsub)
{
    double h = m->Ts / nsub, k1[4], k2[4], k3[4], k4[4], t[4];
    for (int n = 0; n < nsub; n++) {
        modelo_ode(m, x, u, k1);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k1[i];
        modelo_ode(m, t, u, k2);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k2[i];
        modelo_ode(m, t, u, k3);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h * k3[i];
        modelo_ode(m, t, u, k4);
        for (int i = 0; i < 4; i++) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
}

static void ode_acel(const modelo *m, const double x[4], double a, double dx[4])
{
    double s = sin(x[1]), c = cos(x[1]), d1 = x[2], d2 = x[3];
    double r2 = -m->mu2 * d2 + 0.5 * m->beta * d1 * d1 * sin(2 * x[1]) + m->eta * s;
    if (m->tc2 != 0) r2 -= m->tc2 * tanh(d2 / 0.01);   /* atrito seco: só na planta simulada */
    dx[0] = d1;
    dx[1] = d2;
    dx[2] = a;
    dx[3] = (r2 - m->gamma * c * a) / m->delta;
}

void modelo_passo_acel(const modelo *m, double x[4], double a, double h0, int nsub)
{
    double h = h0 / nsub, k1[4], k2[4], k3[4], k4[4], t[4];
    for (int n = 0; n < nsub; n++) {
        ode_acel(m, x, a, k1);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k1[i];
        ode_acel(m, t, a, k2);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k2[i];
        ode_acel(m, t, a, k3);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h * k3[i];
        ode_acel(m, t, a, k4);
        for (int i = 0; i < 4; i++) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
}

/* Estimador e compensação do atraso no tipo do controlador (real.h): no ESP32 em float (o double emulado com
 * sin/cos levava ~2,7 ms por período). Sem atrito seco (o controlador não modela). No PC, real = double: chama
 * a versão em double (mesmo resultado bit a bit). */
#ifdef REAL_FLOAT
static void ode_acel_r(const modelo *m, const real x[4], real a, real dx[4])
{
    real s = sin(x[1]), c = cos(x[1]), d1 = x[2], d2 = x[3];
    real r2 = -(real)m->mu2 * d2 + (real)0.5 * (real)m->beta * d1 * d1 * sin(2 * x[1]) + (real)m->eta * s;
    dx[0] = d1;
    dx[1] = d2;
    dx[2] = a;
    dx[3] = (r2 - (real)m->gamma * c * a) / (real)m->delta;
}
#endif

void modelo_passo_acel_ctl(const modelo *m, double xd[4], double ad, double h0, int nsub)
{
#ifdef REAL_FLOAT
    real x[4] = {(real)xd[0], (real)xd[1], (real)xd[2], (real)xd[3]}, a = (real)ad;
    real h = (real)h0 / nsub, k1[4], k2[4], k3[4], k4[4], t[4];
    for (int n = 0; n < nsub; n++) {
        ode_acel_r(m, x, a, k1);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k1[i];
        ode_acel_r(m, t, a, k2);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h / 2 * k2[i];
        ode_acel_r(m, t, a, k3);
        for (int i = 0; i < 4; i++) t[i] = x[i] + h * k3[i];
        ode_acel_r(m, t, a, k4);
        for (int i = 0; i < 4; i++) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    for (int i = 0; i < 4; i++) xd[i] = x[i];
#else
    modelo_passo_acel(m, xd, ad, h0, nsub);
#endif
}

/* Daqui para baixo: ρ e ZOH, chamados a cada estágio de cada QP do MPC, no tipo do controlador (real.h). */
static real sinc(real th)
{
    if (fabs(th) < (real)1e-4) {
        real t2 = th * th;
        return 1 - t2 / 6 + t2 * t2 / 120 - t2 * t2 * t2 / 5040;
    }
    return sin(th) / th;
}

static real sat(real v, real l) { return v > l ? l : (v < -l ? -l : v); }

/* Mesma ordem e mesmas expressões do código de referência (qlpv_nn_furuta/python/nmpc/modelo.py, sched):
 * r = Ts·(ρ1, ρ3', ρ4, ρ2, ρ5', ρ6, ρ7, ρ8) na disposição das linhas 3 e 4 de (Ac, Bc). */
void modelo_rho(const modelo *m, const real x[4], real r[8])
{
    real vmax = (real)m->vmax;
    real th2 = x[1], dth1 = sat(x[2], vmax), dth2 = sat(x[3], vmax);
    real s = sin(th2), c = cos(th2), s2 = sin(2 * th2), sc = sinc(th2);
    real al = (real)m->alpha, be = (real)m->beta, ga = (real)m->gamma, de = (real)m->delta, et = (real)m->eta;
    real mu1 = (real)m->mu1, mu2 = (real)m->mu2;
    real a = al + be * s * s;
    real Det = a * de - ga * ga * c * c;
    real N1 = -ga * et * c * sc;
    real N2 = -de * be * s2 * dth2 - de * mu1 - ga * be * s * c * c * dth1;
    real N3 = de * ga * s * dth2 + ga * c * mu2;
    real N4 = a * et * sc;
    real N5 = ga * be * c * s2 * dth2 + ga * c * mu1 + a * be * s * c * dth1;
    real N6 = -ga * ga * c * s * dth2 - a * mu2;
    real k = (real)m->Ts / Det;
    r[0] = k * N1;
    r[1] = k * N2;
    r[2] = k * N3;
    r[3] = k * N4;
    r[4] = k * N5;
    r[5] = k * N6;
    r[6] = k * de;
    r[7] = k * (-ga * c);
}

/* expm de matriz 5×5 (linha) por escalonamento e quadratura + Taylor de ordem 15 (igual ao código de referência). */
static void mat5_mul(const real *X, const real *Y, real *Z)
{
    real t[25];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) {
            real acc = 0;
            for (int k = 0; k < 5; k++) acc += X[i * 5 + k] * Y[k * 5 + j];
            t[i * 5 + j] = acc;
        }
    memcpy(Z, t, sizeof(t));
}

static void expm5(const real *M, real *E)
{
    real nrm = 0;
    for (int j = 0; j < 5; j++) {
        real c = 0;
        for (int i = 0; i < 5; i++) c += fabs(M[i * 5 + j]);
        if (c > nrm) nrm = c;
    }
    int s = 0;
    while (nrm > (real)0.125) {
        nrm /= 2;
        s++;
    }
    real A[25], T[25];
    real esc = ldexp((real)1, -s);
    for (int i = 0; i < 25; i++) A[i] = M[i] * esc;
    for (int i = 0; i < 25; i++) E[i] = T[i] = (i % 6 == 0);
#ifdef REAL_FLOAT
    const int ordem = 7;   /* ‖A‖ <= 1/8: resto ~ 0,125^8/8! ~ 6e-13, muito abaixo do épsilon do float */
#else
    const int ordem = 15;  /* igual ao código de referência */
#endif
    for (int k = 1; k <= ordem; k++) {
        mat5_mul(T, A, T);
#ifdef REAL_FLOAT
        real ik = (real)1 / k;   /* divisão em float é cara no ESP32: 1 por termo em vez de 25 */
        for (int i = 0; i < 25; i++) {
            T[i] *= ik;
            E[i] += T[i];
        }
#else
        for (int i = 0; i < 25; i++) {
            T[i] /= k;
            E[i] += T[i];
        }
#endif
    }
    for (int n = 0; n < s; n++) mat5_mul(E, E, E);
}

void modelo_zoh(const modelo *m, const real r[8], real A[16], real B[4]) { modelo_zoh_h(m, r, (real)m->Ts, A, B); }

void modelo_zoh_h(const modelo *m, const real r[8], real h, real A[16], real B[4])
{
    real Ts = (real)m->Ts, M[25] = {0}, E[25];
    M[0 * 5 + 2] = 1;
    M[1 * 5 + 3] = 1;
    M[2 * 5 + 1] = r[0] / Ts;
    M[2 * 5 + 2] = r[1] / Ts;
    M[2 * 5 + 3] = r[2] / Ts;
    M[3 * 5 + 1] = r[3] / Ts;
    M[3 * 5 + 2] = r[4] / Ts;
    M[3 * 5 + 3] = r[5] / Ts;
    M[2 * 5 + 4] = r[6] / Ts;
    M[3 * 5 + 4] = r[7] / Ts;
    for (int i = 0; i < 25; i++) M[i] *= h;
    expm5(M, E);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) A[j * 4 + i] = E[i * 5 + j];   /* coluna */
        B[i] = E[i * 5 + 4];
    }
}
