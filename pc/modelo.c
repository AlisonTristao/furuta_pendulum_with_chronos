#include "modelo.h"

#include <math.h>
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

static double sinc(double th)
{
    if (fabs(th) < 1e-4) {
        double t2 = th * th;
        return 1 - t2 / 6 + t2 * t2 / 120 - t2 * t2 * t2 / 5040;
    }
    return sin(th) / th;
}

static double sat(double v, double l) { return v > l ? l : (v < -l ? -l : v); }

/* Mesma ordem e mesmas expressões do código de referência (qlpv_nn_furuta/python/nmpc/modelo.py, sched):
 * r = Ts·(ρ1, ρ3', ρ4, ρ2, ρ5', ρ6, ρ7, ρ8) na disposição das linhas 3 e 4 de (Ac, Bc). */
void modelo_rho(const modelo *m, const double x[4], double r[8])
{
    double th2 = x[1], dth1 = sat(x[2], m->vmax), dth2 = sat(x[3], m->vmax);
    double s = sin(th2), c = cos(th2), s2 = sin(2 * th2), sc = sinc(th2);
    double a = m->alpha + m->beta * s * s;
    double Det = a * m->delta - m->gamma * m->gamma * c * c;
    double be = m->beta, ga = m->gamma, de = m->delta, et = m->eta, mu1 = m->mu1, mu2 = m->mu2;
    double N1 = -ga * et * c * sc;
    double N2 = -de * be * s2 * dth2 - de * mu1 - ga * be * s * c * c * dth1;
    double N3 = de * ga * s * dth2 + ga * c * mu2;
    double N4 = a * et * sc;
    double N5 = ga * be * c * s2 * dth2 + ga * c * mu1 + a * be * s * c * dth1;
    double N6 = -ga * ga * c * s * dth2 - a * mu2;
    double k = m->Ts / Det;
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
static void mat5_mul(const double *X, const double *Y, double *Z)
{
    double t[25];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) {
            double acc = 0;
            for (int k = 0; k < 5; k++) acc += X[i * 5 + k] * Y[k * 5 + j];
            t[i * 5 + j] = acc;
        }
    memcpy(Z, t, sizeof(t));
}

static void expm5(const double *M, double *E)
{
    double nrm = 0;
    for (int j = 0; j < 5; j++) {
        double c = 0;
        for (int i = 0; i < 5; i++) c += fabs(M[i * 5 + j]);
        if (c > nrm) nrm = c;
    }
    int s = 0;
    while (nrm > 0.125) {
        nrm /= 2;
        s++;
    }
    double A[25], T[25];
    double esc = ldexp(1.0, -s);
    for (int i = 0; i < 25; i++) A[i] = M[i] * esc;
    for (int i = 0; i < 25; i++) E[i] = T[i] = (i % 6 == 0);
    for (int k = 1; k < 16; k++) {
        mat5_mul(T, A, T);
        for (int i = 0; i < 25; i++) {
            T[i] /= k;
            E[i] += T[i];
        }
    }
    for (int n = 0; n < s; n++) mat5_mul(E, E, E);
}

void modelo_zoh(const modelo *m, const double r[8], double A[16], double B[4])
{
    double Ts = m->Ts, M[25] = {0}, E[25];
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
    for (int i = 0; i < 25; i++) M[i] *= Ts;
    expm5(M, E);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) A[j * 4 + i] = E[i * 5 + j];   /* coluna */
        B[i] = E[i * 5 + 4];
    }
}
