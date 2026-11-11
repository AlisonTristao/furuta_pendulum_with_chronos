/* Modelo do pêndulo de Furuta (artigo ICIT 27, seção IV): dinâmica não linear, 8 ρ e qLPV discreto por ZOH.
 * Estado x = (θ1, θ2, dθ1, dθ2), θ2 = 0 no topo; entrada u = torque no braço [N·m]. Matrizes em ordem de coluna. */
#ifndef MODELO_H
#define MODELO_H

/* Parâmetros físicos (SI). */
typedef struct {
    double m2, L1, l2, I1, I2, mu1, mu2, g;
} fisico;

/* Parâmetros agrupados (eq. 9) + Ts e o limite de velocidade usado em f_ρ. */
typedef struct {
    double alpha, beta, gamma, delta, eta, mu1, mu2, Ts, vmax;
    double tc2;   /* atrito seco no pêndulo [N·m] (só na planta simulada; o MPC usa 0) */
} modelo;

modelo modelo_de_fisico(fisico f, double Ts, double vmax);

/* ẋ = f(x, u) (eq. 9). */
void modelo_ode(const modelo *m, const double x[4], double u, double dx[4]);
/* θ̈1 que o torque u produz no estado x: o que o motor de passo impõe ao braço. */
double modelo_acel_braco(const modelo *m, const double x[4], double u);
/* Planta: RK4 com nsub subpassos em Ts, u constante. */
void modelo_passo(const modelo *m, double x[4], double u, int nsub);
/* Planta como no hardware: o motor impõe θ̈1 = a (constante) e o pêndulo responde. RK4 com nsub subpassos em h. */
void modelo_passo_acel(const modelo *m, double x[4], double a, double h, int nsub);

/* ρ = f_ρ(x) (8, eq. 12), na escala de Ts (ρ·Ts), com dθ saturadas em ±vmax. */
void modelo_rho(const modelo *m, const double x[4], double r[8]);
/* (A, B) de um estágio por ZOH (eq. 13): A 4×4 (coluna), B 4. r na escala de Ts. */
void modelo_zoh(const modelo *m, const double r[8], double A[16], double B[4]);

#endif
