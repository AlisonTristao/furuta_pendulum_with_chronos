/* Identificação do pêndulo pela oscilação livre, com o braço travado pelo motor.
 *
 *   ident.exe <config.ini> [saida.csv]
 *   ident.exe <config.ini> --analisa gravado.csv      (só refaz a análise de uma gravação)
 *
 * Com o braço parado, o pêndulo pendurado é um oscilador:  J φ̈ + μ2 φ̇ + m2 g l2 φ = 0,  J = I2 + m2 l2²,
 * φ = θ2 − π. Dividindo por m2, a dinâmica só depende de l2 (medido no equilíbrio sobre uma borda), J/m2 e μ2/m2:
 *   ωn² = g l2 / (J/m2),   2 ζ ωn = (μ2/m2) / (J/m2).
 * O programa mede o período (ωd, corrigido para a amplitude) e o decaimento dos picos (σ = ζ ωn), e devolve I2 e μ2
 * para o m2 do .ini (a massa só escala I2 e μ2 juntos).
 *
 * Passos: Z com o pêndulo parado embaixo -> você levanta e segura o pêndulo -> O (trava o braço, sem homing) -> você
 * solta -> grava 20 s a partir da soltura mandando aceleração 0 (o braço fica parado) -> E0. */
#include "config.h"
#include "furuta_serial.h"
#include "periodo.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

#define PI 3.14159265358979323846
#define NMAX 20000

static void espera_enter(const char *msg)
{
    printf("%s", msg);
    fflush(stdout);
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {
    }
}

/* mínimos quadrados y = a + b t; devolve R² */
static double reta(const double *t, const double *y, int n, double *a, double *b)
{
    double st = 0, sy = 0, stt = 0, sty = 0, syy = 0;
    for (int i = 0; i < n; i++) st += t[i], sy += y[i], stt += t[i] * t[i], sty += t[i] * y[i], syy += y[i] * y[i];
    double d = n * stt - st * st;
    *b = (n * sty - st * sy) / d;
    *a = (sy - *b * st) / n;
    double ssr = 0, my = sy / n, sst = syy - n * my * my;
    for (int i = 0; i < n; i++) {
        double r = y[i] - (*a + *b * t[i]);
        ssr += r * r;
    }
    return sst > 0 ? 1 - ssr / sst : 0;
}

static int analisa(const double *t, const double *phi, int n, double g, double l2, double m2);

/* lê "t,phi" de um CSV gravado antes */
static int le_csv(const char *arq, double *t, double *phi)
{
    FILE *fp = fopen(arq, "r");
    if (!fp) return -1;
    char linha[128];
    int n = 0;
    if (!fgets(linha, sizeof(linha), fp)) n = -1;
    while (n >= 0 && n < NMAX && fgets(linha, sizeof(linha), fp))
        if (sscanf(linha, "%lf,%lf", &t[n], &phi[n]) == 2) n++;
    fclose(fp);
    return n;
}

int main(int argc, char **argv)
{
#ifdef _WIN32
    SetConsoleOutputCP(65001);   /* acentos no console */
#endif
    if (argc < 2) {
        fprintf(stderr, "uso: %s <config.ini> [saida.csv]\n", argv[0]);
        return 2;
    }
    config cfg;
    if (config_le(argv[1], &cfg)) return 1;
    const double dur = 20.0, espera_max = 30.0, Tam = 0.01, g = cfg.f.g, l2 = cfg.f.l2, m2 = cfg.f.m2;
    static double t[NMAX], phi[NMAX];
    if (argc > 3 && !strcmp(argv[2], "--analisa")) {        /* só reanalisa um CSV gravado */
        int n = le_csv(argv[3], t, phi);
        if (n <= 0) {
            fprintf(stderr, "não leu %s\n", argv[3]);
            return 1;
        }
        printf("l2 = %.4f m, m2 = %.4f kg (do %s)\n", l2, m2, argv[1]);
        return analisa(t, phi, n, g, l2, m2);
    }

    furuta *f = furuta_conecta(cfg.e.porta, FURUTA_BAUD);
    if (!f) {
        fprintf(stderr, "não abriu %s\n", cfg.e.porta);
        return 1;
    }
    char r[128];
    furuta_limpa(f);
    if (furuta_comando(f, "S", r, sizeof(r), 300) != FURUTA_OK) {
        fprintf(stderr, "o ESP32 não respondeu (%s)\n", r);
        return 1;
    }
    printf("ESP32: %s\nl2 = %.4f m, m2 = %.4f kg (do %s)\n", r, l2, m2, argv[1]);
    espera_enter("\n1) Deixe o pêndulo PARADO embaixo e aperte Enter (zera o encoder)... ");
    furuta_comando(f, "Z", r, sizeof(r), 300);
    espera_enter("2) Levante o pêndulo uns 20-30 graus, SEGURE parado e aperte Enter (trava o braço)... ");
    if (furuta_comando(f, "O", r, sizeof(r), 300) != FURUTA_OK) {
        fprintf(stderr, "O falhou: %s\n", r);
        return 1;
    }
    printf("3) Braço travado. SOLTE o pêndulo quando quiser (até %.0f s) e não encoste mais.\n", espera_max);

    int n = 0;
    double t_solto = -1;
    periodo *p = periodo_cria(Tam);
    double t0 = periodo_agora();
    while (n < NMAX) {
        periodo_espera(p);
        double x[4], tt = periodo_agora() - t0;
        if (furuta_acel(f, 0.0, x, 20) != FURUTA_OK) {      /* aceleração 0: braço parado, watchdog alimentado */
            fprintf(stderr, "comunicação falhou em t = %.2f s: %s\n", tt, furuta_ultima_resposta(f));
            break;
        }
        t[n] = tt;
        phi[n] = x[1] - PI;
        n++;
        if (t_solto < 0 && fabs(phi[n - 1]) > 0.1 && fabs(x[3]) > 1.0) {   /* saiu da mão: ganhou velocidade */
            t_solto = tt;
            printf("solto em t = %.1f s, gravando %.0f s...\n", tt, dur);
        }
        if (t_solto < 0 && tt > espera_max) {
            fprintf(stderr, "não detectou a soltura em %.0f s\n", espera_max);
            break;
        }
        if (t_solto >= 0 && tt > t_solto + dur) break;
    }
    furuta_comando(f, "E0", r, sizeof(r), 300);
    furuta_desconecta(f);
    periodo_libera(p);
    printf("%d amostras, driver desligado\n", n);

    if (argc > 2) {
        FILE *o = fopen(argv[2], "w");
        fprintf(o, "t,phi\n");
        for (int i = 0; i < n; i++) fprintf(o, "%.4f,%.6f\n", t[i], phi[i]);
        fclose(o);
    }
    return analisa(t, phi, n, g, l2, m2);
}

static int analisa(const double *t, const double *phi, int n, double g, double l2, double m2)
{
    /* soltura = maior |φ| depois que passou de 0,1 rad (antes disso o pêndulo está na mão) */
    int i0 = -1;
    for (int i = 0; i < n; i++)
        if (fabs(phi[i]) > 0.1) {
            i0 = i;
            break;
        }
    if (i0 < 0) {
        fprintf(stderr, "o pêndulo não passou de 0,1 rad: nada para identificar\n");
        return 1;
    }
    int ipico = i0;
    for (int i = i0; i < n && t[i] < t[i0] + 3; i++)
        if (fabs(phi[i]) > fabs(phi[ipico])) ipico = i;
    /* a partir do pico de soltura: cruzamentos por zero (interpolados) e picos entre eles, até a amplitude cair
     * a 0,02 rad (~13 vezes a resolução do encoder) */
    static double tz[NMAX], tp[NMAX], ap[NMAX];
    int nz = 0, np = 0;
    double amax = 0;
    for (int i = ipico + 1; i < n; i++) {
        amax = fmax(amax, fabs(phi[i]));
        if ((phi[i - 1] < 0) != (phi[i] < 0)) {
            if (amax < 0.02) break;
            tz[nz++] = t[i - 1] + (t[i] - t[i - 1]) * phi[i - 1] / (phi[i - 1] - phi[i]);
            if (nz >= 2) {                                  /* pico entre os dois últimos cruzamentos */
                double tm = 0, am = 0;
                for (int j = i - 1; j > 0 && t[j] >= tz[nz - 2]; j--)
                    if (fabs(phi[j]) > am) am = fabs(phi[j]), tm = t[j];
                tp[np] = tm, ap[np++] = am;
            }
            amax = 0;
        }
    }
    if (np < 4) {
        fprintf(stderr, "poucos ciclos (%d meios-ciclos com amplitude > 0,02 rad)\n", np);
        return 1;
    }
    /* período: meio-ciclos corrigidos para a amplitude (T(a) ≈ T0 (1 + a²/16)) */
    double soma = 0;
    for (int k = 0; k < np; k++) soma += (tz[k + 1] - tz[k]) / (1 + ap[k] * ap[k] / 16);
    double Td = 2 * soma / np, wd = 2 * PI / Td;
    /* decaimento: ln(a) linear em t (viscoso) e a linear em t (Coulomb), para comparar */
    double la[NMAX], a, b, b_c, a_c;
    for (int k = 0; k < np; k++) la[k] = log(ap[k]);
    double r2v = reta(tp, la, np, &a, &b), r2c = reta(tp, ap, np, &a_c, &b_c);
    double sigma = -b, wn = sqrt(wd * wd + sigma * sigma), zeta = sigma / wn;
    double Jm = g * l2 / (wn * wn), I2m = Jm - l2 * l2, mu2m = 2 * sigma * Jm;

    printf("\n%d meios-ciclos, amplitude %.3f -> %.3f rad em %.1f s\n", np, ap[0], ap[np - 1], tp[np - 1] - tp[0]);
    printf("período (pequenas oscilações) %.4f s  ->  wn = %.3f rad/s, zeta = %.4f\n", Td, wn, zeta);
    printf("decaimento: exponencial (atrito viscoso) R² = %.4f | linear (atrito seco) R² = %.4f%s\n", r2v, r2c,
           r2c > r2v ? "  <- atrito seco domina: mu2 viscoso é só aproximação" : "");
    printf("J/m2 = %.4e m², I2/m2 = %.4e m², mu2/m2 = %.4e m²/s\n", Jm, I2m, mu2m);
    if (I2m <= 0) printf("ATENÇÃO: I2 < 0 -> l2 = %.4f inconsistente com o período (confira a medida de l2)\n", l2);
    printf("\nPara o furuta.ini (no lugar de [pendulo]; m2 = %.4f kg só escala I2 e mu2):\n", m2);
    printf("[fisico]\nm2  = %.5f\nl2  = %.5f\nI2  = %.4e\nmu2 = %.4e\n", m2, l2, I2m * m2, mu2m * m2);
    return 0;
}
