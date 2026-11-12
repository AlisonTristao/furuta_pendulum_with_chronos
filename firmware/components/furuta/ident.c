/* Identificação do pêndulo pela oscilação livre, com o braço travado pelo motor (furuta -ident <l2>; era o
 * pc/ident.exe).
 *
 * Com o braço parado, o pêndulo pendurado é um oscilador:  J φ̈ + μ2 φ̇ + m2 g l2 φ = 0,  J = I2 + m2 l2²,
 * φ = θ2 − π. Dividindo por m2, a dinâmica só depende de l2 (medido no equilíbrio sobre uma borda), J/m2 e μ2/m2:
 *   ωn² = g l2 / (J/m2),   2 ζ ωn = (μ2/m2) / (J/m2).
 * Mede o período (ωd, corrigido para a amplitude) e o decaimento dos picos (σ = ζ ωn), e devolve I2 e μ2 para o m2
 * da configuração (a massa só escala I2 e μ2 juntos).
 *
 * Passos: furuta -zera com o pêndulo parado embaixo -> levante e segure o pêndulo -> furuta -ident 0.085 (trava o
 * braço, sem homing) -> solte -> grava 20 s a partir da soltura (braço parado) -> driver desligado. */
#include "controle.h"
#include "embarcado.h"
#include "ident.h"
#include "saida.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PI 3.14159265358979323846
#define TAM 0.01           /* amostragem [s] */
#define DUR 20.0           /* gravação depois da soltura [s] */
#define ESPERA_MAX 30.0    /* espera pela soltura [s] */
#define NMAX 5010          /* (DUR + ESPERA_MAX) / TAM + folga */
#define PMAX 400           /* meios-ciclos */

static float s_phi[NMAX];
static double s_tz[PMAX + 1], s_tp[PMAX], s_ap[PMAX], s_la[PMAX];

static double t_de(int i) { return i * TAM; }

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

static const char *analisa(int n, double g, double l2, double m2)
{
    const float *phi = s_phi;
    /* soltura = maior |φ| depois que passou de 0,1 rad (antes disso o pêndulo está na mão) */
    int i0 = -1;
    for (int i = 0; i < n; i++)
        if (fabsf(phi[i]) > 0.1f) {
            i0 = i;
            break;
        }
    if (i0 < 0) return "o pêndulo não passou de 0,1 rad: nada para identificar";
    int ipico = i0;
    for (int i = i0; i < n && t_de(i) < t_de(i0) + 3; i++)
        if (fabsf(phi[i]) > fabsf(phi[ipico])) ipico = i;
    /* a partir do pico de soltura: cruzamentos por zero (interpolados) e picos entre eles, até a amplitude cair
     * a 0,02 rad (~13 vezes a resolução do encoder) */
    int nz = 0, np = 0;
    double amax = 0;
    for (int i = ipico + 1; i < n && np < PMAX; i++) {
        amax = fmax(amax, fabsf(phi[i]));
        if ((phi[i - 1] < 0) != (phi[i] < 0)) {
            if (amax < 0.02) break;
            s_tz[nz++] = t_de(i - 1) + TAM * phi[i - 1] / (phi[i - 1] - phi[i]);
            if (nz >= 2) {                                  /* pico entre os dois últimos cruzamentos */
                double tm = 0, am = 0;
                for (int j = i - 1; j > 0 && t_de(j) >= s_tz[nz - 2]; j--)
                    if (fabsf(phi[j]) > am) am = fabsf(phi[j]), tm = t_de(j);
                s_tp[np] = tm, s_ap[np++] = am;
            }
            amax = 0;
        }
    }
    if (np < 4) return "poucos ciclos (menos de 4 meios-ciclos com amplitude > 0,02 rad)";
    /* período: meio-ciclos corrigidos para a amplitude (T(a) ≈ T0 (1 + a²/16)) */
    double soma = 0;
    for (int k = 0; k < np; k++) soma += (s_tz[k + 1] - s_tz[k]) / (1 + s_ap[k] * s_ap[k] / 16);
    double Td = 2 * soma / np, wd = 2 * PI / Td;
    /* decaimento: ln(a) linear em t (viscoso) e a linear em t (Coulomb), para comparar */
    double a, b, b_c, a_c;
    for (int k = 0; k < np; k++) s_la[k] = log(s_ap[k]);
    double r2v = reta(s_tp, s_la, np, &a, &b), r2c = reta(s_tp, s_ap, np, &a_c, &b_c);
    double sigma = -b, wn = sqrt(wd * wd + sigma * sigma), zeta = sigma / wn;
    double Jm = g * l2 / (wn * wn), I2m = Jm - l2 * l2, mu2m = 2 * sigma * Jm;

    saida_printf("%d meios-ciclos, amplitude %.3f -> %.3f rad em %.1f s\n", np, s_ap[0], s_ap[np - 1],
                 s_tp[np - 1] - s_tp[0]);
    saida_printf("período (pequenas oscilações) %.4f s -> wn = %.3f rad/s, zeta = %.4f\n", Td, wn, zeta);
    saida_printf("decaimento: exponencial (atrito viscoso) R² = %.4f | linear (atrito seco) R² = %.4f%s\n", r2v, r2c,
                 r2c > r2v ? "  <- atrito seco domina: mu2 viscoso é só aproximação" : "");
    saida_printf("J/m2 = %.4e m², I2/m2 = %.4e m², mu2/m2 = %.4e m²/s\n", Jm, I2m, mu2m);
    if (I2m <= 0) saida_printf("ATENÇÃO: I2 < 0 -> l2 = %.4f inconsistente com o período (confira a medida)\n", l2);
    saida_printf("para o furuta.ini ([fisico]; m2 = %.5f só escala I2 e mu2):\nl2  = %.5f\nI2  = %.4e\nmu2 = %.4e\n",
                 m2, l2, I2m * m2, mu2m * m2);
    saida_printf("(nesta sessão: cfg -set fisico.l2=%.5f, cfg -set fisico.I2=%.4e, cfg -set fisico.mu2=%.4e)\n", l2,
                 I2m * m2, mu2m * m2);
    return NULL;
}

const char *ident_roda(double l2)
{
    if (embarcado_ativo()) return "tem um ensaio rodando";
    if (controle_estado() == EST_RODANDO || controle_estado() == EST_HOMING) return "a malha está ocupada";
    if (!(l2 > 0.01 && l2 < 1)) return "informe o l2 medido em metros (ex.: furuta -ident 0.085)";
    double m2, g;
    embarcado_massa(&m2, &g);
    controle_origem_aqui();                       // trava o braço aqui (sem homing)
    if (controle_estado() != EST_PRONTO) return "não travou o braço";
    saida_printf("braço travado. SOLTE o pêndulo quando quiser (até %.0f s) e não encoste mais.\n", ESPERA_MAX);

    int n = 0;
    double t_solto = -1;
    const char *erro = NULL;
    TickType_t ultimo = xTaskGetTickCount();
    while (n < NMAX) {
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS((int)(TAM * 1000)));
        if (!controle_acel(0.0f)) {               // aceleração 0: braço parado, watchdog alimentado
            erro = controle_erro()[0] ? controle_erro() : "a malha saiu de READY";
            break;
        }
        leitura_t x;
        controle_leitura(&x);
        double tt = t_de(n);
        s_phi[n++] = x.t2 - (float)PI;
        if (t_solto < 0 && fabsf(s_phi[n - 1]) > 0.1f && fabsf(x.dt2) > 1.0f) {   // saiu da mão: ganhou velocidade
            t_solto = tt;
            saida_printf("solto em t = %.1f s, gravando %.0f s...\n", tt, DUR);
        }
        if (t_solto < 0 && tt > ESPERA_MAX) {
            erro = "não detectou a soltura";
            break;
        }
        if (t_solto >= 0 && tt > t_solto + DUR) break;
    }
    controle_habilita(false);
    saida_printf("%d amostras, driver desligado (refaça o homing)\n", n);
    if (erro) return erro;
    return analisa(n, g, l2, m2);
}
