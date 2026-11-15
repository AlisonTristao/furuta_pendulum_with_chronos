/* Inclinação da mesa pela varredura do braço (furuta -inclina; também no -auto, entre o homing e o start).
 *
 * Com o eixo do braço fora da vertical, o pêndulo gira junto com o braço e a vertical verdadeira, vista no plano em
 * que ele oscila (perpendicular ao braço), sai do zero do θ2 por
 *     δ(θ1) = −gx·sin θ1 + gy·cos θ1 + erro do zero do encoder,
 * com (gx, gy) a inclinação da mesa [rad]. Pendurado ou em pé, o pêndulo fica ao longo da mesma vertical, então
 * medir δ embaixo dá o topo de verdade. Sem a correção, o controle equilibra em torno do θ2 = 0 do encoder, o
 * pêndulo de verdade fica torto, e o braço vai parar num θ1 qualquer para segurar isso.
 *
 * inclinacao.pontos = 1 (padrão do -auto): só o zero, com uma oscilação em θ1 = 0 (~8 s), e seno/cosseno da config.
 *
 * Varredura: o braço para em `pontos` posições igualmente espaçadas no curso do homing (os movimentos são feitos
 * pela malha de 1 kHz, controle_vai/controle_excita: comandar a aceleração daqui, a 10 ms, com a leitura um período
 * atrasada, deixava o braço oscilando em volta do alvo sem nunca assentar). Em cada uma o braço excita o pêndulo em
 * ressonância, θ̈1 = A cos(ωn t) por `ciclos` ciclos (termina parado no mesmo lugar, depois de andar até 2A/ωn²),
 * fica parado e grava `grava` s; o centro da oscilação (ident_centro: o mesmo do furuta -calibra, imune ao atrito
 * seco) é δ naquele θ1.
 *
 * Excitação: pendurado, φ̈ = −ωn² φ − k θ̈1 com k = L1·l2/(J/m2) (seção [fisico]). Em ressonância, a partir do
 * repouso, a amplitude cresce k·A·t/(2ωn): N ciclos (t = 2πN/ωn) dão  amplitude = k·A·π·N/ωn², então
 * A = amplitude·ωn²/(k·π·N). Um tranco curto (+a, −a, +a) com 20 rad/s² dava só ~0,13 rad sem atrito (k ≈ 0,66:
 * o braço efetivo é curto), e o atrito seco apagava a oscilação antes de 3 trincas.
 *
 * Ajuste por mínimos quadrados em  δ = s·sin θ1 + q·(cos θ1 − 1) + c:  c = δ(0) vai para o zero do encoder
 * (o θ2 parado embaixo em θ1 = 0 vira π), e s, q ficam em inclinacao.seno/cosseno (controlador_inclinacao). Com o
 * curso limitado (~±80°) c e o termo em cos θ1 puro seriam quase colineares; com (cos θ1 − 1) o c é bem
 * determinado e o q só pega a curvatura. O que importa para o controle é a soma, dentro do curso medido. */
#include "controle.h"
#include "embarcado.h"
#include "ident.h"
#include "inclina.h"
#include "saida.h"

#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PI 3.14159265358979323846
#define TAM 0.01           /* amostragem da gravação [s] (o ident_centro espera 10 ms) */
#define MARGEM 0.05f       /* do limite de software, além da excursão da excitação [rad] */
#define PONTOS_MAX 15
#define PONTOS_INCLINA 7   /* furuta -inclina: a mesa inteira, qualquer que seja o inclinacao.pontos do -auto */
#define GRAVA_MAX 8.0      /* parada em READY por 10 s a malha desliga o driver (DESLIGA_OCIOSO_MS) */
#define NMAX 802           /* GRAVA_MAX / TAM + folga */

static float s_phi[NMAX];

/* Espera o movimento da malha acabar (controle_vai/controle_excita). NULL se acabou em READY. */
static const char *espera_mov(double max_s)
{
    for (int i = 0; controle_movendo(); i++) {
        if (i * 0.01 > max_s) return "o movimento do braço não acabou a tempo";
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (controle_estado() != EST_PRONTO) return controle_erro()[0] ? controle_erro() : "a malha saiu de READY";
    return NULL;
}

static const char *vai(float alvo)
{
    if (!controle_vai(alvo)) return "a malha não aceitou o movimento (não está em READY)";
    return espera_mov(20);
}

/* Excitação ressonante (pela malha, a 1 kHz: termina parado no mesmo lugar) e grava n amostras de φ = θ2 − π com
 * o braço parado. θ1 médio da gravação em *t1, maior |φ − média| em *amp. NULL se deu certo. */
static const char *oscila_e_grava(float A, float wn, int ciclos, int n, double *t1, double *amp)
{
    if (!controle_excita(A, wn, ciclos)) return "a malha não aceitou a excitação (não está em READY)";
    const char *e = espera_mov(2 + ciclos * 6.3 / wn);
    if (e) return e;
    double soma = 0;
    TickType_t ultimo = xTaskGetTickCount();
    for (int i = 0; i < n; i++) {
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS((int)(TAM * 1000)));
        leitura_t x;
        controle_leitura(&x);
        s_phi[i] = x.t2 - (float)PI;
        soma += x.t1;
    }
    *t1 = soma / n;
    double media = 0;
    for (int i = 0; i < n; i++) media += s_phi[i] / n;
    *amp = 0;
    for (int i = 0; i < n; i++) *amp = fmax(*amp, fabs(s_phi[i] - media));
    if (controle_estado() != EST_PRONTO) return controle_erro()[0] ? controle_erro() : "a malha saiu de READY";
    return NULL;
}

/* Resolve A·p = b (3x3) por eliminação de Gauss com pivô parcial. false se singular. */
static bool resolve3(double A[3][3], double b[3], double p[3])
{
    for (int c = 0; c < 3; c++) {
        int piv = c;
        for (int r = c + 1; r < 3; r++)
            if (fabs(A[r][c]) > fabs(A[piv][c])) piv = r;
        if (fabs(A[piv][c]) < 1e-12) return false;
        for (int k = 0; k < 3; k++) {
            double t = A[c][k];
            A[c][k] = A[piv][k], A[piv][k] = t;
        }
        double t = b[c];
        b[c] = b[piv], b[piv] = t;
        for (int r = c + 1; r < 3; r++) {
            double f = A[r][c] / A[c][c];
            for (int k = c; k < 3; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 2; r >= 0; r--) {
        double s = b[r];
        for (int k = r + 1; k < 3; k++) s -= A[r][k] * p[k];
        p[r] = s / A[r][r];
    }
    return true;
}

const char *inclina_roda(bool varredura)
{
    if (embarcado_ativo()) return "tem um ensaio rodando";
    if (controle_estado() != EST_PRONTO && controle_estado() != EST_RODANDO) return "faça o homing antes (furuta -home)";
    inclinacao_params ip;
    embarcado_inclinacao_params(&ip);
    int pontos = !varredura ? ip.pontos : ip.pontos >= 4 ? ip.pontos : PONTOS_INCLINA;
    if (pontos != 1 && (pontos < 4 || pontos > PONTOS_MAX)) return "inclinacao.pontos: 1 (só o zero) ou 4..15";
    if (!(ip.grava >= 2 && ip.grava <= GRAVA_MAX)) return "inclinacao.grava fora de 2..8 s";
    if (!(ip.amplitude > 0.05 && ip.amplitude <= 2.0)) return "inclinacao.amplitude fora de 0,05..2 rad";
    if (ip.ciclos < 1 || ip.ciclos > 10) return "inclinacao.ciclos fora de 1..10";
    if (!(ip.wn > 1 && ip.k > 0.01)) return "seção [fisico] sem L1/l2/I2/m2 válidos";
    double a_exc = ip.amplitude * ip.wn * ip.wn / (ip.k * PI * ip.ciclos);
    if (a_exc > 50) return "a excitação passaria de 50 rad/s² (diminua inclinacao.amplitude ou aumente ciclos)";
    float excursao = (float)(2 * a_exc / (ip.wn * ip.wn));
    float L = controle_limite() - excursao - MARGEM;
    if (L < 0.3f) return "curso pequeno demais para a varredura (diminua inclinacao.amplitude)";
    int n = (int)lround(ip.grava / TAM);
    saida_aviso("varredura da inclinação: %d posições em θ1 = ±%.2f rad; excitação %.1f rad/s² x %d ciclos de %.3f s "
                "(pêndulo até %.2f rad sem atrito, braço ±%.2f rad); %.1f s gravados em cada",
                pontos, (double)L, a_exc, ip.ciclos, 2 * PI / ip.wn, ip.amplitude, (double)excursao, ip.grava);

    double th[PONTOS_MAX], d[PONTOS_MAX];
    int m = 0;
    const char *erro = NULL;
    for (int k = 0; k < pontos; k++) {
        float alvo = pontos == 1 ? 0.0f : -L + 2 * L * k / (pontos - 1);
        if ((erro = vai(alvo))) break;
        double t1;
        double amp;
        if ((erro = oscila_e_grava((float)a_exc, (float)ip.wn, ip.ciclos, n, &t1, &amp))) break;
        double c;
        int trincas;
        const char *e = ident_centro(s_phi, n, &c, &trincas);
        if (e) {
            saida_aviso("  θ1 %+.3f: amplitude %.3f rad, descartado (%s; aumente inclinacao.amplitude)", t1, amp, e);
            continue;
        }
        th[m] = t1, d[m] = c;
        saida_aviso("  θ1 %+.3f rad: amplitude %.3f rad, centro %+.4f rad (%+.2f°), %d trincas", t1, amp, c,
                    c * 180 / PI, trincas);
        m++;
    }
    const char *volta = erro ? NULL : vai(0.0f);   // braço no meio para o start
    if (erro) return erro;
    if (volta) return volta;
    if (pontos == 1) {                            // só o zero: centro em θ1 ≈ 0 menos a inclinação da config
        if (m < 1) return "a oscilação não deu o centro: zero não corrigido";
        double c = d[0] - (ip.seno * sin(th[0]) + ip.cosseno * (cos(th[0]) - 1));
        controle_desloca_pendulo((float)-c);
        embarcado_zero_calibrado();
        saida_aviso("zero do θ2 pelo centro da oscilação em θ1 = %+.3f: corrigido em %+.4f rad (%+.2f°)", th[0], -c,
                    -c * 180 / PI);
        return NULL;
    }
    if (m < 4) return "menos de 4 posições com oscilação suficiente: nada ajustado";

    /* mínimos quadrados em [sin θ1, cos θ1 − 1, 1] */
    double A[3][3] = {{0}}, b[3] = {0}, p[3];
    for (int i = 0; i < m; i++) {
        double f[3] = {sin(th[i]), cos(th[i]) - 1, 1};
        for (int r = 0; r < 3; r++) {
            for (int s = 0; s < 3; s++) A[r][s] += f[r] * f[s];
            b[r] += f[r] * d[i];
        }
    }
    if (!resolve3(A, b, p)) return "ajuste singular (posições muito juntas?)";
    double ssr = 0;
    for (int i = 0; i < m; i++) {
        double r = d[i] - (p[0] * sin(th[i]) + p[1] * (cos(th[i]) - 1) + p[2]);
        ssr += r * r;
    }
    double rms = sqrt(ssr / m), beta = hypot(p[0], p[1]);
    double var = 0;                               /* quanto o topo verdadeiro anda dentro do curso */
    for (int i = 0; i < m; i++) var = fmax(var, fabs(p[0] * sin(th[i]) + p[1] * (cos(th[i]) - 1)));

    controle_desloca_pendulo((float)-p[2]);       // θ2 parado embaixo em θ1 = 0 vira π
    embarcado_zero_calibrado();
    if (!embarcado_inclinacao_define(p[0], p[1])) return "não deu para guardar (ensaio rodando?)";
    saida_aviso("ajuste: δ(θ1) = %+.4f·sin θ1 %+.4f·(cos θ1 − 1) %+.4f rad | resíduo rms %.4f rad (%.2f°)",
                 p[0], p[1], p[2], rms, rms * 180 / PI);
    saida_aviso("inclinação da mesa ≈ %.2f° (o topo verdadeiro anda até %.2f° no curso medido); zero do θ2 "
                 "corrigido em %+.4f rad (%+.2f°)",
                 beta * 180 / PI, var * 180 / PI, -p[2], -p[2] * 180 / PI);
    saida_aviso("gravado na flash: seno = %.4f, cosseno = %.4f (vale depois do boot; cfg -show mostra); offset do "
                "topo (trim.valor) mantido. O motor esquentou: espere esfriar antes do start",
                p[0], p[1]);
    if (rms > 0.01) saida_aviso("ATENÇÃO: resíduo alto: pêndulo encostando, folga no encoder ou mesa mexendo?");
    return NULL;
}
