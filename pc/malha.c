/* Malha fechada no pêndulo real: o PC roda o NMPC qLPV (CHRONOS em C) e o ESP32 é a interface com o hardware.
 *
 *   malha.exe <config.ini> [saida.csv]      (padrão: ensaios/ensaio_AAAAMMDD_HHMMSS.csv)
 *
 * Sequência: conecta -> pêndulo parado embaixo, Enter: zera θ2 (Z) -> Enter: homing (H; o limite de θ1 do MPC vem
 * daqui) -> espera a condição de partida mandando aceleração 0 (modo swingup: pêndulo parado embaixo por 1 s; modo
 * equilibrio: você leva o pêndulo até |θ2| < janela_equilibrio) -> laço de Ts. A cada período: X (lê o estado), MPC
 * (torque u), u -> θ̈1 pelo modelo, envia θ̈1. Termina no fim da duração, no Ctrl+C ou em qualquer erro: para de
 * mandar comandos (o firmware freia o braço em 200 ms) e desliga o driver (E0).
 *
 * CSV: t, θ1, θ2, dθ1, dθ2, θ1_ref, u, a, n_qp, n_it, dP, t_calc_ms (MPC), t_ciclo_ms (X até a resposta do envio),
 *      atraso_ms (início do período em relação ao instante ideal) */
#include "config.h"
#include "furuta_serial.h"
#include "modelo.h"
#include "periodo.h"
#include "qlpv_mpc.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#include <windows.h>
#define mkdir_(d) _mkdir(d)
#else
#include <sys/stat.h>
#include <unistd.h>
#define mkdir_(d) mkdir(d, 0755)
#endif

#define PI 3.14159265358979323846

static volatile sig_atomic_t g_parar = 0;
static void ao_ctrl_c(int s)
{
    (void)s;
    g_parar = 1;
}

static void dorme_ms(int ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

static void espera_enter(const char *msg)
{
    printf("%s", msg);
    fflush(stdout);
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {
    }
}

/* Para o braço e desliga o driver. */
static void encerra(furuta *f)
{
    char r[128];
    dorme_ms(400);                                /* sem comando: o watchdog (200 ms) freia até parar */
    if (furuta_comando(f, "E0", r, sizeof(r), 200) == FURUTA_OK) printf("driver desligado (E0)\n");
    else printf("ATENÇÃO: E0 sem resposta (%s) - o firmware desliga sozinho após 10 s parado\n", r);
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
    const ensaio *e = &cfg.e;

    char arq[256];
    if (argc > 2) {
        snprintf(arq, sizeof(arq), "%s", argv[2]);
    } else {
        time_t agora = time(NULL);
        char data[32];
        strftime(data, sizeof(data), "%Y%m%d_%H%M%S", localtime(&agora));
        mkdir_("ensaios");
        snprintf(arq, sizeof(arq), "ensaios/ensaio_%s.csv", data);
    }

    furuta *f = furuta_conecta(e->porta, FURUTA_BAUD);
    if (!f) {
        fprintf(stderr, "não abriu %s\n", e->porta);
        return 1;
    }
    char r[128];
    furuta_limpa(f);
    if (furuta_comando(f, "S", r, sizeof(r), 300) != FURUTA_OK) {
        fprintf(stderr, "o ESP32 não respondeu em %s (%s)\n", e->porta, r);
        return 1;
    }
    printf("ESP32: %s\n", r);

    config_imprime(&cfg);
    printf("\nModo %s, %.0f s, Ts = %.0f ms.\n", e->equilibrio ? "EQUILÍBRIO" : "SWING-UP", e->duracao, e->Ts * 1e3);
    espera_enter("1) Deixe o pêndulo PARADO, pendurado embaixo, e aperte Enter (zera o encoder)... ");
    furuta_comando(f, "Z", r, sizeof(r), 300);
    double x[4];
    if (furuta_estado(f, x, 100) != FURUTA_OK) {
        fprintf(stderr, "leitura do estado falhou: %s\n", furuta_ultima_resposta(f));
        return 1;
    }
    printf("   θ2 = %.3f (pi = embaixo)\n", x[1]);
    espera_enter("2) Enter para começar: homing e, em seguida, o controle (Ctrl+C para parar)... ");

    /* homing: limite de θ1. Daqui em diante sempre há comando a cada <= 20 ms (o driver não desliga) */
    signal(SIGINT, ao_ctrl_c);
    printf("homing (o braço vai até os dois fins de curso e volta ao meio)...\n");
    if (furuta_comando(f, "H", r, sizeof(r), 60000) != FURUTA_OK) {
        fprintf(stderr, "homing falhou: %s\n", r);
        encerra(f);
        return 1;
    }
    double meio = 0, lim = 0;
    sscanf(r, "OK,%lf,%lf", &meio, &lim);
    printf("homing ok: meio curso %.3f rad, limite de software %.3f rad\n", meio, lim);
    if (e->usa_limite_homing) {
        cfg.mpc.theta1_max = lim - e->margem_theta1;
        cfg.mpc.theta1_min = -cfg.mpc.theta1_max;
        printf("restrição de θ1 do MPC: ±%.3f rad\n", cfg.mpc.theta1_max);
    }

    /* espera a condição de partida mandando aceleração 0 (braço parado no meio) */
    if (e->equilibrio)
        printf("\nLEVE O PÊNDULO ATÉ O TOPO com a mão. O controle assume quando |θ2| < %.2f rad (%.0f graus) e\n"
               "|dθ2| < 2 rad/s; solte quando sentir o braço mexer.\n",
               e->janela_equilibrio, e->janela_equilibrio * 57.3);
    else
        printf("esperando o pêndulo parar embaixo (a homing balança ele)...\n");
    double tw = periodo_agora(), t_quieto = -1;
    for (;;) {
        if (g_parar || periodo_agora() - tw > 60) {
            printf("%s\n", g_parar ? "Ctrl+C" : "60 s sem a condição de partida");
            encerra(f);
            return 1;
        }
        if (furuta_acel(f, 0.0, x, 20) != FURUTA_OK) {
            fprintf(stderr, "comunicação falhou antes de começar: %s\n", furuta_ultima_resposta(f));
            encerra(f);
            return 1;
        }
        double th = x[1] - 2 * PI * floor((x[1] + PI) / (2 * PI));   /* (-π, π], 0 = topo */
        if (e->equilibrio) {
            if (fabs(th) < e->janela_equilibrio && fabs(x[3]) < 2) break;
        } else {                                                     /* parado embaixo por 1 s */
            int quieto = fabs(fabs(th) - PI) < 0.1 && fabs(x[3]) < 0.1;
            if (!quieto) t_quieto = -1;
            else if (t_quieto < 0) t_quieto = periodo_agora();
            else if (periodo_agora() - t_quieto > 1.0) break;
        }
        dorme_ms(10);
    }
    printf("controle ligado\n");
    /* θ2 no múltiplo de 2π mais próximo de 0 no início (o pêndulo pode ter subido por qualquer lado); depois
     * fica contínuo para não pular a predição quando passa por baixo */
    double off2 = e->equilibrio ? 2 * PI * floor((x[1] + PI) / (2 * PI))   /* topo: θ2 em (-π, π] */
                                : 2 * PI * floor(x[1] / (2 * PI));         /* embaixo: θ2 em [0, 2π), ≈ π */
    x[1] -= off2;

    modelo m = modelo_de_fisico(cfg.f, e->Ts, cfg.mpc.dtheta_max);
    mpc *c = mpc_cria(&cfg.mpc, &m, x, 0.0);
    FILE *out = fopen(arq, "w");
    if (!out) {
        fprintf(stderr, "não abriu %s\n", arq);
        encerra(f);
        return 1;
    }
    fprintf(out, "t,theta1,theta2,dtheta1,dtheta2,theta1_ref,u,a,n_qp,n_it,dP,t_calc_ms,t_ciclo_ms,atraso_ms\n");

    periodo_prioridade_alta();
    periodo *p = periodo_cria(e->Ts);
    int K = (int)lround(e->duracao / e->Ts), k, estouros = 0;
    double soma_calc = 0, max_calc = 0, soma_ciclo = 0, max_ciclo = 0, max_atraso = 0;
    const char *motivo = "fim da duração";
    double t0 = periodo_agora();
    for (k = 0; k < K && !g_parar; k++) {
        double atraso = periodo_espera(p) * 1e3;
        double ti = periodo_agora();
        double t = ti - t0;
        int st = furuta_estado(f, x, 10);
        if (st != FURUTA_OK) {
            motivo = st == FURUTA_REINICIOU ? "o ESP32 reiniciou" : "leitura do estado falhou";
            break;
        }
        x[1] -= off2;
        double ref[4] = {t >= e->t_degrau ? e->theta1_degrau : e->theta1_ref, 0, 0, 0};
        mpc_info in;
        double tc0 = periodo_agora();
        double u = mpc_passo(c, x, ref, &in);
        double tcalc = (periodo_agora() - tc0) * 1e3;
        if (!in.finito) {
            motivo = "controle não finito";
            break;
        }
        double a = modelo_acel_braco(&m, x, u);
        double xa[4];
        st = furuta_acel(f, a, xa, 10);
        double tciclo = (periodo_agora() - ti) * 1e3;
        fprintf(out, "%.4f,%.6f,%.6f,%.5f,%.5f,%.5f,%.6f,%.4f,%d,%d,%.3e,%.4f,%.4f,%.4f\n", t, x[0], x[1], x[2],
                x[3], ref[0], u, a, in.n_qp, in.n_it, in.dP, tcalc, tciclo, atraso);
        if (st != FURUTA_OK) {
            motivo = st == FURUTA_ERR ? "o firmware recusou o comando (ver estado)" : "envio do comando falhou";
            break;
        }
        soma_calc += tcalc, soma_ciclo += tciclo;
        max_calc = fmax(max_calc, tcalc), max_ciclo = fmax(max_ciclo, tciclo), max_atraso = fmax(max_atraso, atraso);
        estouros += tciclo > e->Ts * 1e3;
        if (fabs(x[3]) > e->theta2_aborta) {
            motivo = "|dθ2| acima de dtheta2_aborta";
            break;
        }
    }
    if (g_parar) motivo = "Ctrl+C";
    fclose(out);
    printf("\nparou: %s (t = %.2f s, %d passos)\n", motivo, k * e->Ts, k);
    furuta_comando(f, "S", r, sizeof(r), 300);
    printf("ESP32: %s\n", r);
    encerra(f);
    if (k > 0)
        printf("MPC: %.2f ms médio, %.2f ms máx | ciclo (X + MPC + envio): %.2f ms médio, %.2f ms máx, %d acima de Ts "
               "| maior atraso do período: %.2f ms\n",
               soma_calc / k, max_calc, soma_ciclo / k, max_ciclo, estouros, max_atraso);
    printf("dados: %s\n", arq);
    mpc_libera(c);
    periodo_libera(p);
    furuta_desconecta(f);
    return 0;
}
