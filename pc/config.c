#include "config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void padroes(config *c)
{
    /* pêndulo do artigo (ICIT 27) */
    c->f = (fisico){.m2 = 0.270 / 3, .L1 = 0.170 / 2, .l2 = 0.130, .I1 = 0.005, .I2 = 5.07e-4,
                    .mu1 = 0.001, .mu2 = 0.001, .g = 9.81};
    c->mpc = mpc_config_artigo();
    ensaio e = {0};
    e.Ts = 0.02;
    e.duracao = 10;
    e.t_degrau = 1e9;
    e.t_disturbio = 1e9;
    e.x0[1] = 3.14159265358979;
    e.rk4_sub = 10;
    strcpy(e.porta, "COM7");
    e.margem_theta1 = 0.0;
    e.usa_limite_homing = 1;
    e.theta2_aborta = 60;
    e.janela_equilibrio = 0.15;
    e.est_L1 = 0.5;
    e.est_L2 = 20;
    c->e = e;
}

static char *apara(char *s)
{
    while (isspace((unsigned char)*s)) s++;
    char *f = s + strlen(s);
    while (f > s && isspace((unsigned char)f[-1])) *--f = '\0';
    return s;
}

/* "a, b, c" -> até n números */
static int lista(const char *v, double *out, int n)
{
    int k = 0;
    char *p = (char *)v, *fim;
    while (k < n) {
        out[k] = strtod(p, &fim);
        if (fim == p) break;
        k++;
        p = fim;
        while (*p == ',' || isspace((unsigned char)*p)) p++;
    }
    return k;
}

static int aplica(config *c, const char *sec, const char *k, const char *v)
{
    double d = atof(v);
#define NUM(S, K, DST)                                    \
    if (!strcmp(sec, S) && !strcmp(k, K)) {               \
        DST = d;                                          \
        return 0;                                         \
    }
#define INT(S, K, DST)                                    \
    if (!strcmp(sec, S) && !strcmp(k, K)) {               \
        DST = atoi(v);                                    \
        return 0;                                         \
    }
    NUM("fisico", "m2", c->f.m2) NUM("fisico", "L1", c->f.L1) NUM("fisico", "l2", c->f.l2)
    NUM("fisico", "I1", c->f.I1) NUM("fisico", "I2", c->f.I2) NUM("fisico", "mu1", c->f.mu1)
    NUM("fisico", "mu2", c->f.mu2) NUM("fisico", "g", c->f.g)

    INT("mpc", "Np", c->mpc.Np) NUM("mpc", "Rdu", c->mpc.Rdu)
    NUM("mpc", "u_min", c->mpc.u_min) NUM("mpc", "u_max", c->mpc.u_max)
    NUM("mpc", "du_min", c->mpc.du_min) NUM("mpc", "du_max", c->mpc.du_max)
    NUM("mpc", "theta1_min", c->mpc.theta1_min) NUM("mpc", "theta1_max", c->mpc.theta1_max)
    NUM("mpc", "dtheta_max", c->mpc.dtheta_max) NUM("mpc", "qv_theta1", c->mpc.qv_theta1)
    NUM("mpc", "qv_dtheta", c->mpc.qv_dtheta) INT("mpc", "rho_avg", c->mpc.rho_avg)
    INT("mpc", "terminal_dlqr", c->mpc.terminal_dlqr) NUM("mpc", "Ru_lqr", c->mpc.Ru_lqr)
    INT("mpc", "l_max", c->mpc.l_max) NUM("mpc", "eps_rho", c->mpc.eps_rho)
    INT("mpc", "max_iter", c->mpc.op.max_iter) NUM("mpc", "t_barreira", c->mpc.op.t)
    NUM("mpc", "eps", c->mpc.op.eps) NUM("mpc", "peso_exp", c->mpc.peso_exp) NUM("mpc", "horizonte", c->mpc.horizonte)
    if (!strcmp(sec, "mpc") && (!strcmp(k, "Q") || !strcmp(k, "peso_exp_estados"))) {
        double t[4];
        if (lista(v, t, 4) != 4) return -1;
        real *dst = !strcmp(k, "Q") ? c->mpc.Q : c->mpc.peso_exp_estados;   /* real: float no controlador do ESP32 */
        for (int i = 0; i < 4; i++) dst[i] = (real)t[i];
        return 0;
    }

    NUM("ensaio", "Ts", c->e.Ts) NUM("ensaio", "duracao", c->e.duracao)
    NUM("ensaio", "theta1_ref", c->e.theta1_ref) NUM("ensaio", "t_degrau", c->e.t_degrau)
    NUM("ensaio", "theta1_degrau", c->e.theta1_degrau) NUM("ensaio", "margem_theta1", c->e.margem_theta1)
    INT("ensaio", "usa_limite_homing", c->e.usa_limite_homing) NUM("ensaio", "dtheta2_aborta", c->e.theta2_aborta)
    NUM("ensaio", "janela_equilibrio", c->e.janela_equilibrio)
    INT("ensaio", "estimador", c->e.estimador) NUM("ensaio", "est_L1", c->e.est_L1) NUM("ensaio", "est_L2", c->e.est_L2)
    NUM("ensaio", "comp_atraso_ms", c->e.comp_atraso_ms)
    if (!strcmp(sec, "ensaio") && !strcmp(k, "modo")) {
        if (strcmp(v, "equilibrio") && strcmp(v, "swingup")) return -1;
        c->e.equilibrio = !strcmp(v, "equilibrio");
        return 0;
    }
    if (!strcmp(sec, "ensaio") && !strcmp(k, "porta")) {
        snprintf(c->e.porta, sizeof(c->e.porta), "%s", v);
        return 0;
    }

    NUM("sim", "t_disturbio", c->e.t_disturbio) NUM("sim", "u_disturbio", c->e.u_disturbio)
    NUM("sim", "dur_disturbio", c->e.dur_disturbio) NUM("sim", "tc2", c->e.tc2) INT("sim", "rk4_sub", c->e.rk4_sub)
    INT("sim", "planta_acel", c->e.planta_acel) NUM("sim", "obs_hz", c->e.obs_hz)
    NUM("sim", "atraso_ms", c->e.atraso_ms) INT("sim", "bits_enc", c->e.bits_enc)
    NUM("sim", "ruido_enc", c->e.ruido_enc) NUM("sim", "passos_rev", c->e.passos_rev)
    NUM("sim", "a_max", c->e.a_max_fw) NUM("sim", "v_max", c->e.v_max_fw) NUM("sim", "limite_sw", c->e.limite_fw)
    NUM("sim", "f_motor", c->e.f_motor) NUM("sim", "zeta_motor", c->e.zeta_motor)
    NUM("sim", "mu2_planta", c->e.mu2_planta) INT("sim", "semente", c->e.semente)
    NUM("sim", "ripple_passo", c->e.ripple_passo)
    if (!strcmp(sec, "sim") && !strcmp(k, "x0")) return lista(v, c->e.x0, 4) == 4 ? 0 : -1;
#undef NUM
#undef INT
    return -1;
}

/* Pêndulo = haste uniforme (massa mh, comprimento L, do eixo à ponta) + massa pontual mp na ponta:
 * m2 = mh + mp, l2 = centro de massa, I2 em torno do centro de massa. */
static void pendulo_composto(fisico *f, double mh, double L, double mp)
{
    f->m2 = mh + mp;
    f->l2 = (mh * L / 2 + mp * L) / f->m2;
    double dh = L / 2 - f->l2, dp = L - f->l2;
    f->I2 = mh * L * L / 12 + mh * dh * dh + mp * dp * dp;
}

int config_le(const char *arquivo, config *c)
{
    padroes(c);
    if (!arquivo) return 0;
    FILE *fp = fopen(arquivo, "r");
    if (!fp) {
        fprintf(stderr, "não abriu %s\n", arquivo);
        return -1;
    }
    char linha[256], sec[32] = "";
    int erros = 0, tem_geo = 0;
    double geo[3] = {0, 0, 0};   /* m_haste, L_haste, m_ponta */
    while (fgets(linha, sizeof(linha), fp)) {
        char *p = strpbrk(linha, ";#");
        if (p) *p = '\0';
        p = apara(linha);
        if (!*p) continue;
        if (*p == '[') {
            char *f = strchr(p, ']');
            if (f) *f = '\0';
            snprintf(sec, sizeof(sec), "%s", apara(p + 1));
            continue;
        }
        char *ig = strchr(p, '=');
        if (!ig) continue;
        *ig = '\0';
        char *k = apara(p), *v = apara(ig + 1);
        if (!strcmp(sec, "pendulo")) {           /* geometria: calcula m2, l2, I2 no fim */
            if (!strcmp(k, "m_haste")) geo[0] = atof(v), tem_geo = 1;
            else if (!strcmp(k, "L_haste")) geo[1] = atof(v), tem_geo = 1;
            else if (!strcmp(k, "m_ponta")) geo[2] = atof(v), tem_geo = 1;
            else {
                fprintf(stderr, "%s: chave desconhecida: [%s] %s\n", arquivo, sec, k);
                erros++;
            }
            continue;
        }
        if (aplica(c, sec, k, v)) {
            fprintf(stderr, "%s: chave desconhecida ou valor inválido: [%s] %s = %s\n", arquivo, sec, k, v);
            erros++;
        }
    }
    fclose(fp);
    if (tem_geo) pendulo_composto(&c->f, geo[0], geo[1], geo[2]);
    return erros ? -1 : 0;
}

int config_arg(config *c, const char *arg)
{
    char b[160];
    snprintf(b, sizeof(b), "%s", arg);
    char *ig = strchr(b, '='), *pt = strchr(b, '.');
    if (!ig || !pt || pt > ig) {
        fprintf(stderr, "argumento inválido: %s (use secao.chave=valor)\n", arg);
        return -1;
    }
    *ig = *pt = '\0';
    if (aplica(c, apara(b), apara(pt + 1), apara(ig + 1))) {
        fprintf(stderr, "chave desconhecida ou valor inválido: %s\n", arg);
        return -1;
    }
    return 0;
}

int config_le_args(int argc, char **argv, config *c, const char **livre)
{
    if (config_le(argv[1], c)) return -1;
    *livre = NULL;
    for (int i = 2; i < argc; i++) {
        if (strchr(argv[i], '=')) {
            if (config_arg(c, argv[i])) return -1;
            printf("sobrescrito: %s\n", argv[i]);
        } else if (!*livre) {
            *livre = argv[i];
        }
    }
    return 0;
}

void config_imprime(const config *c)
{
    const fisico *f = &c->f;
    const mpc_config *m = &c->mpc;
    printf("fisico: m2=%g L1=%g l2=%g I1=%g I2=%g mu1=%g mu2=%g g=%g\n", f->m2, f->L1, f->l2, f->I1, f->I2, f->mu1,
           f->mu2, f->g);
    printf("mpc: Ts=%g Np=%d Q=(%g,%g,%g,%g) Rdu=%g u=[%g,%g] du=[%g,%g] theta1=[%g,%g] dtheta<=%g qv=(%g,%g)\n",
           c->e.Ts, m->Np, m->Q[0], m->Q[1], m->Q[2], m->Q[3], m->Rdu, m->u_min, m->u_max, m->du_min, m->du_max,
           m->theta1_min, m->theta1_max, m->dtheta_max, m->qv_theta1, m->qv_dtheta);
    if (m->horizonte > m->Np * c->e.Ts) {
        double inc = 2 * (m->horizonte - m->Np * c->e.Ts) / ((double)m->Np * (m->Np - 1));
        printf("passo variável: horizonte %g s em %d estágios, h = %g ms + k·%.3g ms (último %.1f ms)\n", (double)m->horizonte,
               m->Np, c->e.Ts * 1e3, inc * 1e3, (c->e.Ts + (m->Np - 1) * inc) * 1e3);
    }
    printf("     rho_avg=%d terminal_dlqr=%d l_max=%d eps_rho=%g max_iter=%d t=%g\n", m->rho_avg, m->terminal_dlqr,
           m->l_max, m->eps_rho, m->op.max_iter, m->op.t);
}
