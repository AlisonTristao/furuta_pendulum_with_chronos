// Controle embarcado: o NMPC qLPV (comum/controlador.c, float) roda no núcleo 1, sincronizado com a malha de
// 1 kHz (controle.c, núcleo 0). Atraso fixo de um período: o θ̈1 calculado com a amostra k é aplicado na
// amostra k+1. A cada período uma amostra de telemetria vai para uma fila, lida por src/main.cpp (tópico controle).
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float t, x[4], u, a, t_calc_ms;   // t do ensaio [s]; x = θ1, θ2, dθ1, dθ2 (dθ2 do estimador)
    unsigned char n_qp, n_it;
    unsigned short estouros;
    float trim;                       // trim do zero do θ2 [rad] (controlador.h)
} telemetria_t;

// Resultado do último ensaio de controle (furuta -lote).
typedef struct {
    bool valido;
    int lado;
    bool partiu;           // o controle chegou a ligar (a condição de partida veio)
    double t_topo;         // 1ª vez no topo [s desde a partida], < 0 se não chegou
    int quedas;
    int pct_topo;          // % do ensaio no topo
    double duracao;        // quanto rodou [s]
    double temp_ini, temp_fim;   // driver [°C]
    const char *motivo;    // por que terminou
} ensaio_resultado;

typedef enum { FASE_PARADO, FASE_ESPERA, FASE_RODANDO, FASE_SECO, FASE_BENCH, FASE_SIM } fase_t;

void embarcado_init(void);   // configuração padrão = pc/furuta.ini embutido; tarefa do MPC no núcleo 1

// "secao.chave=valor" (as mesmas chaves do pc/furuta.ini). false se a chave não existir, o valor for inválido ou
// um ensaio estiver rodando. Vale até o próximo boot (ou embarcado_config_padrao).
bool embarcado_config(const char *par);
void embarcado_config_padrao(void);    // volta ao pc/furuta.ini embutido
// Resumo da configuração em uso (texto de uma linha por grupo). Devolve o tamanho escrito.
int embarcado_config_texto(char *buf, int n);
void embarcado_massa(double *m2, double *g);   // da configuração em uso (furuta -ident)
// O zero do θ2 veio da oscilação (furuta -calibra/-ident): o -start só confere o repouso, sem sobrescrever.
void embarcado_zero_calibrado(void);
// Varredura da inclinação (inclina.c): seção [inclinacao] em uso e, pela seção [fisico], o pêndulo pendurado
// como oscilador φ̈ = −ωn² φ − k θ̈1. _define guarda os coeficientes medidos (e zera o trim); false com ensaio
// rodando.
typedef struct {
    int pontos, ciclos;
    double grava, amplitude;   // [s], [rad]
    double wn, k;              // [rad/s], [1]
    double seno, cosseno;      // inclinação da mesa em uso [rad]
} inclinacao_params;
void embarcado_inclinacao_params(inclinacao_params *p);
// furuta -auto: lado do swing-up pelo sinal do offset do topo (ensaio.lado_auto · sinal(trim.valor)); 0 = sortear
// (lado_auto = 0 ou offset zero). embarcado_offset_topo = trim.valor em uso [rad].
int embarcado_lado_auto(void);
double embarcado_offset_topo(void);
bool embarcado_inclinacao_define(double seno, double cosseno);   // também grava na flash (NVS)
void embarcado_inclinacao_carrega(void);   // no boot, depois do device.begin(): a da flash, se houver

// Liga: modo 1 = controle (precisa de READY após o homing; espera a condição de partida do ensaio e então
// controla por ensaio.duracao); modo 2 = a seco (calcula com os sensores, mas não comanda o motor); modo 3 =
// benchmark do MPC com a configuração em uso e a planta simulada (não mexe no motor); modo 4 = simulação em tempo
// real (furuta -sim): o controle do ensaio contra a planta calibrada do sim.exe, com telemetria (não mexe no motor).
// Devolve NULL se ligou, ou o motivo da recusa.
const char *embarcado_liga(int modo);
void embarcado_desliga(void);
bool embarcado_ativo(void);
bool embarcado_ultimo_resultado(ensaio_resultado *r);   // false se ainda não houve ensaio de controle
void embarcado_ensaio_atual(int *lado, double *duracao);   // ensaio.lado e ensaio.duracao em uso
fase_t embarcado_fase(void);
const char *embarcado_fase_nome(fase_t f);

// Próxima amostra de telemetria (espera até espera_ms). false se não chegou nenhuma.
bool embarcado_telemetria(telemetria_t *t, unsigned espera_ms);

// Durante o furuta -sim: o que a planta simulada mede (θ1, θ2, dθ1, dθ2). false fora da simulação.
bool embarcado_sim_leitura(float x[4]);

#ifdef __cplusplus
}
#endif
