/* Laço periódico com horário absoluto (sem acumular atraso): t_k = t_0 + k·T.
 * Windows: timer de alta resolução (Windows 10 1803+) até ~0,5 ms antes do instante e espera ativa no resto,
 * para o jitter ficar na casa de dezenas de µs em vez dos 1-15 ms do Sleep. */
#ifndef PERIODO_H
#define PERIODO_H

typedef struct periodo periodo;

periodo *periodo_cria(double T_s);
void periodo_libera(periodo *p);
/* Espera até o próximo instante. Devolve o atraso [s] em relação ao instante (>= 0). Se um passo estourar
 * o período inteiro, os instantes perdidos são pulados (não tenta "alcançar"). */
double periodo_espera(periodo *p);
/* Relógio monotônico [s]. */
double periodo_agora(void);
/* Prioridade alta para o processo e a thread atual (0 se conseguiu). */
int periodo_prioridade_alta(void);

#endif
