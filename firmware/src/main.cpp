// Pêndulo de Furuta no ESP32-S3: app_main e a comunicação com o TraceView, um dispositivo BTP "furuta" pela USB
// nativa (Serial, sem criptografia). O núcleo em C (malha, motor, encoder, MPC) está em components/furuta.
//
// Terminal (help lista tudo):
//   furuta -home | -origem | -zera | -driver <0|1> | -status   malha interna (controle.c)
//   furuta -start | -dry | -stop | -bench                      ensaio (embarcado.c): controle, a seco, para, benchmark
//   furuta -auto                                               zera + homing + calibração + lado pelo offset + start
//   furuta -inclina                                            varredura do braço: inclinação da mesa (inclina.c)
//   furuta -sim                                                ensaio simulado em tempo real, com telemetria (sem motor)
//   furuta -ident <l2>                                         oscilação livre -> I2, mu2 (ident.c; era o ident.exe)
//   furuta -calibra                                            zero do θ2 pelo centro da oscilação livre (ident.c)
//   cfg -set <secao.chave=valor> | -show | -reset             configuração (chaves do pc/furuta.ini; listas com
//                                                              espaço: cfg -set mpc.Q=10 100 5 1)
//   pinos -entradas | -i2c | -motor                            teste de bancada (pinos.c)
// Tópicos:
//   controle  a cada período do MPC durante o ensaio (evento): t, θ1, θ2 ([-π, π), 0 = topo), dθ1, dθ2 (estimado), u, a, n_qp, n_it,
//             t_calc, estouros
//   sensores  leitura da malha de 1 kHz, até 125 Hz (também fora do ensaio)
//   maquina   texto: estado da malha, erro, fase do ensaio, chaves, nFAULT, encoder
//
// Tarefas: comms (prioridade 10) e shell da BTPDevice no núcleo 0, abaixo da malha de 1 kHz (configMAX - 2); a
// telemetria (prioridade 4) também no núcleo 0. O núcleo 1 fica só com o MPC.
#include <BtpDevice.h>

#include <cmath>

extern "C" {
#include "controle.h"
#include "embarcado.h"
#include "encoder.h"
#include "ident.h"
#include "inclina.h"
#include "pinos.h"
#include "saida.h"
#include "stepper.h"
#include "termopar.h"
}

#include "gravacao.h"

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr float kPi = 3.14159265358979f;

// Faixas dos eixos no TraceView (.f32(nome, unidade, min, max)): os limites do sistema com 20% de folga
constexpr double kFolga = 1.2;
constexpr double kTheta1 = 1.8 * kFolga;    // curso do braço: meio curso medido 1,71 rad
constexpr double kVel = 25 * kFolga;     // V_MAX da malha (dθ1) [rad/s]
constexpr double kVel2 = 40 * kFolga;    // ensaio.dtheta2_aborta [rad/s]
constexpr double kU = 2.5 * kFolga;      // torque virtual: mpc.u_max padrão 2,5 [N·m]
constexpr double kAcel = 300 * kFolga;   // A_MAX da malha [rad/s²]
constexpr double kAng = kPi * kFolga;    // θ2 em [-π, π)
constexpr double kAngHist = 1.5 * kPi * kFolga;   // θ2 com histerese em [-3π/2, 3π/2]

float wrap(float a) { return a - 2.0f * kPi * std::floor((a + kPi) / (2.0f * kPi)); }

// θ2 do envio ao TraceView: tira voltas inteiras só quando passa de ±3π/2 (deitado, depois de uma volta a mais).
// O corte seco em ±π cai bem embaixo, onde o pêndulo passa o tempo todo; com a histerese ele não salta nem embaixo
// (±π) nem no topo (0), só quando gira de verdade. Só no envio: a malha continua com o contínuo.
float wrap_histerese(float a) {
    static float voltas = 0.0f;   // múltiplo de 2π já descontado (só a tarefa comms chama)
    float y = a - voltas;
    while (y > 1.5f * kPi) voltas += 2.0f * kPi, y -= 2.0f * kPi;
    while (y < -1.5f * kPi) voltas -= 2.0f * kPi, y += 2.0f * kPi;
    return y;
}

class Furuta : public btpdev::App {
public:
    btpdev::DeviceConfig config() const override {
        return {
            .name = "furuta",
            .link = btpdev::Link::Serial,
            .memory = btpdev::Memory::Medium,
            .comms = {.core = 0, .priority = 10U, .period_ms = 1U},
            .shell = {.core = 0},
            .serial_cleartext = true,
        };
    }
    btpdev::StorageConfig storage() const override { return {.key = btpdev::KeySource::None}; }

    void commands(btpdev::Commands& c) override {
        c.module("furuta", "Pêndulo: malha interna e ensaio")
            .add("home", [] { return home(); }, "furuta -home: homing pelos fins de curso (liga o driver)")
            .add("origem", [] { return origem(); }, "furuta -origem: θ1 = 0 aqui, sem homing (limite ±0,5 rad)")
            .add("zera", [] {
                controle_zera_pendulo();
                btpdev::printf("θ2 = π (pêndulo parado embaixo)\n");
                return uint8_t{RESULT_OK};
            }, "furuta -zera: pêndulo parado embaixo, θ2 = π")
            .add("driver", [](uint8_t on) { return driver(on); }, "furuta -driver <0|1>: desliga/liga o DRV8825")
            .add("status", [] { return status(); }, "furuta -status: estado, chaves, encoder, ensaio")
            .add("start", [] { return liga(1); }, "furuta -start: controle (após o homing; espera a partida)")
            .add("auto", [] { return automatico(); },
                 "furuta -auto: zera (pêndulo parado embaixo), homing, zero pela oscilação (inclinacao.pontos), lado "
                 "do swing-up pelo sinal do offset do topo (ensaio.lado_auto) e start")
            .add("lote", [](uint8_t n, uint8_t dur) { return lote(n, dur); },
                 "furuta -lote <n>, <s>: n subidas de s segundos alternando o lado (+π, −π, ...); no log, uma linha por "
                 "subida e o resumo por lado. -stop interrompe")
            .add("inclina", [] { return inclina(true); },
                 "furuta -inclina: varre o braço e mede a inclinação da mesa, gravada na flash (após o homing, pêndulo "
                 "pendurado; ~1 min: deixe o motor esfriar antes do start)")
            .add("dry", [] { return liga(2); }, "furuta -dry: a seco (calcula com os sensores, sem motor)")
            .add("stop", [] {
                s_lote_aborta = true;
                embarcado_desliga();
                return uint8_t{RESULT_OK};
            }, "furuta -stop: para o ensaio")
            .add("bench", [] { return liga(3); }, "furuta -bench: tempo do MPC com a config em uso (planta simulada)")
            .add("sim", [] { return liga(4); },
                 "furuta -sim: ensaio simulado em tempo real (planta da seção [sim]), com telemetria; sem motor")
            .add("gravar", [] {
                if (embarcado_ativo()) return err("tem um ensaio rodando");
                btpdev::printf("reiniciando no modo de gravação: feche a porta e rode pio run -t upload\n");
                gravacao_reinicia_depois();
                return uint8_t{RESULT_OK};
            }, "furuta -gravar: reinicia no modo de gravação (sem BOOT/RESET)")
            .add("calibra", [] {
                const char* e = ident_calibra();
                return e ? err(e) : uint8_t{RESULT_OK};
            }, "furuta -calibra: zero do θ2 pelo centro da oscilação livre (solte de 30-60°; até ~50 s)")
            .add("ident", [](float l2) {
                const char* e = ident_roda(l2);
                return e ? err(e) : uint8_t{RESULT_OK};
            }, "furuta -ident <l2 medido [m]>: oscilação livre com o braço travado -> I2 e mu2 (até ~50 s)");
        c.module("cfg", "Configuração do controle (chaves do pc/furuta.ini)")
            .add("set", [](const char* par) { return set(par); },
                 "cfg -set <secao.chave=valor>: ex. mpc.Np=8; listas com espaço: mpc.Q=10 100 5 1")
            .add("show", [] { return show(); }, "cfg -show: configuração em uso")
            .add("reset", [] {
                if (embarcado_ativo()) return err("tem um ensaio rodando");
                embarcado_config_padrao();
                return show();
            }, "cfg -reset: volta ao pc/furuta.ini embutido");
        c.module("pinos", "Teste de bancada dos pinos")
            .add("entradas", [] {
                pinos_entradas();
                return uint8_t{RESULT_OK};
            }, "pinos -entradas: níveis crus das chaves e do nFAULT")
            .add("i2c", [] {
                pinos_i2c();
                return uint8_t{RESULT_OK};
            }, "pinos -i2c: scan do barramento e encoder detectado")
            .add("motor", [] {
                const char* e = pinos_motor();
                return e ? err(e) : uint8_t{RESULT_OK};
            }, "pinos -motor: 1/8 de volta em cada micropasso (só em IDLE)");
    }

    void topics(btpdev::Topics& t) override {
        controle_ = t.topic("controle", btpdev::OnEvent)
                        .f32("t", "s")
                        .f32("theta1", "rad", -kTheta1, kTheta1)
                        .f32("theta2", "rad", -kAng, kAng)   // em [-π, π): 0 = topo
                        .f32("dtheta1", "rad/s", -kVel, kVel)
                        .f32("dtheta2", "rad/s", -kVel2, kVel2)
                        .f32("u", "N·m", -kU, kU)
                        .f32("a", "rad/s²", -kAcel, kAcel)
                        .u8("n_qp")
                        .range(0, 10 * kFolga)
                        .u8("n_it")
                        .range(0, 100 * kFolga)
                        .f32("t_calc", "ms", 0, 20 * kFolga)
                        .u16("estouros")
                        .f32("trim", "rad", -0.1, 0.1)   // trim do zero do θ2 (seção [trim] do furuta.ini)
                        .fill([this](btpdev::Sample& s) {
                            telemetria_t tl;
                            taskENTER_CRITICAL(&mux_);
                            tl = tel_;
                            taskEXIT_CRITICAL(&mux_);
                            s.put("t", tl.t)
                                .put("theta1", tl.x[0])
                                .put("theta2", wrap(tl.x[1]))
                                .put("dtheta1", tl.x[2])
                                .put("dtheta2", tl.x[3])
                                .put("u", tl.u)
                                .put("a", tl.a)
                                .put("n_qp", tl.n_qp)
                                .put("n_it", tl.n_it)
                                .put("t_calc", tl.t_calc_ms)
                                .put("estouros", tl.estouros)
                                .put("trim", tl.trim);
                        });
        t.topic("sensores", btpdev::Hz(125))   // 8 ms: período inteiro no agendador de 1 ms
            .f32("theta1", "rad", -kTheta1, kTheta1)
            .f32("theta2", "rad", -kAngHist, kAngHist)     // só no envio: voltas tiradas com histerese (wrap_histerese)
            .f32("theta2_topo", "rad", -kAng, kAng)        // θ2 em [-π, π): 0 = topo
            .f32("dtheta1", "rad/s", -kVel, kVel)
            .f32("dtheta2", "rad/s", -kVel2, kVel2)
            .fill([](btpdev::Sample& s) {
                leitura_t x;
                float xs[4];
                if (embarcado_sim_leitura(xs)) {   // furuta -sim: o que a planta simulada mede
                    x.t1 = xs[0], x.t2 = xs[1], x.dt1 = xs[2], x.dt2 = xs[3];
                } else {
                    controle_leitura(&x);
                }
                s.put("theta1", x.t1)
                    .put("theta2", wrap_histerese(x.t2))
                    .put("theta2_topo", wrap(x.t2))
                    .put("dtheta1", x.dt1)
                    .put("dtheta2", x.dt2);
            });
        t.topic("temperatura", btpdev::Hz(2))   // termopar do driver (MAX6675; termopar.c): NAN sem leitura
            .f32("driver", "°C", 0, 120)
            .fill([](btpdev::Sample& s) { s.put("driver", termopar_temp()); });
        t.text("maquina", btpdev::Hz(2), [](btpdev::Text& out) {
            estado_t e = controle_estado();
            out.printf("malha %s%s%s | ensaio %s | fim_neg %d fim_pos %d nFAULT %d | encoder %s | limite θ1 %.3f rad | "
                       "driver %.1f °C (%s)",
                       controle_estado_nome(e), controle_erro()[0] ? " - " : "", controle_erro(),
                       embarcado_fase_nome(embarcado_fase()), controle_fim_neg(), controle_fim_pos(),
                       controle_falha_driver(), encoder_nome(), (double)controle_limite(), (double)termopar_temp(),
                       termopar_estado());
        });
    }

    // Tarefa da telemetria: cada amostra do MPC vira um evento do tópico controle. A comms (prioridade maior, mesmo
    // núcleo) acorda no publish() e chama o fill() na hora, antes da próxima amostra (20 ms depois).
    void publica_telemetria() {
        telemetria_t tl;
        for (;;) {
            if (!embarcado_telemetria(&tl, 1000)) continue;
            taskENTER_CRITICAL(&mux_);
            tel_ = tl;
            taskEXIT_CRITICAL(&mux_);
            device().publish(controle_);
        }
    }

private:
    // Também no painel de logs: um comando que passa de 5 s (homing, -inclina, -auto) tem a resposta descartada
    // pelo TraceView (btp::kCommandTimeoutMs), e o log chega de qualquer jeito.
    static uint8_t err(const char* motivo) {
        btpdev::log(btpdev::LogLevel::Error, "erro: %s", motivo);
        btpdev::printf("erro: %s\n", motivo);
        return RESULT_ERROR;
    }

    static uint8_t curso() {
        saida_aviso("pronto: meio curso %.4f rad, limite de software ±%.4f rad", (double)controle_meio_curso(),
                    (double)controle_limite());
        return RESULT_OK;
    }

    static uint8_t home() {
        if (embarcado_ativo()) return err("tem um ensaio rodando");
        if (!controle_homing()) return err("a malha está rodando");
        while (controle_estado() == EST_HOMING) vTaskDelay(pdMS_TO_TICKS(20));
        if (controle_estado() != EST_PRONTO) return err(controle_erro());
        int32_t perdidos;
        bool solto;
        if (controle_desvio_homing(&perdidos, &solto))   // ±2-3 passos é a repetibilidade das chaves
            saida_aviso("passos perdidos desde o último homing: %+ld (%+.2f°)%s", (long)perdidos,
                        perdidos / (double)PASSOS_POR_RAD * 180 / 3.14159265358979,
                        solto ? " - o driver desligou no meio: o braço pode ter sido mexido" : "");
        return curso();
    }

    static uint8_t origem() {
        if (embarcado_ativo()) return err("tem um ensaio rodando");
        controle_origem_aqui();
        return controle_estado() == EST_PRONTO ? curso() : err(controle_estado_nome(controle_estado()));
    }

    static uint8_t driver(uint8_t on) {
        if (embarcado_ativo()) return err("tem um ensaio rodando");   // não desliga o driver no meio do controle
        controle_habilita(on != 0U);
        btpdev::printf("driver %s (refaça o homing)\n", on ? "ligado" : "desligado");
        return RESULT_OK;
    }

    static uint8_t status() {
        leitura_t x;
        controle_leitura(&x);
        btpdev::printf("malha %s %s | ensaio %s | fim_neg %d fim_pos %d nFAULT %d | encoder %s\n"
                       "θ1 %.4f θ2 %.4f dθ1 %.3f dθ2 %.3f | meio curso %.4f limite %.4f\n"
                       "driver %.2f °C (termopar: %s)\n",
                       controle_estado_nome(controle_estado()), controle_erro(), embarcado_fase_nome(embarcado_fase()),
                       controle_fim_neg(), controle_fim_pos(), controle_falha_driver(), encoder_nome(), (double)x.t1,
                       (double)x.t2, (double)x.dt1, (double)x.dt2, (double)controle_meio_curso(),
                       (double)controle_limite(), (double)termopar_temp(), termopar_estado());
        return RESULT_OK;
    }

    static uint8_t liga(int modo) {
        const char* e = embarcado_liga(modo);
        if (e) return err(e);
        saida_aviso("%s", modo == 1 ? "controle ligado: esperando a condição de partida"
                               : modo == 2 ? "a seco ligado"
                               : modo == 3 ? "bench rodando no núcleo 1"
                                           : "simulação ligada (furuta -stop para)");
        return RESULT_OK;
    }

    static uint8_t inclina(bool varredura) {
        const char* e = inclina_roda(varredura);
        return e ? err(e) : RESULT_OK;
    }

    // Zera o θ2 (pêndulo parado embaixo), homing, calibração embaixo (inclinacao.pontos: 1 = só o zero em θ1 = 0,
    // >= 4 = varredura da mesa), lado do swing-up e start. O lado vem do sinal do offset do topo (ensaio.lado_auto ·
    // sinal(trim.valor)); com lado_auto = 0 ou offset zero, é sorteado (esp_random: RNG de hardware). Fica na config
    // em uso, como um cfg -set ensaio.lado: o cfg -show mostra o escolhido.
    // O zera é uma leitura só: espera o pêndulo parar (θ2 variando menos de 0,01 rad por 1 s, até 20 s), senão
    // logo depois de um -stop ele pega o pêndulo balançando (14/11: ±6,8°). Pela posição e não pelo dθ2: parado
    // na divisa entre duas contagens do encoder, a leitura alterna 1 LSB e o observador dá picos de ~0,8 rad/s.
    static void espera_e_zera(const char* quem) {
        int quieto_ms = 0, espera_ms = 0;
        float th_min = 0, th_max = 0;
        for (; quieto_ms < 1000 && espera_ms < 20000; espera_ms += 20) {
            leitura_t x;
            controle_leitura(&x);
            if (quieto_ms == 0) th_min = th_max = x.t2;
            th_min = std::fmin(th_min, x.t2), th_max = std::fmax(th_max, x.t2);
            quieto_ms = th_max - th_min < 0.01f ? quieto_ms + 20 : 0;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        controle_zera_pendulo();
        saida_aviso("%s: θ2 = π (pêndulo %s embaixo, %.1f s esperando); homing...", quem,
                    quieto_ms >= 1000 ? "parado" : "AINDA BALANÇANDO", espera_ms * 1e-3);
    }

    static uint8_t automatico() {
        if (embarcado_ativo()) return err("tem um ensaio rodando");
        espera_e_zera("-auto");
        if (home() != RESULT_OK) return RESULT_ERROR;   // home() já imprimiu o motivo
        inclinacao_params ip;
        embarcado_inclinacao_params(&ip);
        if (ip.pontos > 0 && inclina(false) != RESULT_OK) return RESULT_ERROR;
        int lado = embarcado_lado_auto();
        const bool sorteado = lado == 0;
        if (sorteado) lado = (esp_random() & 1U) ? 1 : -1;
        if (!embarcado_config(lado > 0 ? "ensaio.lado=1" : "ensaio.lado=-1")) return err("não deu para mudar o lado");
        const double off = embarcado_offset_topo();
        saida_aviso("-auto: lado %+d (sobe por %s) %s; offset do topo %+.4f rad (%+.2f°); start", lado,
                    lado > 0 ? "+π" : "−π", sorteado ? "sorteado" : "pelo sinal do offset (ensaio.lado_auto)", off,
                    off * 180 / 3.14159265358979);
        return liga(1);
    }

    // furuta -lote: série de subidas alternando o lado, numa tarefa própria (a do shell fica livre para o -stop).
    // Uma vez: espera parar, zera, homing e o zero pela oscilação (inclinacao.pontos). Cada rodada: homing (o driver
    // desliga no fim de cada ensaio) e o ensaio, que só parte com o pêndulo parado embaixo: no fim da duração ele
    // cai e a rodada seguinte espera estabilizar. Restaura ensaio.lado e ensaio.duracao no fim.
    static inline volatile bool s_lote_ativo = false, s_lote_aborta = false;
    static inline int s_lote_n = 0, s_lote_dur = 0;

    static uint8_t lote(uint8_t n, uint8_t dur) {
        if (embarcado_ativo() || s_lote_ativo) return err("tem um ensaio ou lote rodando");
        if (n < 1 || n > 40) return err("n entre 1 e 40");
        if (dur < 5 || dur > 60) return err("duração entre 5 e 60 s");
        s_lote_n = n;
        s_lote_dur = dur;
        s_lote_aborta = false;
        s_lote_ativo = true;
        if (xTaskCreatePinnedToCore(tarefa_lote, "lote", 6144, nullptr, 5, nullptr, 0) != pdPASS) {
            s_lote_ativo = false;
            return err("não deu para criar a tarefa do lote");
        }
        btpdev::printf("lote: %d subidas de %d s alternando o lado; acompanhe no log (-stop interrompe)\n", n, dur);
        return RESULT_OK;
    }

    static void tarefa_lote(void*) {
        int lado0;
        double dur0;
        embarcado_ensaio_atual(&lado0, &dur0);
        char par[40];
        snprintf(par, sizeof(par), "ensaio.duracao=%d", s_lote_dur);
        embarcado_config(par);
        const int n = s_lote_n;
        int feitas[2] = {0, 0}, limpas[2] = {0, 0}, quedas[2] = {0, 0}, subiram[2] = {0, 0};
        double soma_topo[2] = {0, 0};
        espera_e_zera("lote");
        bool ok = home() == RESULT_OK;
        inclinacao_params ip;
        embarcado_inclinacao_params(&ip);
        if (ok && ip.pontos > 0) ok = inclina(false) == RESULT_OK;
        for (int i = 0; ok && i < n && !s_lote_aborta; i++) {
            const int lado = (i % 2 == 0) ? 1 : -1;
            embarcado_config(lado > 0 ? "ensaio.lado=1" : "ensaio.lado=-1");
            if (i > 0 && home() != RESULT_OK) break;
            if (s_lote_aborta) break;
            saida_aviso("lote %d/%d: sobe por %s", i + 1, n, lado > 0 ? "+π" : "−π");
            if (embarcado_liga(1)) break;
            for (int w = 0; w < 50 && !embarcado_ativo(); w++) vTaskDelay(pdMS_TO_TICKS(20));
            while (embarcado_ativo()) {
                if (s_lote_aborta) embarcado_desliga();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            ensaio_resultado r;
            if (!embarcado_ultimo_resultado(&r)) continue;
            const int j = lado > 0 ? 0 : 1;
            feitas[j]++;
            quedas[j] += r.quedas;
            if (r.t_topo >= 0) subiram[j]++, soma_topo[j] += r.t_topo;
            const bool limpa = r.t_topo >= 0 && r.t_topo < 1.0 && r.quedas == 0;
            limpas[j] += limpa;
            saida_aviso("lote %d/%d %s: %s, topo em %.2f s, %d quedas, %d%% no topo, driver %.1f -> %.1f °C (%s)", i + 1,
                        n, lado > 0 ? "+π" : "−π", limpa ? "LIMPA" : r.partiu ? "ruim" : "não partiu", r.t_topo,
                        r.quedas, r.pct_topo, r.temp_ini, r.temp_fim, r.motivo);
        }
        for (int j = 0; j < 2; j++)
            saida_aviso("lote, lado %s: %d/%d limpas (topo < 1 s e 0 quedas), topo médio %.2f s, %d quedas no total",
                        j == 0 ? "+π" : "−π", limpas[j], feitas[j], subiram[j] ? soma_topo[j] / subiram[j] : -1.0,
                        quedas[j]);
        snprintf(par, sizeof(par), "ensaio.lado=%d", lado0);
        embarcado_config(par);
        snprintf(par, sizeof(par), "ensaio.duracao=%g", dur0);
        embarcado_config(par);
        saida_aviso("lote %s", s_lote_aborta ? "interrompido (-stop)" : "terminado");
        s_lote_ativo = false;
        vTaskDelete(nullptr);
    }

    static uint8_t set(const char* par) {
        if (embarcado_ativo()) return err("tem um ensaio rodando");
        if (!embarcado_config(par)) return err("chave desconhecida ou valor inválido (use secao.chave=valor)");
        btpdev::printf("ok: %s\n", par);
        return RESULT_OK;
    }

    static uint8_t show() {
        char buf[600];
        embarcado_config_texto(buf, sizeof(buf));
        btpdev::printf("%s\n", buf);
        return RESULT_OK;
    }

    btpdev::Topic controle_;
    telemetria_t tel_ = {};
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

Furuta app;
BtpDevice device(app);

void tarefa_telemetria(void*) { app.publica_telemetria(); }

// Texto do núcleo em C (saida.h): terminal, e os avisos também no painel de logs
void saida_btp(int aviso, const char* texto) {
    if (aviso) {
        btpdev::log(btpdev::LogLevel::Info, "%s", texto);
        btpdev::printf("%s\n", texto);
    } else {
        btpdev::printf("%s", texto);
    }
}

}  // namespace

extern "C" void app_main(void) {
    static const char* TAG = "furuta";
#ifdef FURUTA_DEBUG_BOOT   // esp32s3_debug: o console (USB-JTAG) fica visível antes de o TinyUSB tomar a USB
    for (int i = 4; i > 0; i--) {
        ESP_LOGI(TAG, "boot: BTP em %d s", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    ESP_LOGI(TAG, "controle_init");
    controle_init();   // malha de 1 kHz e ISR dos passos no núcleo 0
    termopar_init();   // temperatura do driver (MAX6675), tarefa de 4 Hz no núcleo 0
    ESP_LOGI(TAG, "embarcado_init");
    embarcado_init();  // configuração = pc/furuta.ini embutido; tarefa do MPC no núcleo 1
    saida_registra(saida_btp);
    ESP_LOGI(TAG, "device.begin (a USB passa para o TinyUSB)");
    device.begin();    // falha: a causa fica em device.last_error() (sem USB não há como mostrar)
    gravacao_init();
    embarcado_inclinacao_carrega();   // a mesa medida pelo -inclina (NVS: o BTPDevice inicializa no begin)   // toque de 1200 baud -> modo de gravação (pio run -t upload sem os botões)
    xTaskCreatePinnedToCore(tarefa_telemetria, "telemetria", 4096, nullptr, 4, nullptr, 0);
}
