// Pêndulo de Furuta no ESP32-S3: app_main e a comunicação com o TraceView, um dispositivo BTP "furuta" pela USB
// nativa (Serial, sem criptografia). O núcleo em C (malha, motor, encoder, MPC) está em components/furuta.
//
// Terminal (help lista tudo):
//   furuta -home | -origem | -zera | -driver <0|1> | -status   malha interna (controle.c)
//   furuta -start | -dry | -stop | -bench                      ensaio (embarcado.c): controle, a seco, para, benchmark
//   furuta -ident <l2>                                         oscilação livre -> I2, mu2 (ident.c; era o ident.exe)
//   cfg -set <secao.chave=valor> | -show | -reset             configuração (chaves do pc/furuta.ini; listas com
//                                                              espaço: cfg -set mpc.Q=10 100 5 1)
//   pinos -entradas | -i2c | -motor                            teste de bancada (pinos.c)
// Tópicos:
//   controle  a cada período do MPC durante o ensaio (evento): t, θ1, θ2, dθ1, dθ2 (estimado), u, a, n_qp, n_it,
//             t_calc, estouros
//   sensores  leitura da malha de 1 kHz, até 50 Hz (também fora do ensaio)
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
#include "pinos.h"
#include "saida.h"
}

#include "gravacao.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr float kPi = 3.14159265358979f;

float wrap(float a) { return a - 2.0f * kPi * std::floor((a + kPi) / (2.0f * kPi)); }

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
            .add("dry", [] { return liga(2); }, "furuta -dry: a seco (calcula com os sensores, sem motor)")
            .add("stop", [] {
                embarcado_desliga();
                return uint8_t{RESULT_OK};
            }, "furuta -stop: para o ensaio")
            .add("bench", [] { return liga(3); }, "furuta -bench: tempo do MPC com a config em uso (planta simulada)")
            .add("gravar", [] {
                if (embarcado_ativo()) return err("tem um ensaio rodando");
                btpdev::printf("reiniciando no modo de gravação: feche a porta e rode pio run -t upload\n");
                gravacao_reinicia_depois();
                return uint8_t{RESULT_OK};
            }, "furuta -gravar: reinicia no modo de gravação (sem BOOT/RESET)")
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
                        .f32("theta1", "rad")
                        .f32("theta2", "rad")
                        .f32("dtheta1", "rad/s")
                        .f32("dtheta2", "rad/s")
                        .f32("u", "N·m")
                        .f32("a", "rad/s²")
                        .u8("n_qp")
                        .u8("n_it")
                        .f32("t_calc", "ms")
                        .u16("estouros")
                        .fill([this](btpdev::Sample& s) {
                            telemetria_t tl;
                            taskENTER_CRITICAL(&mux_);
                            tl = tel_;
                            taskEXIT_CRITICAL(&mux_);
                            s.put("t", tl.t)
                                .put("theta1", tl.x[0])
                                .put("theta2", tl.x[1])
                                .put("dtheta1", tl.x[2])
                                .put("dtheta2", tl.x[3])
                                .put("u", tl.u)
                                .put("a", tl.a)
                                .put("n_qp", tl.n_qp)
                                .put("n_it", tl.n_it)
                                .put("t_calc", tl.t_calc_ms)
                                .put("estouros", tl.estouros);
                        });
        t.topic("sensores", btpdev::Hz(50))
            .f32("theta1", "rad")
            .f32("theta2", "rad")
            .f32("theta2_topo", "rad")   // θ2 em [-π, π): 0 = topo
            .f32("dtheta1", "rad/s")
            .f32("dtheta2", "rad/s")
            .fill([](btpdev::Sample& s) {
                leitura_t x;
                controle_leitura(&x);
                s.put("theta1", x.t1)
                    .put("theta2", x.t2)
                    .put("theta2_topo", wrap(x.t2))
                    .put("dtheta1", x.dt1)
                    .put("dtheta2", x.dt2);
            });
        t.text("maquina", btpdev::Hz(2), [](btpdev::Text& out) {
            estado_t e = controle_estado();
            out.printf("malha %s%s%s | ensaio %s | fim_neg %d fim_pos %d nFAULT %d | encoder %s | limite θ1 %.3f rad",
                       controle_estado_nome(e), controle_erro()[0] ? " - " : "", controle_erro(),
                       embarcado_fase_nome(embarcado_fase()), controle_fim_neg(), controle_fim_pos(),
                       controle_falha_driver(), encoder_nome(), (double)controle_limite());
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
    static uint8_t err(const char* motivo) {
        btpdev::printf("erro: %s\n", motivo);
        return RESULT_ERROR;
    }

    static uint8_t curso() {
        btpdev::printf("pronto: meio curso %.4f rad, limite de software ±%.4f rad\n", (double)controle_meio_curso(),
                       (double)controle_limite());
        return RESULT_OK;
    }

    static uint8_t home() {
        if (embarcado_ativo()) return err("tem um ensaio rodando");
        if (!controle_homing()) return err("a malha está rodando");
        while (controle_estado() == EST_HOMING) vTaskDelay(pdMS_TO_TICKS(20));
        return controle_estado() == EST_PRONTO ? curso() : err(controle_erro());
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
                       "θ1 %.4f θ2 %.4f dθ1 %.3f dθ2 %.3f | meio curso %.4f limite %.4f\n",
                       controle_estado_nome(controle_estado()), controle_erro(), embarcado_fase_nome(embarcado_fase()),
                       controle_fim_neg(), controle_fim_pos(), controle_falha_driver(), encoder_nome(), (double)x.t1,
                       (double)x.t2, (double)x.dt1, (double)x.dt2, (double)controle_meio_curso(),
                       (double)controle_limite());
        return RESULT_OK;
    }

    static uint8_t liga(int modo) {
        const char* e = embarcado_liga(modo);
        if (e) return err(e);
        btpdev::printf("%s\n", modo == 1 ? "controle ligado: esperando a condição de partida"
                               : modo == 2 ? "a seco ligado" : "bench rodando no núcleo 1");
        return RESULT_OK;
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
    ESP_LOGI(TAG, "embarcado_init");
    embarcado_init();  // configuração = pc/furuta.ini embutido; tarefa do MPC no núcleo 1
    saida_registra(saida_btp);
    ESP_LOGI(TAG, "device.begin (a USB passa para o TinyUSB)");
    device.begin();    // falha: a causa fica em device.last_error() (sem USB não há como mostrar)
    gravacao_init();   // toque de 1200 baud -> modo de gravação (pio run -t upload sem os botões)
    xTaskCreatePinnedToCore(tarefa_telemetria, "telemetria", 4096, nullptr, 4, nullptr, 0);
}
