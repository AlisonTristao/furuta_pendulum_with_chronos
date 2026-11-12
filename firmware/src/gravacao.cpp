// Gravação sem os botões BOOT/RESET. O TinyUSB ocupa a USB nativa, então o esptool não alcança o ROM: quando o PC
// abre a porta CDC a 1200 baud (o "toque de 1200" que o PlatformIO faz com board_upload.use_1200bps_touch), o
// firmware devolve a USB ao USB-Serial-JTAG e reinicia direto no modo de gravação do ROM. O furuta -gravar faz o
// mesmo pelo terminal. Depois de gravar, o esptool sai com watchdog_reset (platformio.ini), que limpa o pedido.
#include "gravacao.h"

#include "hal/wdt_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "tinyusb_cdc_acm.h"

namespace {

void reinicia_tarefa(void*) {
    vTaskDelay(pdMS_TO_TICKS(50));   // deixa a resposta do comando / o ACK do USB sair
    gravacao_reinicia();
}

void on_line_coding(int, cdcacm_event_t* ev) {
    if (ev->line_coding_changed_data.p_line_coding->bit_rate == 1200)
        xTaskCreate(reinicia_tarefa, "gravacao", 2048, nullptr, 20, nullptr);
}

}  // namespace

void gravacao_reinicia() {
    // USB de volta ao USB-Serial-JTAG (o ROM grava por ele) e boot forçado no modo de gravação
    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL |
                                                   RTC_CNTL_USB_PAD_ENABLE);
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    // Reset de sistema pelo watchdog do RTC, como o esptool --after watchdog_reset: o esp_restart() (reset de
    // software) às vezes voltava para o app em vez do modo de gravação.
    wdt_hal_context_t rtc = RWDT_HAL_CONTEXT_DEFAULT();
    wdt_hal_write_protect_disable(&rtc);
    wdt_hal_config_stage(&rtc, WDT_STAGE0, 2000, WDT_STAGE_ACTION_RESET_SYSTEM);   // ~15 ms (relógio lento)
    wdt_hal_enable(&rtc);
    wdt_hal_write_protect_enable(&rtc);
    for (;;) {
    }
}

void gravacao_reinicia_depois() { xTaskCreate(reinicia_tarefa, "gravacao", 2048, nullptr, 20, nullptr); }

void gravacao_init() {
    tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_CODING_CHANGED, &on_line_coding);
}
