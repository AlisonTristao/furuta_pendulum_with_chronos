# Gravação sem BOOT/RESET (extra_scripts do platformio.ini). Antes do upload:
#   - placa no modo de gravação (USB-Serial-JTAG, 303A:1001): grava nela;
#   - firmware rodando (porta "furuta", 303A:4001): abre a 1200 baud (src/gravacao.cpp reinicia no modo de
#     gravação) e espera até 20 s a porta do ROM aparecer (o Windows demora mais que a espera do PlatformIO).
# O esptool sai com watchdog_reset (board_upload.after_reset), que limpa o pedido de gravação e roda o app.
import time

Import("env")  # noqa: F821  (SCons)

ESPRESSIF = 0x303A
PID_JTAG, PID_APP = 0x1001, 0x4001


def _porta(pid):
    from serial.tools import list_ports
    achadas = [p for p in list_ports.comports() if p.vid == ESPRESSIF and p.pid == pid]
    if not achadas:
        return None
    # USB-Serial-JTAG tem duas interfaces com COM: a serial é a .0 (MI_00)
    achadas.sort(key=lambda p: (not (p.location or "").endswith(".0"), p.device))
    return achadas[0].device


def _antes_do_upload(source, target, env):
    import serial
    jtag = _porta(PID_JTAG)
    if not jtag:
        app = _porta(PID_APP)
        if not app:
            print("grava_s3: nenhuma placa (nem 'furuta' nem USB-JTAG). Segure BOOT, aperte RESET, solte BOOT.")
            env.Exit(1)
        print("grava_s3: %s a 1200 baud -> modo de gravação (feche o TraceView se a porta estiver ocupada)" % app)
        try:
            s = serial.Serial(app, 1200)
            time.sleep(0.1)
            s.close()
        except serial.SerialException as e:
            # a placa reinicia no meio da configuração da porta: o Windows acusa "dispositivo não está
            # funcionando" (erro 31). "Acesso negado" é outro programa (o TraceView) com a porta.
            if "Acesso negado" in str(e) or "Access is denied" in str(e) or "PermissionError(13, 'Acesso" in str(e):
                print("grava_s3: %s está ocupada: feche o TraceView" % app)
                env.Exit(1)
        t0 = time.time()
        while not jtag and time.time() - t0 < 20:
            time.sleep(0.3)
            jtag = _porta(PID_JTAG)
        if not jtag:
            print("grava_s3: a porta do ROM não apareceu em 20 s. Segure BOOT, aperte RESET, solte BOOT.")
            env.Exit(1)
        time.sleep(1.0)   # o driver do Windows termina de subir a porta
    print("grava_s3: gravando em %s" % jtag)
    env.Replace(UPLOAD_PORT=jtag)


env.AddPreAction("upload", _antes_do_upload)  # noqa: F821
