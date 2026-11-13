# Pêndulo de Furuta com CHRONOS

NMPC qLPV (CHRONOS) para um pêndulo de Furuta com motor de passo, rodando num ESP32-S3. O mesmo controlador em C
(`comum/`) roda no simulador do PC e no firmware.

```
comum/      controlador (CHRONOS, qlpv_mpc, modelo, estimador, config), compartilhado PC/firmware
pc/         simulação da malha fechada (sim.exe) e a configuração padrão furuta.ini
firmware/   ESP32-S3 (ESP-IDF via PlatformIO); comunicação BTP com o TraceView pela USB nativa
  extern/   submódulos: BTPDevice, BTP, TinyShell
```

## Clonar

As bibliotecas de comunicação são submódulos git, fixados no commit testado com este firmware:

```
git clone --recursive https://github.com/AlisonTristao/furuta_pendulum_with_chronos.git
```

Se você já clonou sem `--recursive`, rode `git submodule update --init` na raiz do repositório.
Depois de um `git pull` que atualize os submódulos, rode o mesmo comando de novo.

## Firmware (ESP32-S3 N16R8)

Requer o [PlatformIO](https://platformio.org/). Ele baixa sozinho a plataforma `espressif32 @ 7.0.1` (ESP-IDF 6.0.1)
e as dependências do `dependencies.lock`.

```
cd firmware
pio run                 # compila
pio run -t upload       # grava: segure BOOT, aperte RESET, solte BOOT antes
```

O `pc/furuta.ini` é embutido no firmware como configuração padrão. Conecte o TraceView na porta serial "furuta" e
digite `help` no terminal para ver os comandos (`furuta -home`, `furuta -start`, `furuta -bench`, `cfg -set ...`).

## Simulador (PC)

Requer gcc e make (MinGW-w64 no Windows: `mingw32-make`).

```
cd pc
make                    # sim.exe (double) e sim_f.exe (float, como no ESP32)
./sim.exe furuta.ini
```
