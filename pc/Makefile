# mingw32-make (Windows, MinGW-w64) ou make (Linux)
# O controlador (CHRONOS, qlpv_mpc, modelo, estimador) fica em ../comum, compartilhado com o firmware.
# O ensaio real é pelo TraceView (BTP, firmware/src/main.cpp); aqui fica só a simulação.
CC      = gcc
CFLAGS  = -O2 -std=c99 -Wall -Wextra -I../comum
LDLIBS  = -lm
ifeq ($(OS),Windows_NT)
LDLIBS += -lwinmm
endif

COMUM  = $(addprefix ../comum/,estimador.c qlpv_mpc.c modelo.c chronos.c config.c controlador.c planta.c)
NUCLEO = $(COMUM) periodo.c
HDRS   = *.h ../comum/*.h

all: sim.exe sim_f.exe

sim.exe: sim.c $(NUCLEO) $(HDRS)
	$(CC) $(CFLAGS) -o $@ sim.c $(NUCLEO) $(LDLIBS)

# simulação com o controlador em float, como no ESP32 (planta e o resto continuam em double)
OBJF = $(patsubst ../comum/%.c,obj_f/%.o,$(COMUM))
obj_f/%.o: ../comum/%.c $(HDRS)
	@mkdir -p obj_f
	$(CC) $(CFLAGS) -DREAL_FLOAT -DCHRONOS_FIXO -Werror=double-promotion -c -o $@ $<

sim_f.exe: sim.c periodo.c $(OBJF) $(HDRS)
	$(CC) $(CFLAGS) -DREAL_FLOAT -o $@ sim.c periodo.c $(OBJF) $(LDLIBS)

clean:
	rm -rf *.exe *.csv obj_f
