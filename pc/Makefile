# mingw32-make (Windows, MinGW-w64) ou make (Linux)
CC      = gcc
CFLAGS  = -O2 -std=c99 -Wall -Wextra
LDLIBS  = -lm
ifeq ($(OS),Windows_NT)
LDLIBS += -lwinmm
endif

NUCLEO = estimador.c qlpv_mpc.c modelo.c chronos.c config.c periodo.c

all: sim.exe malha.exe ident.exe

sim.exe: sim.c $(NUCLEO) *.h
	$(CC) $(CFLAGS) -o $@ sim.c $(NUCLEO) $(LDLIBS)

malha.exe: malha.c furuta_serial.c serial.c $(NUCLEO) *.h
	$(CC) $(CFLAGS) -o $@ malha.c furuta_serial.c serial.c $(NUCLEO) $(LDLIBS)

ident.exe: ident.c furuta_serial.c serial.c config.c modelo.c qlpv_mpc.c chronos.c periodo.c *.h
	$(CC) $(CFLAGS) -o $@ ident.c furuta_serial.c serial.c $(NUCLEO) $(LDLIBS)

clean:
	rm -f *.exe *.csv
