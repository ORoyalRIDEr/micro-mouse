IDIR =../Core/Inc ../Drivers
SDIR =../Core/Src/Lib ../Core/Src/Ecl
SDIRT =src/
CC=gcc
TARG=STM32F072xB

IDIRR = $(shell find $(IDIR) -type d)
CFLAGS=$(patsubst %,-I% ,$(IDIRR)) -D$(TARG) -ggdb #$(patsubst %,-I% ,$(DEPS))

ODIR=obj/WallE
ODIRT=obj/Test

LDIR =../lib

LIBS=-lm

DEPS = $(shell find $(IDIR) -name '*.h')

SRCS = $(shell find $(SDIR) -name '*.c')
SRCST = $(shell find $(SDIRT) -name '*.c')
OBJ = $(patsubst %.c,%.o,$(patsubst %,$(ODIR)/%,$(subst ../,,$(SRCS))))
OBJT = $(patsubst %.c,%.o,$(patsubst %,$(ODIRT)/%,$(SRCST)))

ODIRALL = $(dir $(OBJ) $(OBJT))
$(shell mkdir -p $(ODIRALL))

$(ODIRT)/%.o:%.c
	@$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o:../%.c $(DEPS)
	@$(CC) -c -o $@ $< $(CFLAGS)

build: $(OBJ) $(OBJT)
	@$(CC) -o $@ $^ $(CFLAGS) $(LIBS)


.PHONY: clean
clean:
	find $(ODIR) -name "*.o" -type f -delete
	find $(ODIRT) -name "*.o" -type f -delete