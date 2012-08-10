# Makefile for simrf, currently only builds a single platform at a time
# Karl Palsson <karlp@tweak.net.au> 2012
# MIT/X11/BSD licensed


OBJECTS=simrf.o
LIBNAME=libsimrf
INSTALLDIR=..

CFLAGS+=-DNDEBUG
CFLAGS+=-std=gnu99
CFLAGS+=-Wall -Os -g
CFLAGS+=-ffunction-sections -fdata-sections
CFLAGS+=-Wstrict-prototypes

CFLAGS_AVR= -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

ifeq ($(CPU),stm32)
CFLAGS+=-fno-common -mcpu=cortex-m3 -mthumb
PREFIX=arm-none-eabi
endif
ifeq ($(CPU),atmega32u4)
CFLAGS+=$(CFLAGS_AVR) -mmcu=$(CPU)
PREFIX=avr-
endif

ARFLAGS=rcs

CC=$(PREFIX)gcc
AR=$(PREFIX)ar

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif


all: $(LIBNAME).a

$(LIBNAME).a: $(OBJECTS)
	@printf "  AR      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(AR) $(ARFLAGS) $@ $^

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<
install:
	cp $(LIBNAME).a $(INSTALLDIR)

clean:
	@printf "  CLEAN\n"
	$(Q)rm -rf $(OBJECTS)
	$(Q)rm -rf $(LIBNAME).so
	$(Q)rm -rf $(LIBNAME).a

.PHONY: all clean library install
