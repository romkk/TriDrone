PRG            = TriDrone

CC             = avr-gcc

OBJDIR 				 = obj
SRCDIR         = src

SRC           := $(wildcard $(SRCDIR)/*.c)
OBJ           := $(patsubst $(SRCDIR)/%,%,$(patsubst %.c,%.o,$(SRC)))
PROGRAMMER     = avrispmkII
PORT           = usb
MCU_TARGET     = atmega328p 
AVRDUDE_TARGET = atmega328p
OPTIMIZE       = -Os
DEFS           =
LIBS           = -lm
 
HZ             = 16000000
 
override CFLAGS  = -g -DF_CPU=$(HZ) -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS = -Wl,-Map,$(PRG).map
 
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
 
NO_COLOR = \x1b[0m
O1_COLOR = \x1b[0;01m
O2_COLOR = \x1b[32;01m

PREFIX = "$(O2_COLOR)==>$(O1_COLOR)"
SUFFIX = "$(NO_COLOR)"


all: clean $(PRG).elf lst text #eeprom
 
clean:
	@echo $(PREFIX) Cleaning up $(SUFFIX)
	rm -rfv $(OBJDIR) *.elf *.eps *.hex *.bin *.srec *.lst *.map \
		$(EXTRA_CLEAN_FILES)
 
$(PRG).elf: $(OBJDIR)/ $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
 
%.o: $(SRCDIR)/%.c
	@echo $(PREFIX) Compiling $< $(SUFFIX)
	$(CC) $(CFLAGS) -c $< -o $(OBJDIR)/$@

$(OBJDIR)/:
	mkdir -p $@

lst: $(PRG).lst
 
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@
 
# Rules for building the .text rom images
 
text: hex bin srec
 
hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec
 
%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
 
%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@
 
%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
 
# Rules for building the .eeprom rom images
 
eeprom: ehex ebin esrec
 
ehex:  $(PRG)_eeprom.hex
#ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec
 
%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@
 
#%_eeprom.srec: %.elf
#	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@
 
%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@
 
 
# command to program chip (invoked by running "make install")
install:  $(PRG).hex
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v  \
         -U flash:w:$(PRG).hex 
 
fuse:
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v \
	-U lfuse:w:0xc6:m -U hfuse:w:0xd9:m 	
 
ddd: gdbinit
	ddd --debugger "avr-gdb -x $(GDBINITFILE)"
 
gdbserver: gdbinit
	simulavr --device $(MCU_TARGET) --gdbserver
 
gdbinit: $(GDBINITFILE)
 
$(GDBINITFILE): $(PRG).hex
	@echo "file $(PRG).elf" > $(GDBINITFILE)
 
	@echo "target remote localhost:1212" >> $(GDBINITFILE)
	@echo "load"                         >> $(GDBINITFILE)
	@echo "break main"                   >> $(GDBINITFILE)
	@echo
	@echo "Use 'avr-gdb -x $(GDBINITFILE)'"

.SILENT: $(OBJDIR)/
