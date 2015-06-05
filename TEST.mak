CC = iccavr
LIB = ilibw
CFLAGS =  -e -D__ICC_VERSION=722 -D_EE_EXTIO -DATMega640  -l -g -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -g -e:0x10000 -ucrtatmega.o -bfunc_lit:0x1c8.0x10000 -dram_end:0x21ff -bdata:0x200.0x21ff -dhwstk_size:30 -beeprom:0.4096 -fihx_coff -S2
FILES = motor.o 

TEST:	$(FILES)
	$(CC) -o TEST $(LFLAGS) @TEST.lk   -lcatmega
motor.o: D:\Èí¼þ2\iccv7avr\include\iom640v.h D:\Èí¼þ2\iccv7avr\include\_iom640to2561v.h D:\Èí¼þ2\iccv7avr\include\AVRdef.h
motor.o:	motor.c
	$(CC) -c $(CFLAGS) motor.c
