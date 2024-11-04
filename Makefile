DRIVER:= tilib # mspdebug driver
DEVICE = msp430fr5994

ifndef MSPGCCDIR
	MSPGCCDIR=$(HOME)/ti/msp430-gcc
endif

UPDEVICE=$(shell echo $(DEVICE) | tr a-z A-Z)

#paths
SUPPORT_FILE_DIRECTORY = $(MSPGCCDIR)/include

# compiler options
CC      = $(MSPGCCDIR)/bin/msp430-elf-g++

CXXFLAGS = -I . -I $(SUPPORT_FILE_DIRECTORY) -mmcu=$(DEVICE) -g -Os -mhwmult=f5series -std=c++11
LFLAGS = -L . -L $(SUPPORT_FILE_DIRECTORY)


#### Compiling ####

INCL_SRC = $(wildcard includes/*.c)
INCL_H = $(wildcard includes/*.h)

# "$@ task label and $? is dependency list
simple_controller.elf:	simple_controller.c $(INCL_H) $(INCL_SRC)
	$(CC) $(CXXFLAGS) $(LFLAGS) -D __$(UPDEVICE)__ $? -o $@

simple_target.elf:	simple_target.c $(INCL_H) $(INCL_SRC)
	$(CC) $(CXXFLAGS) $(LFLAGS) -D __$(UPDEVICE)__ $? -o $@

# Upload to board
install_simple_controller:	clean simple_controller.elf
	mspdebug $(DRIVER) "prog simple_controller.elf" --allow-fw-update
	
install_simple_target:	clean simple_target.elf
	mspdebug $(DRIVER) "prog simple_target.elf" --allow-fw-update

clean:
	rm -f  *.o *.elf

# start server in background, then client in foreground use gdb debugger
# suppress printing to terminal by redirecting to null device
# default server port is from localhost and port 2000
debug_controller:	install_simple_controller
	mspdebug tilib gdb >/dev/null &
	msp430-elf-gdb simple_controller.elf -ex "target remote :2000"

stop_debug_server:
	pkill mspdebug
