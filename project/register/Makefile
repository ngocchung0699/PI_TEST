
CC=gcc
TARGET=bin
OBJ= main.o lib.o lib_uart.o
DEPS= lib.h lib_uart.h
$(TARGET): $(OBJ)
	$(CC)	-o	$@	$(OBJ) -lpthread
%.0: %.c	(DEPS)
	$(CC)	-c	$<
clean: 
	rm	-f	$(TARGET)	$(OBJ)
