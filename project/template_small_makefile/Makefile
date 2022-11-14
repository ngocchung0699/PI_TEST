
CC=gcc
TARGET=bin
OBJ=main.o	lib.o
DEPS=lib.h
$(TARGET): $(OBJ)
	$(CC)	-o	$@	$(OBJ)
%.o: %.c	(DEPS)
	$(CC)	-c	$<
clean: 
	rm	-f	$(TARGET)	$(OBJ)
