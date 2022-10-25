
CC=gcc
TARGET=main
OBJ=dcfan.o	wiringPi.o
DEPS=wiringPi.h
$(TARGET): $(OBJ)
	$(CC)	-o	$@	$(OBJ)
%.0: %.c	(DEPS)
	$(CC)	-c	$<
clean: 
	rm	-f	$(TARGET)	$(OBJ)
