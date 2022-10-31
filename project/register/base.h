#ifndef	__BASE_H__
#define	__BASE_H__

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#define BASE_ADR 0xfe200000

volatile uint32_t *base ;


#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

void base_init();

#endif