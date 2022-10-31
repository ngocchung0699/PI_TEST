#include"base.h"


void base_init(){
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    base = (uint32_t *)mmap(NULL, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, memfd, BASE_ADR);
    if (base == MAP_FAILED)
        printf("mmap gpio failed: %s\n", strerror(errno));    
    close(memfd);
}
