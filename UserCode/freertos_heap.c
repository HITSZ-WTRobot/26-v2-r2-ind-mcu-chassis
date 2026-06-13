#include "FreeRTOS.h"

uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram_bss")));
