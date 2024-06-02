#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

#include "roboszpon_node.h"

void Flash_SaveNodeConfig(uint32_t pageAddress, roboszpon_node_t* node);

void Flash_LoadNodeConfig(uint32_t pageAddress, roboszpon_node_t* node);

uint32_t Flash_GetTotalWriteCount();

#endif // FLASH_MEMORY_H