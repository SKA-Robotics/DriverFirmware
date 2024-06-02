#include "flash_memory.h"

void Flash_SaveNodeConfig(uint32_t pageAddress, roboszpon_node_t* node) {
    printf("Writing to flash...\n");
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct = {.TypeErase =
                                                  FLASH_TYPEERASE_PAGES,
                                              .PageAddress = pageAddress,
                                              .NbPages = 1,
                                              .Banks = 0};

    uint32_t errorStatus = 0;
    HAL_FLASHEx_Erase(&eraseInitStruct, &errorStatus);
    if (errorStatus != 0xffffffff) {
        ErrorHandler();
    }

    for (uint8_t paramId = MIN_PARAM_ADDRESS; paramId < MAX_PARAM_ADDRESS + 1;
         ++paramId) {
        uint32_t paramAddress = paramId * 4;
        float value = RoboszponNode_ReadParam(node, paramId);
        printf("Saving parameter 0x%02x at 0x%08x\n", paramId,
               (pageAddress + paramAddress));
        uint32_t data = *(uint32_t*)&value;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAddress + paramAddress,
                          data);
    }

    HAL_FLASH_Lock();
    printf("Done\n");
}

void Flash_LoadNodeConfig(uint32_t pageAddress, roboszpon_node_t* node) {
    printf("Loading configuration from flash...\n");
    for (uint8_t paramId = MIN_PARAM_ADDRESS; paramId < MAX_PARAM_ADDRESS + 1;
         ++paramId) {
        uint32_t paramAddress = paramId * 4;
        uint32_t data = *(__IO uint32_t*)(pageAddress + paramAddress);
        printf("Loading parameter 0x%02x from 0x%08x: 0x%08x\n", paramId,
               (pageAddress + paramAddress), data);
        RoboszponNode_WriteParam(node, paramId, *(float*)&data);
    }
    printf("Done\n");
}

uint32_t Flash_GetTotalWriteCount() { return 0; }
