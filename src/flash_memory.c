#include "flash_memory.h"
#include "roboszpon_config.h"

void Flash_SaveNodeConfig(uint32_t pageAddress, roboszpon_node_t* node) {
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
        float value = RoboszponConfig_ReadParam(node, paramId);
        uint32_t data = FloatAsUint32(value);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAddress + paramAddress,
                          data);
    }
    HAL_FLASH_Lock();
}

void Flash_LoadNodeConfig(uint32_t pageAddress, roboszpon_node_t* node) {
    for (uint8_t paramId = MIN_PARAM_ADDRESS; paramId < MAX_PARAM_ADDRESS + 1;
         ++paramId) {
        uint32_t paramAddress = paramId * 4;
        uint32_t data = *(__IO uint32_t*)(pageAddress + paramAddress);
        RoboszponConfig_WriteParam(node, paramId, Uint32AsFloat(data));
    }
}

uint32_t Flash_GetTotalWriteCount() { return 0; }
