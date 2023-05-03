#pragma once
#include <stdint.h>
#include "MDR32F9Qx_eeprom.h"

uint8_t FLASH_ErasePage (uint32_t Address, uint32_t Bank_Select);
uint8_t FLASH_ProgramWord (uint32_t Address, uint32_t Data, uint32_t Bank_Select);
uint8_t FLASH_ProgramByte (uint32_t Address, uint8_t Data, uint32_t Bank_Select);
void FLASH_ReadWord (uint32_t Address, uint32_t *buf, uint32_t Bank_Select);
void FLASH_ReadByte (uint32_t Address, uint8_t *buf, uint32_t Bank_Select);
