#include "eeprom.h"


uint8_t FLASH_ErasePage (uint32_t Address, uint32_t Bank_Select)
{
	__disable_irq();
	EEPROM_ErasePage (Address, Bank_Select);	
	__enable_irq();
	return 0;
}

uint8_t FLASH_ProgramWord (uint32_t Address, uint32_t Data, uint32_t Bank_Select)
{
	__disable_irq();
	EEPROM_ProgramWord(Address, Bank_Select, Data);
	__enable_irq();
	return 0;
}

uint8_t FLASH_ProgramByte (uint32_t Address, uint8_t Data, uint32_t Bank_Select)
{
	__disable_irq();
	EEPROM_ProgramByte(Address, Bank_Select, Data);
	__enable_irq();
	return 0;
}

void FLASH_ReadWord (uint32_t Address, uint32_t *buf, uint32_t Bank_Select)
{	
	__disable_irq();
	*buf = EEPROM_ReadWord (Address, Bank_Select);
	__enable_irq();	
}

void FLASH_ReadByte (uint32_t Address, uint8_t *buf, uint32_t Bank_Select)
{	
	__disable_irq();
	*buf = EEPROM_ReadByte (Address, Bank_Select);
	__enable_irq();	
}
