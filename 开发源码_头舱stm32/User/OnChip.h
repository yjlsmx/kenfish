#ifndef OnChipH
#define OnChipH
#include "SmartTuna.h"
//#define ConfigAddress 0x0800fC00
//#define YJ_RUN_MODE_FLAGS_FLASH_ADDR		(ConfigAddress + 6 * sizeof(int32_t)) 
//#define YJ_NODE_TYPE_FLASH_ADDR					(YJ_RUN_MODE_FLAGS_FLASH_ADDR + 1 * sizeof(int32_t))

#define YJ_RUN_MODE_FLAGS_FLASH_ADDR 		0x0801fC00
#define YJ_NODE_TYPE_FLASH_ADDR					(YJ_RUN_MODE_FLAGS_FLASH_ADDR + 1 * sizeof(int32_t))
#define ConfigAddress 									(YJ_NODE_TYPE_FLASH_ADDR + 1 * sizeof(int32_t))
	
#define YJ_USER_PROGRAM_FLASH_ADDR			0x08008000
#define VERSION 1

#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x800)
#else
#define FLASH_PAGE_SIZE ((uint16_t)0x400)
#endif

typedef struct Parameter
{
	int32_t ServoOffset[3];
	int32_t LServoOffset_sm; //螺旋桨推进舱
	int32_t RServoOffset_sm;//螺旋桨推进舱
	int32_t Version;
	
}Config;

u8 FlashErasePage(u32 PageAddress,u16 PageCount);
extern u8 FlashWriteBytes(u32 Address, u8 *Buffer, u16 ByteCount);
extern void FlashReadBytes(u32 Address, u8 *Buffer, u16 ByteCount);
extern void FlashReadWords(u32 Address, u32 *Buffer, u16 WordCount);
void SaveParameter(Config *cfg);
extern void ReadParameter(Config *Config);


//设置系统运行状态
void YJ_SetRunMode(int32_t mode);

//获取系统运行状态
int32_t YJ_GetRunMode();

void YJ_SetNodeType(int32_t type);

int32_t YJ_GetNodeType();

void testFlash();

#endif
