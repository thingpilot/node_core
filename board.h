/**
  * @file    
  * @version 1.0.0
  * @author  
  * @brief    
  * 
  */

/** Includes 
 */
#include "mbed.h"


#define BOARD DEVELOPMENT_BOARD_v1_1_0
#if defined (BOARD) && (BOARD == DEVELOPMENT_BOARD_V1_1_0)
#define _PERSISTENT_STORAGE_DRIVER STM24256

//UART
#define         PC_TXD         PC_10
#define         PC_RXD         PC_11

//Pin Defines for I2C Bus
#define         EEPROM_WC      PC_2
#define         SDA            PB_9 
#define         SCL            PB_8
#define         I2C_FREQ       400000

#endif /* #if defined (BOARD) && (BOARD == DEVELOPMENT_BOARD_V1.1.0) */