
#include "sd.h"



extern volatile uint16_t Timer1; 
typedef struct sd_info { 
  volatile uint8_t type;
} sd_info_ptr; 
sd_info_ptr sdinfo;

//----------------------------------------------- 
extern SPI_HandleTypeDef hspi2; 
extern UART_HandleTypeDef huart1; 
//-----------------------------------------------

void SD_PowerOn(void) 
{ 
  Timer1 = 0; 
  while(Timer1<2) //Aktivierungsfunktion  (20 Millisekunden Wartezeit 
    ; 
}

//----------------------------------------------- 
uint8_t sd_ini(void) 					//Initialisieren der SD-Karte 
{ 
	{ 
  uint8_t i; 
  int16_t tmr; 
  uint32_t temp; 
  LD_OFF; 
  sdinfo.type = 0;
	return 0; 
}
  
} 
//-----------------------------------------------

static void Error (void) 										// rote LED im Falle eines Fehlers
{ 
  LD_ON; 
} 
//-----------------------------------------------

uint8_t SPIx_WriteRead(uint8_t Byte) 
{ 
  uint8_t receivedbyte = 0; 
  if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK) 	//Schreiben und Lesen des SPI- Busses
  { 
    Error(); 
  } 
  return receivedbyte; 
} 
//-------------------------------------------		drei Funktionen für SPI - Lesen, Schreiben und normaler Bytelauf 

void SPI_SendByte(uint8_t bt) 
{ 
  SPIx_WriteRead(bt); 
} 
//----------------------------------------------- 
uint8_t SPI_ReceiveByte(void) 
{ 
  uint8_t bt = SPIx_WriteRead(0xFF); 
  return bt; 
} 
//----------------------------------------------- 
void SPI_Release(void) 
{ 
  SPIx_WriteRead(0xFF); 
} 
//-----------------------------------------------





