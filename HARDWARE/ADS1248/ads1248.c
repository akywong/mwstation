#include "spi.h"
#include "RTD_Math_ads1248.h"
#include "ads1248.h"
#include "delay.h"

#define ADS1248_SPI2

#ifdef ADS1248_SPI2
#define ads1248_spi_init()         SPI2_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#else
#define ads1248_spi_init()         SPI1_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#endif
#define ADS1248_DELAY(n)   delay_ms(n)

void ADS1248_GPIO_Init(void){
  GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_APB2PeriphClockCmd(ADS1248_CS_GPIO_CLK|ADS1248_START_GPIO_CLK|ADS1248_DRDY_GPIO_CLK|ADS1248_RESET_GPIO_CLK,ENABLE);  	 

	GPIO_InitStructure.GPIO_Pin = ADS1248_START_GPIO_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADS1248_START_GPIO_PORT, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = ADS1248_DRDY_GPIO_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADS1248_DRDY_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ADS1248_RESET_GPIO_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADS1248_RESET_GPIO_PORT, &GPIO_InitStructure);	

	//SEL0
	GPIO_InitStructure.GPIO_Pin = ADS1248_SEL0_GPIO_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADS1248_SEL0_GPIO_PORT, &GPIO_InitStructure);	
	
	//SEL1
	GPIO_InitStructure.GPIO_Pin = ADS1248_SEL1_GPIO_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ADS1248_SEL1_GPIO_PORT, &GPIO_InitStructure);
	
	/*Delay20ms();
  ADS1248_DISABLE();
	ADS1248_START_L();
	ADS1248_RST_L();
	Delay10ms();
  ADS1248_RST_H();
	ADS1248_START_H();
  Delay10ms();*/
	
  ADS1248_DISABLE();
	ADS1248_START_H();
	ADS1248_RST_H();
	Delay10ms();  
	ADS1248_RST_L();
	Delay10ms();  
	ADS1248_RST_H();
	Delay10ms(); 
}
 
static unsigned char ADS1248_SPI_SendByte(unsigned char byte){
#ifdef ADS1248_SPI2
	return SPI2_ReadWriteByte(byte);
#else
	return SPI1_ReadWriteByte(byte);
#endif
}

int ADS1248WaitForDataReady(int Timeout){
  if (Timeout > 0)
	{
		// wait for /DRDY = 1 to make sure it is high before we look for the transition low
		while ( !( IS_ADS1248_READY() ) && (Timeout-- >= 0));
		// wait for /DRDY = 0
		while ( (IS_ADS1248_READY()) && (Timeout-- >= 0));
		if (Timeout < 0)
			return ADS1248_ERROR; 					//ADS1248_TIMEOUT_WARNING;
	}
	else
	{
		// wait for /DRDY = 1
		while ( !( IS_ADS1248_READY() ) );
		// wait for /DRDY = 0
		while ( IS_ADS1248_READY() );
	}
	return 0;
}
  
void ADS1248ReadRegister(int StartAddress, int NumRegs, unsigned * pData)
{
	int i;
	 
	ads1248_spi_init();
	// assert CS to start transfer 
	ADS1248_ENABLE(); 
	ADS1248_START_H(); 
	ADS1248_DELAY(1);
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_RREG | (StartAddress & 0x0f));
	ADS1248_SPI_SendByte((NumRegs-1) & 0x0f);
	// get the register content
	for (i=0; i< NumRegs; i++)
	{
		*pData++ = ADS1248_SPI_SendByte(0xFF);
	} 
	ADS1248_DELAY(1);
	//ADS1248_START_L();
	ADS1248_DISABLE(); 
	
	return;
}

void ADS1248WriteRegister(int StartAddress, int NumRegs, unsigned * pData){
	int i;
	
	ads1248_spi_init();
	// set the CS low  
	ADS1248_ENABLE(); 
	ADS1248_START_H(); 
	ADS1248_DELAY(1);
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_WREG | (StartAddress & 0x0f));
	ADS1248_SPI_SendByte((NumRegs-1) & 0x0f);
	// send the data bytes
	for (i=0; i < NumRegs; i++)
	{
		ADS1248_SPI_SendByte(*pData++);
	} 
	
	ADS1248_DELAY(1);
	//ADS1248_START_L(); 
	ADS1248_DISABLE();
}
 
void ADS1248WriteSequence(int StartAddress, int NumReg, unsigned * pData){

}

void ADS1248SendRDATAC(void)
{
	ads1248_spi_init();
	
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_RDATAC);
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return;
}

void ADS1248SendSDATAC(void)
{
	ads1248_spi_init();
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_SDATAC);
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return;
}

void ADS1248SendSYSOCAL(void)
{
	ads1248_spi_init();
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_SYSOCAL);
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return;
}

void ADS1248SendSYSGCAL(void)
{
	ads1248_spi_init();
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_SYSGCAL);
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return;
}

void ADS1248SendSELFOCAL(void)
{
	ads1248_spi_init();
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	// send the command byte
	ADS1248_SPI_SendByte(ADS1248_CMD_SELFOCAL);
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return;
}
/*
 * Register Set Value Commands
 *
 * These commands need to strip out old settings (AND) and add (OR) the new contents to the register
 */
int ADS1248SetBurnOutSource(int BurnOut)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
	Temp &= 0x3f;
	switch(BurnOut) {
		case 0:
			Temp |= ADS1248_BCS_OFF;
			break;
		case 1:
			Temp |= ADS1248_BCS_500nA;
			break;
		case 2:
			Temp |= ADS1248_BCS_2uA;
			break;
		case 3:
			Temp |= ADS1248_BCS_10uA;
			break;
		default:
			dError = ADS1248_ERROR;
			Temp |= ADS1248_BCS_OFF;
	}
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_0_MUX0, 0x01, &Temp);
	return dError;
}

int ADS1248SetChannel(int pMux, int nMux){
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
	Temp &= 0xC0;
	Temp |= (nMux&0x07);
	Temp |= (pMux&0x07)<<3;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_0_MUX0, 0x01, &Temp); 
	return dError;
}

int ADS1248SetBias(unsigned char vBias)
{
	unsigned int Temp;
	Temp = ADS1248_VBIAS_OFF;
	if (vBias & 0x80)
		Temp |=  ADS1248_VBIAS7;
	if (vBias & 0x40)
		Temp |=  ADS1248_VBIAS6;
	if (vBias & 0x20)
		Temp |=  ADS1248_VBIAS5;
	if (vBias & 0x10)
		Temp |=  ADS1248_VBIAS4;
	if (vBias & 0x08)
		Temp |=  ADS1248_VBIAS3;
	if (vBias & 0x04)
		Temp |=  ADS1248_VBIAS2;
	if (vBias & 0x02)
		Temp |=  ADS1248_VBIAS1;
	if (vBias & 0x01)
		Temp |=  ADS1248_VBIAS0;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_1_VBIAS, 0x01, &Temp);
	return ADS1248_NO_ERROR;
}

// Relate to Mux1
int ADS1248SetIntRef(int sRef)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	Temp &= 0x1f;
	switch(sRef) {
		case 0:
			Temp |= ADS1248_INT_VREF_OFF;
			break;
		case 1:
			Temp |= ADS1248_INT_VREF_ON;
			break;
		case 2:
		case 3:
			Temp |= ADS1248_INT_VREF_CONV;
			break;
		default:
			Temp |= ADS1248_INT_VREF_OFF;
			dError = ADS1248_ERROR;

	}
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return dError;
}

int ADS1248SetVoltageReference(int VoltageRef)
{
	unsigned int temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &temp);
	temp &= 0xe7;
	temp |= (VoltageRef&3)<<3;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &temp);
	return dError;
}

int ADS1248SetSystemMonitor(int Monitor)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	Temp &= 0x78;
	switch(Monitor) {
		case 0:
			Temp |= ADS1248_MEAS_NORM;
			break;
		case 1:
			Temp |= ADS1248_MEAS_OFFSET;
			break;
		case 2:
			Temp |= ADS1248_MEAS_GAIN;
			break;
		case 3:
			Temp |= ADS1248_MEAS_TEMP;
			break;
		case 4:
			Temp |= ADS1248_MEAS_REF1;
			break;
		case 5:
			Temp |= ADS1248_MEAS_REF0;
			break;
		case 6:
			Temp |= ADS1248_MEAS_AVDD;
			break;
		case 7:
			Temp |= ADS1248_MEAS_DVDD;
			break;
		default:
			Temp |= ADS1248_MEAS_NORM;
			dError = ADS1248_ERROR;
	}
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return dError;
}

// Relate to SYS0
int ADS1248SetGain(int Gain)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
	Temp &= 0x0f;
	Temp |= (Gain&7)<<4;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_3_SYS0, 0x01, &Temp);
	return dError;
}

int ADS1248SetDataRate(int DataRate)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
	Temp &= 0x70;
	Temp |= (DataRate&0xF);
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_3_SYS0, 0x01, &Temp);
	return dError;
}

// Relate to OFC (3 registers)
int ADS1248SetOFC(long RegOffset)
{
	// find the pointer to the variable so we can write the value as bytes
	unsigned *cptr=(unsigned *)(&RegOffset);
	int i;

	for (i=0; i<3; i++)
	{
		// write the register value containing the new value back to the ADS
		ADS1248WriteRegister((ADS1248_4_OFC0 + i), 0x01, &cptr[i]);
	}
	return ADS1248_NO_ERROR;
}

// Relate to FSC (3 registers)
int ADS1248SetFSC(long RegGain)
{
	// find the pointer to the variable so we can write the value as bytes
	unsigned *cptr=(unsigned *)(&RegGain);
	int i;
	for (i=0; i<3; i++)
	{
		// write the register value containing the new value back to the ADS
		ADS1248WriteRegister((ADS1248_7_FSC0 + i), 0x01, &cptr[i]);
	}
	return ADS1248_NO_ERROR;
}

// Relate to IDAC0
int ADS1248SetDRDYMode(int DRDYMode)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	Temp &= 0xf7;
	Temp |= (DRDYMode&1)<<3;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	return dError;
}

int ADS1248SetCurrentDACOutput(int CurrentOutput)
{
	unsigned int Temp;
	int dError = ADS1248_NO_ERROR;
	ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	Temp &= 0xf8;
	Temp |= (CurrentOutput&0x7);
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	return dError;
}
// Relate to IDAC1
int ADS1248SetIDACRouting(int I1dir, int I2dir)
{
	unsigned int Temp=0;
	int dError = ADS1248_NO_ERROR;
	//ADS1248ReadRegister(ADS1248_11_IDAC1, 0x01, &Temp);
	Temp |= (I1dir&0xf)<<4;
	Temp |= (I2dir&0xf);
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_11_IDAC1, 0x01, &Temp);
	return dError;
}

// Relate to GPIOCFG
int ADS1248SetGPIOConfig(unsigned char cdata)
{
	unsigned int Temp;
	Temp = 0x00;
	if (cdata & 0x80)
		Temp |=  ADS1248_GPIO_7;
	if (cdata & 0x40)
		Temp |=  ADS1248_GPIO_6;
	if (cdata & 0x20)
		Temp |=  ADS1248_GPIO_5;
	if (cdata & 0x10)
		Temp |=  ADS1248_GPIO_4;
	if (cdata & 0x08)
		Temp |=  ADS1248_GPIO_3;
	if (cdata & 0x04)
		Temp |=  ADS1248_GPIO_2;
	if (cdata & 0x02)
		Temp |=  ADS1248_GPIO_1;
	if (cdata & 0x01)
		Temp |=  ADS1248_GPIO_0;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_12_GPIOCFG, 0x01, &Temp);
	return ADS1248_NO_ERROR;
}

// Relate to GPIODIR
int ADS1248SetGPIODir(unsigned char cdata)
{
	unsigned int Temp;
	Temp = 0x00;
	if (cdata & 0x80)
		Temp |=  ADS1248_IO_7;
	if (cdata & 0x40)
		Temp |=  ADS1248_IO_6;
	if (cdata & 0x20)
		Temp |=  ADS1248_IO_5;
	if (cdata & 0x10)
		Temp |=  ADS1248_IO_4;
	if (cdata & 0x08)
		Temp |=  ADS1248_IO_3;
	if (cdata & 0x04)
		Temp |=  ADS1248_IO_2;
	if (cdata & 0x02)
		Temp |=  ADS1248_IO_1;
	if (cdata & 0x01)
		Temp |=  ADS1248_IO_0;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_13_GPIODIR, 0x01, &Temp);
	return ADS1248_NO_ERROR;
}

// Relate to GPIODAT
int ADS1248SetGPIO(unsigned char cdata)
{
	unsigned int Temp;
	Temp = 0x00;
	if (cdata & 0x80)
		Temp |=  ADS1248_OUT_7;
	if (cdata & 0x40)
		Temp |=  ADS1248_OUT_6;
	if (cdata & 0x20)
		Temp |=  ADS1248_OUT_5;
	if (cdata & 0x10)
		Temp |=  ADS1248_OUT_4;
	if (cdata & 0x08)
		Temp |=  ADS1248_OUT_3;
	if (cdata & 0x04)
		Temp |=  ADS1248_OUT_2;
	if (cdata & 0x02)
		Temp |=  ADS1248_OUT_1;
	if (cdata & 0x01)
		Temp |=  ADS1248_OUT_0;
	// write the register value containing the new value back to the ADS
	ADS1248WriteRegister(ADS1248_14_GPIODAT, 0x01, &Temp);
	return ADS1248_NO_ERROR;
}

/* Register Get Value Commands */
// Relate to MUX0
int ADS1248GetBurnOutSource(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
	return ((Temp >> 6) & 0x03);
}

int ADS1248GetChannel(int cMux)			// cMux = 0, AINP; cMux = 1, AINN
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
	if (cMux==0)
		return ((Temp >> 3) & 0x07);
	else
		return (Temp  & 0x07);
}

// Relate to VBIAS
unsigned char ADS1248GetBias(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_1_VBIAS, 0x01, &Temp);
	return (Temp & 0xff);
}

//Relate to MUX1
int ADS1248GetCLKSTAT(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return ((Temp >> 7) & 0x01);
}

int ADS1248GetIntRef(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return ((Temp >> 5) & 0x03);
}

int ADS1248GetVoltageReference(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return ((Temp >> 3) & 0x03);
}

int ADS1248GetSystemMonitor(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
	return (Temp & 0x07);
}

// Relate to SYS0
int ADS1248GetGain(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
	return ((Temp >> 4) & 0x07);
}

int ADS1248GetDataRate(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
	return (Temp & 0x0f);
}

// Relate to OFC (3 registers)
long ADS1248GetOFC(void)
{
	long rData=0;
	unsigned rValue=0;
	unsigned regArray[3];
	int i;
	//write the desired default register settings for the first 4 registers NOTE: values shown are the POR values as per datasheet
	regArray[0] = 0x00;
	regArray[1] = 0x00;
	regArray[2] = 0x00;
	for (i=0; i<3; i++)
	{
		// read the register value for the OFC
		ADS1248ReadRegister((ADS1248_4_OFC0 + i), 0x01, &rValue);
		regArray[i] = rValue;
	}
	rData = regArray[2];
	rData = (rData<<8) | regArray[1];
	rData = (rData<<8) | regArray[0];
	return rData;
}

// Relate to FSC (3 registers)
long ADS1248GetFSC(void)
{
	long rData=0;
	unsigned rValue=0;
	unsigned regArray[3];
	int i;
	//write the desired default register settings for the first 4 registers NOTE: values shown are the POR values as per datasheet
	regArray[0] = 0x00;
	regArray[1] = 0x00;
	regArray[2] = 0x00;
	for (i=0; i<3; i++)
	{
		// read the register value for the OFC
		ADS1248ReadRegister((ADS1248_7_FSC0 + i), 0x01, &rValue);
		regArray[i] = rValue;
	}
	rData = regArray[2];
	rData = (rData<<8) | regArray[1];
	rData = (rData<<8) | regArray[0];
	return rData;
}

int ADS1248GetDRDYMode(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	return ((Temp>>3) & 0x01);
}

int ADS1248GetCurrentDACOutput(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
	return (Temp & 0x07);
}

// Relate to IDAC1
int ADS1248GetIDACRouting(int WhichOne) 		// IDACRoute (0 = I1DIR, 1 = I2DIR)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_11_IDAC1, 0x01, &Temp);
	if (WhichOne==0)
		return ((Temp>>4) & 0x0f);
	else
		return (Temp & 0x0f);
}

// Relate to GPIOCFG
unsigned char ADS1248GetGPIOConfig(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_12_GPIOCFG, 0x01, &Temp);
	return (Temp & 0xff);
}

// Relate to GPIODIR
unsigned char ADS1248GetGPIODir(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_13_GPIODIR, 0x01, &Temp);
	return (Temp & 0xff);
}

// Relate to GPIODAT
unsigned char ADS1248GetGPIO(void)
{
	unsigned int Temp;
	ADS1248ReadRegister(ADS1248_14_GPIODAT, 0x01, &Temp);
	return (Temp & 0xff);
}

int ADS1248RDATACRead(void)		// reads data directly based on RDATAC mode (writes NOP) and 32 SCLKs
{
	int data;
	
	ads1248_spi_init();
	// assert CS to start transfer
	ADS1248_ENABLE();
	ADS1248_DELAY(1);
	
	// get the conversion result
	data = ADS1248_SPI_SendByte(0xFF);
	data = (data << 8) | ADS1248_SPI_SendByte(0xFF);
	data = (data << 8) | ADS1248_SPI_SendByte(0xFF);
	// sign extend data if the MSB is high (24 to 32 bit sign extension)
	if (data & 0x800000)
		data |= 0xff000000;
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();
	return data;
}

int ADS1248RDATARead(void)		// reads data directly based on RDATAC mode (writes NOP) and 32 SCLKs
{
	static int data;
	ads1248_spi_init();
	// assert CS to start transfer 
	Delay1us();Delay1us();Delay1us();Delay1us();Delay1us();
	ADS1248_ENABLE();  
	ADS1248_START_H(); 
	ADS1248_DELAY(1);
	ADS1248_START_L();  
	//while (0 == IS_ADS1248_READY()){}
	ADS1248WaitForDataReady (0);
	// get the conversion result
	data = ADS1248_SPI_SendByte(0xFF);
	data = (data << 8) | ADS1248_SPI_SendByte(0xFF);
	data = (data << 8) | ADS1248_SPI_SendByte(0xFF);
	// sign extend data if the MSB is high (24 to 32 bit sign extension)
	if (data & 0x800000)
		data |= 0xff000000;
	
	ADS1248_DELAY(1);
	// de-assert CS
	ADS1248_DISABLE();  
	
	return data;
}

void Delay1us(void){
  __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}

void Delay10us(void){
	volatile int i=32; 
	while(i--);
	__nop();__nop();__nop();__nop();__nop();__nop();
}

void Delay200us(void){
  volatile int i=700;
	while(i--);
}

void Delay10ms(void){
  volatile int i=35555;
	while(i--);
}

void Delay20ms(void){
	volatile int i=71800;
	while(i--);
}

unsigned char Ads_Calibrate(unsigned int Gain)
{
	  long ofc,fsc;
    unsigned int R=0,Cmd;
    //ADS1248WriteRegister(ADS1248_3_SYS0,1,&Gain);     //

	  ofc = ADS1248GetOFC(); 
	//	fsc =ADS1248GetFSC();
	
   /* Cmd=0x00;
    ADS1248WriteRegister(ADS1248_2_MUX1,1,&Cmd);       //
	  ADS1248_ENABLE();
    ADS1248_SPI_SendByte(ADS1248_CMD_SELFOCAL);     //
		ADS1248_DISABLE();
    R|=ADS1248WaitForDataReady(0);                   //
*/
		ads1248_spi_init();
    Cmd=0x01;
    ADS1248WriteRegister(ADS1248_2_MUX1,1,&Cmd);       //AINP+AINN=(AVDD+AVSS)/2
    ADS1248_ENABLE(); 
		ADS1248_DELAY(1);
		ADS1248_SPI_SendByte(ADS1248_CMD_SYSOCAL);      //
	
		ADS1248_DELAY(1);
		ADS1248_DISABLE();
    R|=ADS1248WaitForDataReady(0);                   //

    /*Cmd=0x02;
    ADS1248WriteRegister(ADS1248_2_MUX1,1,&Cmd);       //AINP=VREF+,AINN=VREF-; for gain calibration
		ADS1248_ENABLE();
    ADS1248_SPI_SendByte(ADS1248_CMD_SYSGCAL);      //
		ADS1248_DISABLE();
    R|=ADS1248WaitForDataReady(0);
*/
		ofc = ADS1248GetOFC(); 
	//	fsc =ADS1248GetFSC();

    return R;
}
float ADS1248_Get_Temperature(void)
{
	int code;
	ADS1248SendRDATAC();
	
	code = ADS1248RDATACRead();
	//delay_ms(10);
	return (ads1248_interpolateTemperatureValue(code));
}
