	/*
	 * AS3935.c
	 * Author: Aky
	 */
#include "spi.h"
#include "as3935.h"
#include "delay.h"

#define AS3935_SPI2

#ifdef AS3935_SPI2
#define as3935_spi_init()         SPI2_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#else
#define as3935_spi_init()         SPI1_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#endif
#define AS3935_DELAY(n)   delay_ms(n)

#define AS3935_CMD_RREG         0x00
#define AS3935_CMD_WREG	        0x40
#define AS3935_DISABLE()        GPIO_SetBits(AS3935_CS_GPIO_PORT, AS3935_CS_PIN)
#define AS3935_ENABLE()         GPIO_ResetBits(AS3935_CS_GPIO_PORT, AS3935_CS_PIN)
	
#define _INDOOR 0x12
#define _OUTDOOR 0x0E
#define CS_HIGH RF_PORT|=(1<<CS)
#define CS_LOW RF_PORT&=~(1<<CS)
#define ICP PD6

#define T0_ON TCCR0B |= (1<<CS02);   // wlacza timer0 i prescaler 256
#define T0_OFF TCCR0B &= ~((1<<CS02) | (1<<CS01) | (1<<CS00));   // wylacza timer0 i prescaler 256
void start(void);
void stop(void);



volatile  uint16_t currentcount =0;
volatile uint16_t Overflow_cnt=0;


/*
	void Counter_init(void)
	{
	TCCR1A = 0; // normal mode
	TCCR1B|= (1<<ICES1)| (1<<CS10);  //  no prescaling, rising edge,
	TIMSK1 |= (1<<TOIE1); // input capture interrupt enable, timer1 overflow interrupt enable
	TIFR1 |= (1<<TOV1);
	TCNT1=0;
	}
*/

void AS3935_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_APB2PeriphClockCmd(AS3935_CS_GPIO_CLK,ENABLE);  	 

	GPIO_InitStructure.GPIO_Pin = AS3935_CS_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(AS3935_CS_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_SetBits(AS3935_CS_GPIO_PORT,AS3935_CS_PIN);
}

static unsigned char AS3935_SPI_SendByte(unsigned char byte){
#ifdef AS3935_SPI2
	return SPI2_ReadWriteByte(byte);
#else
	return SPI1_ReadWriteByte(byte);
#endif
}

  
void AS3935ReadRegister(uint8_t StartAddress, uint8_t NumRegs, uint8_t * pData)
{
	int i;
	 
	as3935_spi_init();
	// assert CS to start transfer 
	AS3935_ENABLE();  
	AS3935_DELAY(1);
	// send the command byte
	AS3935_SPI_SendByte(AS3935_CMD_RREG | (StartAddress & 0x3f));
	//AS3935_SPI_SendByte((NumRegs-1) & 0x0f);
	// get the register content
	for (i=0; i< NumRegs; i++)
	{
		*pData++ = AS3935_SPI_SendByte(0xFF);
	} 
	AS3935_DELAY(1);
	AS3935_DISABLE(); 
	
	return;
}

void AS3935WriteRegister(uint8_t StartAddress, uint8_t NumRegs, uint8_t * pData)
{
	int i;
	
	as3935_spi_init();
	// set the CS low  
	AS3935_ENABLE(); 
	AS3935_DELAY(1);
	// send the command byte
	AS3935_SPI_SendByte(AS3935_CMD_WREG | (StartAddress & 0x3f));
	//AS3935_SPI_SendByte((NumRegs-1) & 0x0f);
	// send the data bytes
	for (i=0; i < NumRegs; i++)
	{
		AS3935_SPI_SendByte(*pData++);
	} 
	
	AS3935_DELAY(1);
	//AS3935_START_L(); 
	AS3935_DISABLE();
}


uint8_t _ffsz(uint8_t mask)
{
	uint8_t i = 0;
	if (mask)
		for (i = 1; ~mask & 1; i++)
			mask >>= 1;
	return i;
}


uint8_t registerRead(uint8_t reg, uint8_t mask)
{
	uint8_t regval =0;
	AS3935ReadRegister(reg, 1, &regval);//_rawRegisterRead(reg);
	regval = regval & mask;
	if (mask)
		regval >>= (_ffsz(mask)-1);
	return regval;
}

void registerWrite(uint8_t reg, uint8_t mask, uint8_t data)
{
		uint8_t regval = 0;
		AS3935ReadRegister(reg, 1, &regval);//_rawRegisterRead(reg);
		regval &= ~(mask);
		if (mask)
			regval |= (data << (_ffsz(mask)-1));
		else
			regval |= data;
		AS3935WriteRegister(reg, 1, &regval);
		
}

void Thunder_Init(void) 
{
	uint8_t  value=0x96;
	
	AS3935WriteRegister(0x3C, 1, &value);// set all registers in default mode
	AS3935WriteRegister(0x3D, 1, &value);// calibrate internal oscillator

	AS3935ReadRegister(0x00,1,&value);
	value &= 0xC1;
	value |= (_INDOOR<<1);
	AS3935WriteRegister(0x00, 1, &value);// set to indoor

	AS3935ReadRegister(0x01,1,&value);
	value &= 0x80;
	value |= 0x44;
	AS3935WriteRegister(0x01, 1, &value);// set NFL and WDTreshold

	AS3935ReadRegister(0x02,1,&value);
	value &= 0x80;
	value |= 0x40;
	AS3935WriteRegister(0x02, 1, &value);// clear statistics, min number of ligtning, spike rejection

	AS3935ReadRegister(0x03,1,&value);
	value &= 0x1f;
	value |= 0x00;
	AS3935WriteRegister(0x03, 1, &value);// Frequency division ratio(antenna),mask disturber, interrupt

	value = 0;
	AS3935WriteRegister(0x08, 1, &value);
}

uint8_t lightningDistanceKm()
{
	return registerRead(AS3935_DISTANCE);
}

void powerDown(void)
{
	registerWrite(AS3935_PWD,1);
}

void  powerUp(void)
{
	uint8_t value = 0x96;
	registerWrite(AS3935_PWD,0);
	AS3935WriteRegister(0x3D, 1, &value);
	AS3935_DELAY(3);
}

uint8_t interruptSource(void)
{
	return registerRead(AS3935_INT);
}

void disableDisturbers(void)
{
	registerWrite(AS3935_MASK_DIST,1);
}

void enableDisturbers(void)
{
	registerWrite(AS3935_MASK_DIST,0);
}

uint8_t getMinimumLightnings()
{
	return registerRead(AS3935_MIN_NUM_LIGH);
}

uint8_t setMinimumLightnings(uint8_t minlightning)
{
	registerWrite(AS3935_MIN_NUM_LIGH,minlightning);
	return getMinimumLightnings();
}

void setIndoors()
{
	registerWrite(AS3935_AFE_GB,AS3935_AFE_INDOOR);
}

void setOutdoors()
{
	registerWrite(AS3935_AFE_GB,AS3935_AFE_OUTDOOR);
}

uint8_t getNoiseFloor()
{
	return registerRead(AS3935_NF_LEV);
}

uint8_t setNoiseFloor(uint8_t noisefloor)
{
	registerWrite(AS3935_NF_LEV,noisefloor);
	return getNoiseFloor();
}

uint8_t getSpikeRejection(void)
{
	return registerRead(AS3935_SREJ);
}

uint8_t setSpikeRejection(uint8_t srej)
{
	registerWrite(AS3935_SREJ, srej);
	return getSpikeRejection();
}

uint8_t getWatchdogThreshold()
{
	return registerRead(AS3935_WDTH);
}

uint8_t setWatchdogThreshold(uint8_t wdth)
{
	registerWrite(AS3935_WDTH,wdth);
	return getWatchdogThreshold();
}

void clearStats()
{
	registerWrite(AS3935_CL_STAT,1);
	registerWrite(AS3935_CL_STAT,0);
	registerWrite(AS3935_CL_STAT,1);
}

uint16_t lightningEnergy(void)
{
	uint16_t v = 0;
	char bits8[4];

	// Energy_u e;
	// REG_u reg4, reg5, reg6;

	bits8[3] = 0;
	bits8[2] = registerRead(AS3935_ENERGY_3);
	bits8[1] = registerRead(AS3935_ENERGY_2);
	bits8[0] = registerRead(AS3935_ENERGY_1);

	v = bits8[2]*65536 + bits8[1]*256 + bits8[0];

	return v;
}


uint8_t tuneAntena (void)
{

	uint16_t stop_count=0;
	uint16_t target = 3125;
	int bestdiff = 32767;
	int currdiff = 0;
	uint8_t bestTune = 0;
	uint8_t currTune = 0;
	/*cli();
	Timer0_init();
	Int2_init();
	int32_t setupTime;

	// set lco_fdiv divider to 0, which translates to 16
	// so we are looking for 31250Hz on irq pin
	// and since we are counting for 100ms that translates to number 3125
	// each capacitor changes second least significant digit
	// using this timing so this is probably the best way to go
	sei();
	start();*/
	registerWrite(AS3935_LCO_FDIV,0);
	registerWrite(AS3935_DISP_LCO,1);
	// tuning is not linear, can't do any shortcuts here
	// going over all built-in cap values and finding the best
		 for (currTune = 0; currTune <= 0x0F ; currTune++)
				{
			registerWrite(AS3935_TUN_CAP,currTune);
					 // wait to settle
			AS3935_DELAY(10);
			currentcount=0;
			/*stop_count=0;
			setupTime=millis()+100;
			// wait 100ms for measurments
			while ((long int)millis() - setupTime<0)
			{
				//
			}*/
			/*ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				stop_count=currentcount;
				}*/
			currentcount=0;
			currdiff = target - stop_count;
			// don't look at me, abs() misbehaves
			if(currdiff < 0)
				currdiff = -currdiff;
			if(bestdiff > currdiff)
			{
				bestdiff = currdiff;
				bestTune = currTune;
			}
				}


	if (bestdiff<109)
	{
		 registerWrite(AS3935_TUN_CAP,bestTune);
		AS3935_DELAY(2);
		registerWrite(AS3935_DISP_LCO,0);
		// and now do RCO calibration
		powerUp();
		/*uart_putint(bestTune,10);
			uart_puts(" ");
		 uart_putint(bestdiff,10);
			uart_puts("\r\n");*/
	return (1);
	}
	// if error is over 109, we are outside allowed tuning range of +/-3.5%
	else
	{
		powerUp();
		return (0);

	}
}


	/*void start(void)
	{
		T0_ON;
	}


// Przerwanie INT2 od AS3935
	ISR(INT2_vect)
	{
		currentcount++;
	}*/
