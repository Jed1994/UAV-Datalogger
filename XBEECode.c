/*
 * XBEECode.c
 *
 * Created: 14/08/2015 4:00:15 p.m.
 *  Author: ktm9431
 */ 
/*******************************************************************************
* DEFINES
*******************************************************************************/
#define F_CPU 8000000UL
#define StartConv ADSC
#define WriteAddr 0xC4
#define ReadAddr 0xC5
#define PRWrite 0xC0
#define PRread 0xC1
#define CTRL_REG1 0x26
#define PT_DATA_CFG 0x13

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
/*******************************************************************************
* STRUCTURES
*******************************************************************************/


/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void Setup(void); // ATmega1284 initialization for this program
void OutputString(char* str);
char TWIinit(char addr, char regist_addr);
char TWIwrite(char address, char reg_addr, char data);
int TWIread(char wraddress, char rdaddr, char reg_addr);
int ReadLiDAR();
int PressureSensor();

/*******************************************************************************
* GLOBAL VARIABLES
*******************************************************************************/


/*******************************************************************************
* MAIN FUNCTION
*******************************************************************************/
int main(void)
{
	volatile int snr;
	volatile static char str[40];
	static int sonar;
	
	
	Setup();
	int a = 0;
	float rdata;

	while(1)
	{
		switch(a)
		{
			case 0:
				rdata = ReadLiDAR();
				sprintf(str, "LIDAR:%i\r\n",rdata);
				OutputString(str);
				a = 1
				break;
			case 1:
				rdata = PressureSensor();
				sprintf(str, "Pressure:%i\r\n",rdata);
				OutputString(str);
				a = 2;
				break;
			case 2:
				//wait for conversion
				while(ADCSRA & (1<<StartConv));
				//Save ADC Value
				snr = ADC;
				sonar = (int)snr; //cm
				//Start Conversion
				ADCSRA |= (1<<StartConv);
						
				
				// String to transmit
				sprintf(str, "Sonar:%d\r\n",sonar);
				OutputString(str);
				_delay_ms(5);
				a = 0;
				break;
		} 
		
	}
	return 0;
}

/*******************************************************************************
* OTHER FUNCTIONS
*******************************************************************************/
void Setup(void) // ATmega1284 setup
{
	// Set Port A for ADC
	DDRA = 0x00; //0001 1111
	PORTA = 0x00;

	//ADC Setup, ADC Enable, 128 prescaler
	ADCSRA = 0x87; //1000 0111
	ADCSRB = 0x00;
	//ADC0 Setup, AVCC setup
	ADMUX = 0x40; //0100 0000
	//Start Initial Conversion
	ADCSRA |= (1<<StartConv);
	
	//Initialize UART
	//RX Interrupt Enable, RX enable, TX Enable
	UCSR0B = 0x98; //1001 1000
	
	//Asynchronous, 8 data bits, no parity, 1 stop, rising edge
	UCSR0C = 0x06; //0000 0110
	//Baud Rate 115.2k changed to 9600 to reduce error 
	UBRR0L = 51;

	//Initialize I2C
	TWBR = 0x08;
	TWDR = 0xFF;                                      // Default content = SDA released.
	TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
	(0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
	(0<<TWWC);
}
	

// transmit serial string NOT USING INTERRUPTS
void OutputString(char* str)
{
	while(*str)
	{
		// wait while the serial port is busy
		while (!(UCSR0A & (1<<UDRE0)));
		// transmit the character
		UDR0 = *str++;		
	}
}

char TWIinit(char addr, char regist_addr)
{
	char error = 0;

	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // enable TWI, send START, clear int flag
	
	while( !(TWCR & (1<<TWINT)) ); // wait for start bit to be sent 1
	
	if ( (TWSR & 0xF8) != TW_START) //status code for successful start is 0x80
		error = 1;
	else
	{
		TWDR = addr; // send slave address and write
		TWCR = (1<<TWINT) | (1<<TWEN); // clear the flag
		while( !(TWCR & (1<<TWINT) )); // wait for slave address to be sent
		
		if ((TWSR & 0xF8) != TW_MT_SLA_ACK) // status code for successful address ACK is 0x18
			error = 1;
		else
		{
			TWDR = regist_addr; //send register address
			TWCR = (1<<TWINT) | (1<<TWEN); //clear the flag
			while( !(TWCR & (1<<TWINT) )); // wait for data to be sent 2
			if ((TWSR & 0xF8) != TW_MT_DATA_ACK) //status code for successful data is 0x28
				error = 1;
		}
	}
	
	return(error);
}

char TWIwrite(char address, char reg_addr, char data)
{
	char error = 0;
	error = TWIinit(address, reg_addr);
	
	if(!error)
	{
		TWDR = data; //send data to appropriate register
		TWCR = (1<<TWINT) | (1<<TWEN); //clear the flag
		while( !(TWCR & (1<<TWINT) )); // wait for data to be sent 3
		if ((TWSR & 0xF8) != TW_MT_DATA_ACK) //status code for successful data is 0x28
			error = 1;
		else
		{
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); // send the STOP bit
			_delay_ms(100);
		}
	}
	return(error);			
}
	

int TWIread(char wraddress, char rdaddr, char reg_addr)
{
	char error = 0;
	char datab = 0;
	
	error = TWIinit(wraddress, reg_addr);
	
	if(!error)
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // enable TWI, send START, clear int flag
		while( !(TWCR & (1<<TWINT)) ); // wait for start bit to be sent 4
				
		if ( (TWSR & 0xF8) != TW_REP_START) //status code for successful start is 0x08
			error = 1;
		else
		{
			TWDR = (rdaddr); // send slave address and write
			TWCR = (1<<TWINT) | (1<<TWEN); // clear the flag
			while( !(TWCR & (1<<TWINT) )); // wait for slave address to be sent 5
				
			if ((TWSR & 0xF8) != TW_MR_SLA_ACK) // status code for successful address ACK is 0x40
				error = 1;
			else
			{
				TWCR = (1<<TWINT)|(1<<TWEN); // clear the flag
				_delay_ms(100);
				datab = TWDR;
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); // send the STOP bit
				//delay
				_delay_ms(100);
				return(datab) ;
			}
		}
	}
	return(error);		
}

int ReadLiDAR()
{
	char msb = 0;
	char lsb = 0; 
	int dist = 0;
	
	TWIwrite(WriteAddr, 0x00, 0x04);
	_delay_ms(100);
	msb = TWIread(WriteAddr, ReadAddr, 0x0f);
	_delay_ms(100);
	lsb = TWIread(WriteAddr, ReadAddr, 0x10);
	
	dist = (int)(msb<<8)|lsb;
	return(dist);
}

int PressureSensor()
{
		char p_msb = 0;
		char p_csb = 0;
		char p_lsb = 0;
		char t_msb = 0;
		char t_lsb = 0;
		char status = 0;
		float tempcsb = 0;
		float altitude = 0;

		TWIwrite(PRWrite, CTRL_REG1, 0xB8);	//Set to Altimeter
		_delay_ms(100);
		TWIwrite(PRWrite, PT_DATA_CFG, 0x07); //Enable Data Flags in PT_DATA_CFG
		_delay_ms(100);
		TWIwrite(PRWrite, CTRL_REG1, 0xB9); // Set Active
		_delay_ms(200);
		
		//Adding a break point here causes the code to work
		status = TWIread(PRWrite, PRread, 0x00); //Read Status register
		_delay_ms(100);
		
		//Wait for PDR bit indicates we have new 
		while((status & (1<<1)) == 0);
		
		//If data is ready
		//Read Pressure
		p_msb = TWIread(PRWrite, PRread, 0x01);
		_delay_ms(100);
		p_csb = TWIread(PRWrite, PRread, 0x02);
		_delay_ms(100);
		p_lsb = TWIread(PRWrite, PRread, 0x03);
		_delay_ms(100);
		//Read Temperature
		t_msb = TWIread(PRWrite, PRread, 0x04);
		_delay_ms(100);
		t_lsb = TWIread(PRWrite, PRread, 0x05);
		_delay_ms(100);
		// The least significant bytes l_altitude and l_temp are 4-bit,
		// fractional values, so you must cast the calculation in (float),
		// shift the value over 4 spots to the right and divide by 16 (since
		// there are 16 values in 4-bits).
		tempcsb = (float)(p_lsb>>4)/16;
		altitude = (float) ((p_msb<<8) | p_csb)+tempcsb;
		
		return(altitude);
}

/*******************************************************************************
* INTERRUPT FUNCTIONS
*******************************************************************************/
ISR(USART0_RXC_vect)
{
	char trash;
	trash = UDR0;
	/*
	char ch;
	static char buffer[40];
	static int index = 0;
	ch = UDR0;

	if(ch >= ' ' && ch <= '~' && index <39)
	{
		buffer[index] = ch;
		index++;
	}
	*/
}
