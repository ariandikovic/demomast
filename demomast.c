/*-------------------------------
This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
--------------------------------

	arian@kset.org
--------------------------------*/


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 115200 //baudrate define
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) //magic number !

unsigned char scratch [15];	//scratchpad!

static inline void usartInit (void)//initialising USART; static inline! doing it only once!
{	
UBRRL=BAUD_PRESCALE ;//load 8 lower bits of baud prescaler
UBRRH=(BAUD_PRESCALE >> 8);	//shift for 8 places right
UCSRC=0b10000110 ;//UCSRC select,asynchroneous,no parity, 1 stop bit, 8 bit character size, clock polarity 0
UCSRB=0b00011000 ;//no interrupts 000, RX enable 1, TX enable 1, no ninth bit 000
}

static inline void spiInit (void) //initialise hardware SPI interface
{
PORTB = PORTB | (1<< PB4);//set SS high, do not touch other bits
SPCR = 0b01010000; //disable interrupt,enable SPI,output MSB first,mode 0,clock depends on SPSR 
SPSR = 1; //SPI double speed ,SPI2X=1 so we have FSPI=FOSC/2
}


static inline void atmegainit (void) //microcontroller peripherial setup
{

DDRB = 0b10110110;//SS (PB4), MOSI(PB5), SCK(PB7) outputs, MISO(PB6) input->we need it for SPI
//pins PB0,PB3 inputs, on PB0 we have switch T1
PORTB = PORTB | (1<< PB0);	//pull up on PB0 active
PORTB = PORTB | (1<< PB1);	//set PB1 high, red LED off
PORTB = PORTB | (1<< PB2);	//set PB2 high, green LED off

DDRD = 0x00;	//all pins inputs, we will need INT0 coneccted to nIRQ
PORTD = 0X00;	//switch pull-up OFF

MCUCR = 0x00;		//low on INT0 trigers interrupt
GICR = 0b01000000;	//enable INT0
}


void usartHEXmyoutput (unsigned char i) //will convert to nice HEX output human looking at usart terminal
{
unsigned char j;	//we need one more variable
j=i & 0b00001111;	//4 lower bits in j, second digit of hex value
i=i>>4;				//upper 4 bits shifted right gives us the first digit of hex value
if (i<10){			//less than then, write ascii
	i=i+48;			//that means value + 48
	}else {			//greater or equal to ten, we need to output ABCDEF
	i=i+55;			//what gives us value plus 55 (uppercase)
	}
if (j<10){			//same as for i...
	j=j+48;
	}else {
	j=j+55;
	}
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=i;							//i goes out
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=j;							//j goes out
return;
}

void usartPutchar (char c)	//writes one char to USART
{
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent, wait UDRE to be set
UDR=c;							//char goes out
return;
}

void usartPutCRLF (void)	//writes CRLF to USART
{
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=13;							//Carriage return 
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=10;							//New line feed
return;
}


void usartSendstring (char *s) //sends string to USART,feed it with a string pointer
{
unsigned char i;
for(i=0 ; s[i] != '\0'; i++){ //loop sending chars until NULL encountered
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent
UDR=s[i]; //if yes get next char for output
}
return; 
}

void spiReadBurst (char addr,unsigned char *buf,unsigned char broj) //cita SPI sabirnicu u BURST modu
{						//input:start adress and pointer to data field
unsigned char i;				//loop counter
PORTB = PORTB & (~(1<< PB4));	//SS low, everything else stays
SPDR= (addr & 0b01111111);		//start SPI adress for burst read + MSB=0 for read
while (!(SPSR & (1<<SPIF)));	//wait for SPIF, byte sent to SPI
for(i=0 ; i<broj; i++){ 		//loop
SPDR=0;							//something to SPDR, RFM22 will ignore this on
while (!(SPSR & (1<<SPIF)));	//wait for SPDR loaded wit data from RFM22
buf[i]=SPDR; 					//write data to scratch
}
PORTB = PORTB | (1<< PB4);		//set SS high
return;
}

void spiWrite (unsigned char addr,unsigned char data) //writes data to address addr to SPI
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others
SPDR= (addr | (1<<7));			//SPI address for write + MSB=1 for write
while (!(SPSR & (1<<SPIF)));	//wait for SPIF, byte is sent to SPI
SPDR=data;						//loading data to SPDR
while (!(SPSR & (1<<SPIF)));	//wait for SPIF, byte is sent to SPI
PORTB = PORTB | (1<< PB4);		//set SS high
return;
}

unsigned char spiRead (unsigned char addr)
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others
SPDR= (addr & 0b01111111);		//start SPI adress for burst read + MSB=0 for read
while (!(SPSR & (1<<SPIF)));	//wait for SPIF, byte is sent to SPI
SPDR=0;							//something to SPDR, RFM22 will ignore this one
while (!(SPSR & (1<<SPIF)));	//wait for SPDR loaded wit data from RFM22
PORTB = PORTB | (1<< PB4);		//set SS high
return SPDR;					//return data from address addr
}

static inline void RFM22init (void) //init RFM22 module
{
_delay_ms(150);			//wait 150 ms, init...
spiWrite(0x07, 0x80);		//software reset RFM22
_delay_ms(150);			//wait another 150 ms to be sure

spiWrite(0x05, 0x00);	//disable all interrupts
spiWrite(0x06, 0x00);	//disable all interrupts

spiRead (0x03);//read interrupt status 1 registar, to clear interrupt flag if set
spiRead (0x04);//read interrupt status 2 registar, to clear interrupt flag if set


spiWrite(0x07, 0x01);	//to READY mode

//Si4432 V2 silicon specific -> silicon labs AN415 (rev 0.6) str. 24
spiWrite(0x5A, 0x7F); //write 0x7F to the VCO Current Trimming register
spiWrite(0x58, 0x80); //write 0xD7 to the ChargepumpCurrentTrimmingOverride register
spiWrite(0x59, 0x40); //write 0x40 to the Divider Current Trimming register
//best receiver performances setup
spiWrite(0x6A, 0x0B); //write 0x0B to the AGC Override 2 register,RSSI readout correction
spiWrite(0x68, 0x04); //write 0x04 to the Deltasigma ADC Tuning 2 register
spiWrite(0x1F, 0x03); //write 0x03 to the Clock Recovery Gearshift Override register

spiWrite(0x09, 0x7D);////XTAL correction for MASTER P1 proto
spiWrite(0x0A, 0x00);//GPIO clock izlaz 30 Mhz, no clock tail, no Low freq clock
spiWrite(0x0B, 0xD2);//GPIO 0 - strong drive (HH), no pullup, TX state
spiWrite(0x0C, 0xD5);//GPIO 1 - strong drive (HH), no pullup, RX state
spiWrite(0x0D, 0x00);//GPIO 2 - strong drive (HH), no pullup, CLK output

spiWrite(0x0F, 0x70);//ADC input ->GND
spiWrite(0x10, 0x00);//ADC offset ->0
spiWrite(0x12, 0x00);//temp. sensor calibration off
spiWrite(0x13, 0x00);//temp. sensor offset ->0

spiWrite(0x1C, 0x04);//IF filter bandwith -> RFM datasheet str. 44
spiWrite(0x1D, 0x40);//AFC enable
spiWrite(0x1E, 0x05);//AFC timing -> ?

spiWrite(0x20, 0xC8);//clock recovery oversampling
spiWrite(0x21, 0x00);//clock recovery offset 2
spiWrite(0x22, 0xA3);//clock recovery offset 1
spiWrite(0x23, 0xD7);//clock recovery offset 0
spiWrite(0x24, 0x00);//clock recovery timing loop 1
spiWrite(0x25, 0xA6);//clock recovery timing loop 0

spiWrite(0x30, 0x8E);//CRC-16 on,TX packet handling on,CRC over entire packet,RX packet handling on
spiWrite(0x32, 0x00);//no header check
spiWrite(0x33, 0x02);//NO header, sync word 3 and 2 ON -> 2D, D4, variable packet lenght on
spiWrite(0x34, 0x10);//preamble 16 nibbles ->64 bits
spiWrite(0x35, 0x30);//preamble detection 4 nibbles -> 24 bits
spiWrite(0x36, 0x2D);//sync word 3
spiWrite(0x37, 0xD4);//sync word 2
//spiWrite(0x3E, 0x02);//packet lenght 2 bytes (payload)
spiWrite(0x69, 0x20);//AGC enable
//-----------------------------------------------------
spiWrite(0x6D, 0x00);//output power 8 dBm

spiWrite(0x70, 0x21);//whitening ON, DATA RATE under 30 kbps!!!- > bit 5 SET!!!!!
spiWrite(0x71, 0x23);//GFSK, FIFO mode

spiWrite(0x72, 0x40);//freq. dev. 40 khz

spiWrite(0x6E, 0xA3);//TX data rate  1-> 20 kbps
spiWrite(0x6F, 0xD7);//TX data rate  0-> 20 kbps

spiWrite(0x73, 0x00);//no frequency offset
spiWrite(0x74, 0x00);//no frequency offset
spiWrite(0x79, 0x00);//no frequency hopping
spiWrite(0x7A, 0x00);//no frequency hopping

spiWrite (0x75,0x53);//freq. band select 430-440 MHz

spiWrite (0x76,0x62);//carrier 433.92 MHz
spiWrite (0x77,0x00);//carrier 433.92 MHz
//-----------------------------------------------------

spiWrite(0x05, 0x03);//interrupt enable 1 register-valid packet and crc error interrupt

//reset RX FIFO
spiWrite (0x08,0x02);//write to addr 08 one, ffclrrx=1
spiWrite (0x08,0x00);//write to addr 08 zero, ffclrrx=0

return;

}

ISR(INT0_vect) 
{ 
   	unsigned char i,j; 
   	i=spiRead (0x03);//read interrupt/status register 1
	j=spiRead (0x04);//read interrupt/status register 2
	i=i & 0b00000011;//leave first 2 bits

/
//last 2 bits on address 0x03 show CRC error or packet OK
if (i==1) {
	spiWrite (0x07,0x01);//to READY mode
	//procedure to reset RX FIFO, read address 0x08...
	spiWrite (0x08,0x02);//write to addr 08 one, ffclrrx=1
	spiWrite (0x08,0x00);//write na addr 08 zero, ffclrrx=0

	PORTB = (PORTB & (~(1<< PB1)));//pin PB1 low, red ON
	_delay_ms(500);					//wait 0.5 sek
	PORTB = (PORTB | (1<< PB1));	//pin PB1 high, red OFF
	spiWrite (0x07,0x05);			//enable READY mode,RX on
}

if (i==2) {
	spiWrite (0x07,0x01);//to READY mode
	spiReadBurst(0x7F,scratch,5);
	//procedure to reset RX FIFO
	spiWrite (0x08,0x02);//write to addr 08 one, ffclrrx=1
	spiWrite (0x08,0x00);//write na addr 08 zero, ffclrrx=0
	
	usartSendstring ("\r\nID node [hex]: ");
	j=scratch[0];//get ID
	usartHEXmyoutput (j);
	usartSendstring ("\r\nBattery [hex]: ");
	j=scratch[1];//get BAT
	usartHEXmyoutput (j);
	usartSendstring ("\r\nRSSI local [hex]: ");
	j=spiRead (0x26);//read RSSI
	usartHEXmyoutput (j);
	usartSendstring ("\r\nAFC 0x73 local [hex]: ");
	j=spiRead (0x73);//get AFC
	usartHEXmyoutput (j);
	usartSendstring ("\r\nAFC 0x74 local [hex]: ");
	j=spiRead (0x74);//get AFC
	usartHEXmyoutput (j);
	usartSendstring ("\r\Data 1 [hex]: ");
	j=scratch[2];//get DAT1
	usartHEXmyoutput (j);
	usartSendstring ("\r\nData 2 [hex]: ");
	j=scratch[3];//get DAT2
	usartHEXmyoutput (j);
	usartSendstring ("\r\nData 3 [hex]: ");
	j=scratch[4];//get DAT3
	usartHEXmyoutput (j);
	usartPutCRLF ();

	PORTB = PORTB & (~(1<< PB2));//pin PB2 low, green ON
	_delay_ms(500);				//wait 0.5 sek
	PORTB = PORTB | (1<< PB2);	//pin PB2 high, green OFF
	spiWrite (0x07,0x05);		//enable READY mode,RX on
	
	}
}


int main (void)
{
	
	atmegainit ();	//init microcontroller peripherials
	usartInit (); 	//init USART
	spiInit ();		// init SPI
	RFM22init ();	//init RFM 22
	
	usartSendstring ("RX node ATmega16L+RFM22(Si4432)\r\n");
	usartSendstring ("--arian@kset.org--\r\n");
	spiWrite (0x07,0x05);//enable READY mode,RX on
	sei(); //global interrupt flag enable


while (1);
	
	return 0;
	}
