//-----------------------------------------------------------------------------
// Part of simple stepper motor controller.
// This module contains UART receiver ISR and SCPI command decoder.
//
// UART data are received to ring buffer in ISR.
// serial_decode() checks and disects commands separated by LF or semicolon.
// Transmission is not in ISR.
// serial_error() function can hold SCPI style error string.
//
// (c) 2025, Stanislav Maslan, s.maslan@seznam.cz
// url: https://github.com/smaslan/Merkur-Motor
// V1.0, 2025-01-01, initial version
//
// The code and all its part are distributed under MIT license
// https://opensource.org/licenses/MIT.
//-----------------------------------------------------------------------------

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

#include "main.h"
#include "serial.h"

// rx data buffer
char rxd[RX_BUF_SZ];
// rx data position
volatile char *rxd_ptr;
volatile char *rxd_read;
// flags
volatile int8_t rxd_stat;

//----------------------------------------------------------------------------------
// UART STUFF
//----------------------------------------------------------------------------------

void inline serial_fifo_wrap(char *buf,char **addr)
{
	if(*addr >= &buf[RX_BUF_SZ])
		*addr = &buf[0]; // wrap around buffer
}


// USART ISR
ISR(USART0_RX_vect)
{
	// re-enable ISR
	cbi(UCSR0B, RXCIE0); // block this ISR
	sei();

	char *ptr = (char*)rxd_ptr; // work with local copy - faster

	// read all data in USART buffer
	do{
		
		// read byte
		char dbyte = UDR0;
		
		// skip stuff when BT not connected
		if(!bit_is_set(BT_STATE_pin, BT_STATE))
			continue;
		
		// store data byte to FIFO
		*ptr++ = dbyte;
		serial_fifo_wrap(rxd,&ptr);
		*ptr = '\0';
		
		// detect command end
		if(dbyte == '\n' || dbyte == ';')
		rxd_stat++;

	}while(bit_is_set(UCSR0A,RXC0));

	rxd_ptr = ptr; // store back local write pointer

	// allow ISR
	cli();
	sbi(UCSR0B, RXCIE0);
}

// init USART
void serial_init(void)
{
	// default serial lines states
	cbi(UART_ddr, UART_RXD);
	sbi(UART_ddr, UART_RXD);

	// BT module state signal
	cbi(BT_STATE_ddr, BT_STATE);
	cbi(BT_STATE_port, BT_STATE);
	
	// init RX/TX
	UCSR0A = (0<<U2X0);
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (0<<UMSEL00) | (0<<UPM00) | (0<<USBS0) | (3<<UCSZ00);
	UBRR0 = (uint16_t)((F_CPU/(USART_BAUDRATE*16ul)) - 1);

	rxd[0] = '\0';
	rxd_ptr = &rxd[0]; // write pointer
	rxd_read = &rxd[0]; // read pointer
	rxd_stat = 0; // no command yet
}

// idle serial lines
void serial_idle()
{
	UCSR0B = 0;
	cbi(UART_port, UART_RXD);
	cbi(UART_port, UART_TXD);	
}

// decode command, supports following format:
//  "my:command:or:whatver[<space(s)>parameter]"
uint8_t serial_decode(char *cbuf,char **par)
{
	// check command completness
	int8_t count = 0;
	int8_t stat;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		stat = rxd_stat;
	}
	if(stat < 1)
		return(0); // no command yet - get out
	
	// we have command in buffer - quickly copy it to secondary buffer
	
	// command buffer pointer
	char *com = cbuf;
	*com = '\0';

	// rx buffer read pointer
	char *ptr = (char*)rxd_read;

	// no parameter detected yet
	*par = NULL;

	// end of command reached?
	uint8_t is_end = 0;

	while(1)
	{
		// get RX byte
		char db = *ptr;		

		if(db == ' ' && !*par)
		{
			// parameter separator
			*com++ = '\0';		

			// return parameter pointer
			*par = com;
			
			// skip parameter separator(s)
			while(*ptr == ' ')
			{
				ptr++;
				serial_fifo_wrap(rxd,&ptr);
			}
			
			continue;			
		}
		else if(db == ';' || db == '\n' || db == '\r')
		{
			// update remainig events count
			if(db != '\r')
				count++;
			
			// end of command
			if(!is_end)
				*com++ = '\0';
			
			// end of command reached, but check if there is additional rubbish after
			is_end = 1;			
		}
		else if(is_end)
		{
			// end of command reached
			rxd_read = ptr; // store next command read position
			break;
		}
		else
		{
			// copy command data
			*com++ = db;
		}
		
		// move to next RX byte
		ptr++;
		serial_fifo_wrap(rxd,&ptr);
	}

	// update new command events count
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		rxd_stat -= count;
	}
	
	return(cbuf[0] != '\0'); // command detected
}

// send byte
void serial_tx_byte(uint8_t byte)
{
	loop_until_bit_is_set(UCSR0A,UDRE0);
	UDR0 = byte;
}

// send data
void serial_tx_data(uint8_t *data, uint8_t len)
{
	for(uint8_t k = 0; k < len; k++)
		serial_tx_byte(*data++);
}

// send WORD binary
void serial_tx_word(uint16_t word)
{
	union{
		uint8_t db[2];
		uint16_t w;
	}data;
	data.w = word;
	for(uint8_t k = 0; k < 2; k++)
		serial_tx_byte(data.db[k]);
}

// send DWORD binary
void serial_tx_dword(uint32_t dword)
{
	union{
		uint8_t db[4];
		uint32_t dw;
	}data;
	data.dw = dword;
	for(uint8_t k = 0; k < 4; k++)
		serial_tx_byte(data.db[k]);
}

// send FLOAT binary
void serial_tx_float(float value)
{
	union{
		uint8_t db[4];
		float dw;
	}data;
	data.dw = value;
	for(uint8_t k = 0; k < 4; k++)
		serial_tx_byte(data.db[k]);
}

// wait TX done
void serial_tx_wait(void)
{
	loop_until_bit_is_set(UCSR0A,TXC0);
}

// send string from progmem
void serial_tx_cstr(const char *str)
{
	char byte;
	while((byte = pgm_read_byte(str++)) != '\0')
	{
		sbi(UCSR0A,TXC0);
		serial_tx_byte(byte);
	}
	loop_until_bit_is_set(UCSR0A,TXC0);
}

// send string from progmem
void serial_tx_str(char *str)
{
	char byte;
	while((byte = *str++) != '\0')
		serial_tx_byte(byte);
}




// --- SCPI error generator ---

// err: error code; info: optional error message; mode: flags
void serial_error(int16_t err,const char *info,uint8_t mode)
{
	static int16_t err_mem = 0;
	static char *info_mem = NULL;
	static uint8_t is_malloc = 0;
	static char buf[SCPI_ERR_MAXBUF+1];
	
	// remember last error
	if(mode&SCPI_ERR_STORE)
	{
		info_mem = NULL;
		err_mem = err;
		if(mode & SCPI_ERR_STR)
		{
			// RAM string mode						
			info_mem = buf;
			strncpy(info_mem,info,SCPI_ERR_MAXBUF);
			buf[SCPI_ERR_MAXBUF] = '\0';
			is_malloc = 1;
		}
		else
		{
			// PSTR mode
			info_mem = (char*)info;
			is_malloc = 0;
		}
	}

	if(mode&SCPI_ERR_SEND)
	{
		switch(err_mem)
		{
			case SCPI_ERR_undefinedHeader:
				serial_tx_cstr(PSTR("-113, Undefined command header.")); break;
			case SCPI_ERR_wrongParamType:
				serial_tx_cstr(PSTR("-104, Wrong parameter type or value.")); break;
			case SCPI_ERR_tooFewParameters:
				serial_tx_cstr(PSTR("-109, Missing parameters.")); break;
			case SCPI_ERR_std_mediaProtected:
				serial_tx_cstr(PSTR("-258, EEPROM write protected.")); break;
			case SCPI_ERR_noError:
				serial_tx_cstr(PSTR("0, No error.")); break;
			default:
				break;

		}
		if(info_mem)
		{
			serial_tx_byte(' ');
			if(is_malloc)
				serial_tx_str(info_mem);
			else
				serial_tx_cstr(info_mem);
		}
		serial_tx_cstr(PSTR("\n"));

		// loose old info buffer
		info_mem = NULL;
		err_mem = 0;		
	}
	
}
