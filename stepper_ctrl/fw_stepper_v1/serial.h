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

#ifndef SERIAL_H
#define SERIAL_H

// --- USART config ---
#define USART_BAUDRATE 9600 /* baud rate */
#define RX_BUF_SZ 200 /* receive buffer size (max 255!) */
#define RX_DONE 0 /* command received flag */


// --- SCPI errors ---
// SCPI error info buffer size [B]
#define SCPI_ERR_MAXBUF 128

// SCPI error flags
#define SCPI_ERR_STORE 1 /* store error message to buffer */
#define SCPI_ERR_SEND 2 /* send error from internal buffer */
#define SCPI_ERR_PSTR 0 /* info is PSTR */
#define SCPI_ERR_STR 4 /* info is RAM string */

// SCPI errors
#define SCPI_ERR_noError 0l
#define SCPI_ERR_undefinedHeader -113l
#define SCPI_ERR_wrongParamType -104l
#define SCPI_ERR_tooFewParameters -109l
#define SCPI_ERR_std_mediaProtected -258l



// --- prototypes ---
void serial_init(void);
void serial_idle();
uint8_t serial_decode(char *cbuf,char **par);
void serial_tx_byte(uint8_t byte);
void serial_tx_data(uint8_t *data, uint8_t len);
void serial_tx_word(uint16_t word);
void serial_tx_dword(uint32_t dword);
void serial_tx_float(float value);
void serial_tx_cstr(const char *str);
void serial_tx_str(char *str);
void serial_error(int16_t err,const char *info,uint8_t mode);



#endif
