/*
 * Simple stepper motor controller. More details here:
 * https://github.com/smaslan/Merkur-Motor
 *
 * (c) 2024-2025, Stanislav Maslan, s.maslan@seznam.cz
 * V1.0, 2025-01-01
 *
 * The code and all its part are distributed under MIT license
 * https://opensource.org/licenses/MIT.
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/sfr_defs.h>
#include <util/atomic.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "serial.h"

// --- jump to address ---
#define boot_start(boot_addr) {goto *(const void*)boot_addr;}



//----------------------------------------------------------------------------------
// analog ADC stuff
//----------------------------------------------------------------------------------

TADC adc_data;

#define ADC_VREF 1.1f
#define ADC_VMON_DIV (10000.0/(10000.0 + 62000.0))
#define ADC_GAIN (ADC_VREF/(ADC_AVG*1024.0f))
#define ADC_VMON_CAL (ADC_GAIN/ADC_VMON_DIV)


// ADC conversion done (polling mode)
//ISR(ADC_vect)
uint8_t adc_conv_done()
{
	//sei();
	if(!bit_is_set(ADCSRA,ADIF))
		return(0);

	// average buffer
	static uint8_t avg_cnt = ADC_AVG;
	uint8_t chn = (ADMUX >> MUX0)&0x0F;
	
	static uint16_t temp = 0;
	static uint16_t vmon = 0;
		
	if(chn == ADC_VMON_MUX)
	{
		// VMON measured
		vmon += ADC;		
		chn = ADC_TEMP_MUX;
	}
	else if(chn == ADC_TEMP_MUX)
	{
		// temperature measured
		temp += ADC;
		chn	= ADC_VMON_MUX;
	}

	avg_cnt--;
	if(!avg_cnt)
	{
		avg_cnt = ADC_AVG;
		if(chn == ADC_VMON_MUX)
		{
			adc_data.a_vmon = vmon; vmon = 0;
			adc_data.a_temp = temp; temp = 0;
		}

		// next channel
		ADMUX = (ADMUX & ~(0x0F<<MUX0)) | (chn<<MUX0);
	}
	
	// start next conversion
	sbi(ADCSRA,ADIF);
	sbi(ADCSRA,ADSC);
	
	return(1);
}

// get VMON voltage [V]
float adc_get_vmon()
{
	uint16_t a_vmon = 0;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		a_vmon = adc_data.a_vmon;
	}
	return(ADC_VMON_CAL*a_vmon);
}

// set Vmon low limit allert
void adc_set_vmon_alert(float vmin_min)
{
	adc_data.vmon_alert = vmin_min/ADC_VMON_CAL;
}

// check Vmon alert
uint8_t adc_check_vmon()
{
	uint16_t a_vmon = 0;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		a_vmon = adc_data.a_vmon;
	}
	if(!isnan(adc_data.vmon_alert) && a_vmon < adc_data.vmon_alert)
		return(1);
	return(0);
}

// get temperature [degC]
float adc_get_temp()
{	
	uint16_t a_temp = 0; 
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		a_temp = adc_data.a_temp;
	}		
	return((float)(a_temp/ADC_AVG) - 280.0);
}

// init ADC
void adc_init()
{
	// initial ADC setup (AVcc ref)
	ADMUX = (3<<REFS0)|(0<<ADLAR)|(ADC_VMON_MUX<<MUX0);
	ADCSRA = (1<<ADEN)|(0<<ADATE)|(0<<ADIE)|(7<<ADPS0);
	ADCSRB = (0<<ADTS0);
	
	// start ADC conversion loop
	sbi(ADCSRA,ADSC);
	
	// initial measured value
	adc_data.a_vmon = 0;
	adc_data.a_temp = 0;

	// set default vmon alert
	adc_set_vmon_alert(NAN);
}

// idle ADC before sleep
void adc_idle()
{
	ADCSRA = 0;
}

//----------------------------------------------------------------------------------
// PWM controllers
//----------------------------------------------------------------------------------

typedef struct{
	uint16_t phi;
	int16_t step;
	uint8_t amp;
}TPWM;
TPWM pwm;

void pwm_init()
{
	// fast PWM
	TCCR0A = (2<<COM0A0)|(2<<COM0B0)|(3<<WGM00);
	TCCR0B = (0<<WGM02)|(1<<CS00);
	TIMSK0 = (1<<TOIE0);

	// default duty
	OCR0A = 127;
	OCR0B = 127;

	// PWM outputs
	sbi(PWM_ddr, PWM_L1);
	sbi(PWM_ddr, PWM_L2);

	// power stage control
	cbi(PWR_ddr, PWR);
	cbi(PWR_port, PWR);

	pwm.amp = 0;
	pwm.phi = 0;
	pwm.step = 0;
}

// start PWM generation
void pwm_enable()
{
	TCCR0A = (2<<COM0A0)|(2<<COM0B0)|(3<<WGM00);
	sbi(PWR_port, PWR);
}


// idle PWM generation (sets zero current), disable power
void pwm_idle()
{
	pwm.amp = 0;
	cbi(PWR_port, PWR);
}

// start PWM generation
void pwm_disable()
{
	TCCR0A = (0<<COM0A0)|(0<<COM0B0)|(0<<WGM00);
	cbi(PWM_port, PWM_L1);
	cbi(PWM_port, PWM_L2);	
}



// PWM wrap-around ISR
ISR(TIMER0_OVF_vect)
{
	// sin(), cos() LUT
	static const int8_t sico[] PROGMEM = {
		0, 127,   3, 127,   6, 127,   9, 127,  12, 126,  16, 126,  19, 126,  22, 125,
		25, 125,  28, 124,  31, 123,  34, 122,  37, 122,  40, 121,  43, 120,  46, 118,
		49, 117,  51, 116,  54, 115,  57, 113,  60, 112,  63, 111,  65, 109,  68, 107,
		71, 106,  73, 104,  76, 102,  78, 100,  81,  98,  83,  96,  85,  94,  88,  92,
		90,  90,  92,  88,  94,  85,  96,  83,  98,  81, 100,  78, 102,  76, 104,  73,
		106,  71, 107,  68, 109,  65, 111,  63, 112,  60, 113,  57, 115,  54, 116,  51,
		117,  49, 118,  46, 120,  43, 121,  40, 122,  37, 122,  34, 123,  31, 124,  28,
		125,  25, 125,  22, 126,  19, 126,  16, 126,  12, 127,   9, 127,   6, 127,   3,
		127,   0, 127,  -3, 127,  -6, 127,  -9, 126, -12, 126, -16, 126, -19, 125, -22,
		125, -25, 124, -28, 123, -31, 122, -34, 122, -37, 121, -40, 120, -43, 118, -46,
		117, -49, 116, -51, 115, -54, 113, -57, 112, -60, 111, -63, 109, -65, 107, -68,
		106, -71, 104, -73, 102, -76, 100, -78,  98, -81,  96, -83,  94, -85,  92, -88,
		90, -90,  88, -92,  85, -94,  83, -96,  81, -98,  78,-100,  76,-102,  73,-104,
		71,-106,  68,-107,  65,-109,  63,-111,  60,-112,  57,-113,  54,-115,  51,-116,
		49,-117,  46,-118,  43,-120,  40,-121,  37,-122,  34,-122,  31,-123,  28,-124,
		25,-125,  22,-125,  19,-126,  16,-126,  12,-126,   9,-127,   6,-127,   3,-127,
		0,-127,  -3,-127,  -6,-127,  -9,-127, -12,-126, -16,-126, -19,-126, -22,-125,
		-25,-125, -28,-124, -31,-123, -34,-122, -37,-122, -40,-121, -43,-120, -46,-118,
		-49,-117, -51,-116, -54,-115, -57,-113, -60,-112, -63,-111, -65,-109, -68,-107,
		-71,-106, -73,-104, -76,-102, -78,-100, -81, -98, -83, -96, -85, -94, -88, -92,
		-90, -90, -92, -88, -94, -85, -96, -83, -98, -81,-100, -78,-102, -76,-104, -73,
		-106, -71,-107, -68,-109, -65,-111, -63,-112, -60,-113, -57,-115, -54,-116, -51,
		-117, -49,-118, -46,-120, -43,-121, -40,-122, -37,-122, -34,-123, -31,-124, -28,
		-125, -25,-125, -22,-126, -19,-126, -16,-126, -12,-127,  -9,-127,  -6,-127,  -3,
		-127,   0,-127,   3,-127,   6,-127,   9,-126,  12,-126,  16,-126,  19,-125,  22,
		-125,  25,-124,  28,-123,  31,-122,  34,-122,  37,-121,  40,-120,  43,-118,  46,
		-117,  49,-116,  51,-115,  54,-113,  57,-112,  60,-111,  63,-109,  65,-107,  68,
		-106,  71,-104,  73,-102,  76,-100,  78, -98,  81, -96,  83, -94,  85, -92,  88,
		-90,  90, -88,  92, -85,  94, -83,  96, -81,  98, -78, 100, -76, 102, -73, 104,
		-71, 106, -68, 107, -65, 109, -63, 111, -60, 112, -57, 113, -54, 115, -51, 116,
		-49, 117, -46, 118, -43, 120, -40, 121, -37, 122, -34, 122, -31, 123, -28, 124,
		-25, 125, -22, 125, -19, 126, -16, 126, -12, 126,  -9, 127,  -6, 127,  -3, 127};
	
	uint16_t phi = pwm.phi;
	phi += pwm.step;
	pwm.phi = phi;

	int8_t const *p = &sico[2*(uint16_t)(phi >> 8)];

	int8_t isin = pgm_read_byte(p++);
	int8_t icos = pgm_read_byte(p++);

	uint8_t amp = pwm.amp;
	OCR0A = 128 + ((isin*amp)>>8);
	OCR0B = 128 + ((icos*amp)>>8);
}

// update motor calibration coefs (call when motor parameters are changed)
void mot_update_cal(TMotor *mot)
{
	mot->amp_coef = (mot->amp_max - mot->amp_min)/(mot->speed_max - mot->speed_min);
}

// calc motor amplitude based on speed and cal data
uint8_t mot_get_amp(TMotor *mot, float speed)
{
	float amp = (fabs(speed) - mot->speed_min)*mot->amp_coef + mot->amp_min;
	if(amp < mot->amp_min)
	amp = mot->amp_min;
	else if(amp > mot->amp_max)
	amp = mot->amp_max;
	return((uint8_t)amp);
}




//----------------------------------------------------------------------------------
// system timer
//----------------------------------------------------------------------------------

// init controls
void ctrl_init()
{
	// init encoder
	cbi(ENC_ddr, ENC_phy);
	cbi(ENC_ddr, ENA_phy);
	cbi(ENC_ddr, ENB_phy);
	sbi(ENC_port, ENC_phy);
	sbi(ENC_port, ENA_phy);
	sbi(ENC_port, ENB_phy);

	// init buttons
	cbi(SW_A_ddr, SW_A);
	cbi(SW_B_ddr, SW_B);
	cbi(SW_START_ddr, SW_START);
	cbi(SW_STOP_ddr, SW_STOP);
	sbi(SW_A_port, SW_A);
	sbi(SW_B_port, SW_B);
	sbi(SW_START_port, SW_START);
	sbi(SW_STOP_port, SW_STOP);

	// enable pin change ISRs for sleep wakeup
	PCICR = (1<<SW_START_PCIE)|(1<<ENC_PCIE);
	SW_START_PCMSK_REG |= SW_START_PCMSK;
	ENC_PCMSK_REG |= ENC_PCMSK;
}

// controls to idle (for low power)
void ctrl_idle()
{
	// idle encoder
	//cbi(ENC_port, ENC_phy);
	cbi(ENC_port, ENA_phy);
	cbi(ENC_port, ENB_phy);

	// idle buttons
	/*cbi(SW_A_port, SW_A);
	cbi(SW_B_port, SW_B);
	cbi(SW_START_port, SW_START);
	cbi(SW_STOP_port, SW_STOP);*/
}

// pin change stub ISRs
ISR(ENC_ISR)
{
}
ISR(SW_START_ISR)
{
}



// state LED mode
volatile uint8_t led_state;

// init state LED
void led_init()
{
	sbi(LED_ddr, LED_A);
	sbi(LED_ddr, LED_C);
	cbi(LED_port, LED_A);
	cbi(LED_port, LED_C);
	led_state = LED_IDLE;
}

// set state LED state
void led_set(uint8_t state)
{
	if(state == LED_IDLE)
	{
		cbi(LED_port, LED_A);
		cbi(LED_port, LED_C);
	}
	else if(state == LED_RED)
	{
		cbi(LED_port, LED_A);
		sbi(LED_port, LED_C);
	}
	else if(state == LED_GREEN)
	{
		sbi(LED_port, LED_A);
		cbi(LED_port, LED_C);
	}
}

// controls flags
TUI ui;

// process rotary encode sample
void re_sample(uint8_t states, uint8_t ups)
{
	// interval timer (~1ms resolution)
	static uint8_t timer = 0;
	if(timer<0xFFu)
	timer++;
	
	if(bit_is_set(ups, ENA))
	{
		// increment size
		int8_t s = max(max(50 - timer,1)>>2,1);
		
		// reset pulse interval timer
		timer = 0;

		// update value
		ui.enc_delta += (bit_is_set(states,ENB))?-s:+s;
		ui.flags |= (1<<UI_ENC_CHANGE);
	}
}

// LED timeout timer
volatile uint8_t timeout_led;
void set_led_timeout(uint8_t time)
{
	timeout_led = time;
}
uint8_t check_led_timeout(void)
{
	return(!timeout_led);
}

// uvlo timeout
volatile uint8_t uvlo_timeout;
void set_uvlo_timeout(uint8_t time)
{
	uvlo_timeout = time;
}
uint8_t check_uvlo_timeout(void)
{
	return(!uvlo_timeout);
}

// sleep timeout timer
volatile uint16_t timeout_sleep;
void set_sleep_timeout(uint16_t time)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timeout_sleep = time;
	}
}
// get sleep timout residual time
uint16_t get_sleep_timeout()
{
	uint16_t time = 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		time = timeout_sleep;
	}
	return(time);
}
// check if sleep timeout done
uint8_t check_sleep_timeout()
{
	uint16_t time = 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		time = timeout_sleep;
	}
	return(time == 0);
}

// system flags
volatile uint8_t sys_flags;

// system tick
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)
{
	//sei();

	// control states
	uint8_t states = 0x00;
	bcopy(states, SW_A, SW_A_pin, SW_A);
	bcopy(states, SW_B, SW_B_pin, SW_B);
	bcopy(states, SW_START, SW_START_pin, SW_START);
	bcopy(states, SW_STOP, SW_STOP_pin, SW_STOP);
	bcopy(states, ENC, ENC_pin, ENC_phy);
	bcopy(states, ENA, ENC_pin, ENA_phy);
	bcopy(states, ENB, ENC_pin, ENB_phy);
	
	// detect UI changes
	static uint8_t states_last = 0;
	uint8_t edges = states^states_last;
	ui.downs |= edges&(~states);
	uint8_t ups = edges&states;
	ui.states = ~states;
	states_last = states;

	// process encoder rotation
	re_sample(states, ups);

	static uint8_t led_blink = 0;
	static uint8_t tick_blink_div = 0;
	static uint8_t tick_10ms_div = 0;
	static uint8_t tick_1s_div = 0;
	static uint8_t hold_dly[8] = {0,0,0,0,0,0,0,0};
	tick_10ms_div--;
	if(!tick_10ms_div)
	{
		// 10ms tick
		tick_10ms_div = 10;
		sys_flags |= 1<<SYS_10MS_TICK;

		// detect button holdoffs
		uint8_t stats = ~states;
		uint8_t holds = 0;
		for(uint8_t k = 0; k < 8; k++)
		{
			holds >>= 1;
			if(stats & 0x01)
			{				
				if(hold_dly[k] <= BTN_HOLD_TIME)
					hold_dly[k]++;
				if(hold_dly[k] == BTN_HOLD_TIME)
					holds |= 0x80;
			}
			else
				hold_dly[k] = 0;
			stats >>= 1;
		}
		ui.holds |= holds;
				
		tick_blink_div--;
		if(!tick_blink_div)
		{
			tick_blink_div = 5;
			led_blink = !led_blink;
		}

		if(timeout_led)
			timeout_led--;

		tick_1s_div--;
		if(!tick_1s_div)
		{
			tick_1s_div = 100;
			// 1s tick

			if(timeout_sleep)
				timeout_sleep--;

			if(uvlo_timeout)
				uvlo_timeout--;
		}
	}

	// generate LED state
	if(led_state == LED_RED)
		led_set(LED_RED);
	else if(led_state == LED_GREEN)
		led_set(LED_GREEN);
	else if(led_state == LED_GREEN_BLINK && led_blink)
		led_set(LED_GREEN);
	else if(led_state == LED_RED_BLINK && led_blink)
		led_set(LED_RED);
	else if(led_state == LED_RG_BLINK && led_blink)
		led_set(LED_RED);
	else if(led_state == LED_RG_BLINK && !led_blink)
		led_set(LED_GREEN);
	else if(led_state == LED_ORANGE && tick_10ms_div > 8)
		led_set(LED_GREEN);
	else if(led_state == LED_ORANGE)
		led_set(LED_RED);
	else
		led_set(LED_IDLE);

}


// init system timer
void sys_timer_init()
{
	// system tick
	TCCR1A = (0<<WGM10);
	TCCR1B = (1<<WGM12)|(1<<CS00);
	OCR1A = SYS_TICK_DIV;
	TIMSK1 = (1<<OCIE1A);

	sys_flags = 0;
	timeout_led = 0;
	timeout_sleep = 0;
}

//----------------------------------------------------------------------------------
// Other stuff
//----------------------------------------------------------------------------------

// send 0/1
void serial_tx_bool(uint8_t state)
{
	if(state)
		serial_tx_cstr(PSTR("1\n"));
	else
		serial_tx_cstr(PSTR("0\n"));
}

// crc16 CCITT somewhere from internet
//   set input crc to 0xFFFFu before first call
//   call several times for separate parts of data if needed
uint16_t crc16(uint16_t crc, uint8_t* data_p, uint16_t length){

	//u16 crc = 0xFFFF;
	while(length--){
		uint16_t x = crc >> 8 ^ *data_p++;
		x ^= x>>4;
		crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ (x);
	}
	return(crc);
}




//----------------------------------------------------------------------------------
// EEPROM
//----------------------------------------------------------------------------------

#define EEPROM_CAL_ADDR 0 // calibration data offset

// try to load calibration data
uint8_t ee_load_cal(TSetup *data)
{	
	// get record
	TSetup buf;
	uint8_t *addr = (uint8_t*)EEPROM_CAL_ADDR;
	eeprom_read_block((void*)&buf, addr, sizeof(TSetup));
	uint16_t crc = crc16(0xFFFFu, (uint8_t*)&buf, sizeof(TSetup));
	uint16_t ee_crc;
	eeprom_read_block((void*)&ee_crc, addr+sizeof(TSetup), sizeof(uint16_t));
	if(crc != ee_crc)
		return(1);
	memcpy((void*)data, (void*)&buf, sizeof(TSetup));
	return(0);
}

// save calibration data
uint8_t ee_save_cal(TSetup *data)
{	
	uint8_t *addr = (uint8_t*)EEPROM_CAL_ADDR;
	eeprom_write_block((void*)data,addr,sizeof(TSetup));
	uint16_t crc = crc16(0xFFFFu, (uint8_t*)data, sizeof(TSetup));
	eeprom_write_block((void*)&crc,addr+sizeof(TSetup),sizeof(uint16_t));
	return(0);
}


//----------------------------------------------------------------------------------
// MAIN
//----------------------------------------------------------------------------------
int main(void)
{
	// disable WDT if enabled (sometimes it fucks up ...)
	uint8_t rst_flag = MCUSR;
	MCUSR = 0;
	wdt_disable();

	// disable all ports
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;
	
	if(bit_is_set(rst_flag,BORF))
	{
		// brown-out reset
		_delay_ms(200);
	}
		
	led_state = LED_IDLE;
	
	// init ADC
	adc_init();

	// set Vmon alert
	adc_set_vmon_alert(VMON_MIN_ALERT);

	// init PWM controller
	pwm_init();
		
	// serial input init
	serial_init();

	// enable power
	pwm_enable();

	// system timer
	ctrl_init();
	led_init();
	sys_timer_init();
	
	// enable ISR
	sei();

	// motor calibration
	TMotor mot;
	mot.speed = 0.0;
	mot.speed_act = 0.0;
	mot.speed_mem = 0.0;
	mot.acc = 500.0;
	mot.speed_lim = 270.0;
	mot.amp_min = 90.0;
	mot.amp_max = 255.0;
	mot.speed_min = 60.0;
	mot.speed_max = 180.0;
	
	// default system setup
	TSetup setup;
	setup.sleep_timout = SLEEP_TIMEOUT;
	setup.mot_speed_lim = mot.speed_lim;

	// try load EEPROM setup
	if(!ee_load_cal(&setup))
	{
		// CRC passed
		mot.acc = setup.mot_acc;
		mot.speed_lim = setup.mot_speed_lim;
	}
			
	// update motor cal data
	mot_update_cal(&mot);

	// reset sleep timeout
	set_sleep_timeout(setup.sleep_timout);

	// wait for power stage charge
	for(uint16_t k = 0; k < 2000; )
		k += adc_conv_done();
	
	while(1)
	{

		char str[64]; // response buffer
		char cmd[RX_BUF_SZ]; // local command buffer
		char *par;
		uint8_t sleep_reset = 0;
		if(serial_decode(cmd,&par))
		{
			// --- command detected ---
			set_led_timeout(LED_BLINK_TIMEOUT);

			// reset sleep timeout flag
			sleep_reset = 1;
						
			// parse command				    
			if(!strcmp_P(cmd,PSTR("SYST:VMON?")))
			{
				// return system voltage
				float vmon = adc_get_vmon();
				snprintf_P(str, sizeof(str), PSTR("%.2f\n"), vmon);
				serial_tx_str(str);
			}			
			else if(!strcmp_P(cmd,PSTR("SYST:TEMP?")))
			{
				// return system temperature
				float temp = adc_get_temp();
				snprintf_P(str, sizeof(str), PSTR("%.0f\n"), temp);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("SPEED")))
			{
				// set speed (signed rpm value)
				if(!par)
				{
					serial_error(SCPI_ERR_tooFewParameters,PSTR("SPEED <speed> - missing speed value!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				mot.speed = atof(par);
			}
			else if(!strcmp_P(cmd,PSTR("SPEED?")))
			{
				// get current speed (signed rpm value)
				snprintf_P(str, sizeof(str), PSTR("%.1f\n"), mot.speed_act);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("SPEED:SET?")))
			{
				// get target speed (signed rpm value)
				snprintf_P(str, sizeof(str), PSTR("%.1f\n"), mot.speed_mem);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("SPEED:LIMIT")))
			{
				// set speed limit [rpm]
				if(!par)
				{
					serial_error(SCPI_ERR_tooFewParameters,PSTR("MOT:ACC <acc> - missing acceleration value [rpm/s]!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				float lim = atof(par);
				if(lim < 1.0 || lim > 500.0)
				{
					serial_error(SCPI_ERR_wrongParamType,PSTR("SPEED:LIMIT <limit> - allowed range is 1 to 500 rpm!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				mot.speed_lim = lim;
			}
			else if(!strcmp_P(cmd,PSTR("SPEED:LIMIT?")))
			{
				// get speed limit [rpm]
				snprintf_P(str, sizeof(str), PSTR("%.1f\n"), mot.speed_lim);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("STOP")))
			{
				// stop
				mot.speed = 0.0;
			}
			else if(!strcmp_P(cmd,PSTR("START")))
			{
				// start with last known speed
				mot.speed = mot.speed_mem;
			}
			else if(!strcmp_P(cmd,PSTR("DIR")))
			{
				// switch direction (or restart from stop)
				if(!par)
				{
					serial_error(SCPI_ERR_tooFewParameters,PSTR("DIR <direction> - missing direction value!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				if(strcmp_P(par,PSTR("CW")) == 0)
					mot.speed = fabs(mot.speed_mem);
				else if(strcmp_P(par,PSTR("CCW")) == 0)
					mot.speed = -fabs(mot.speed_mem);
				else
				{
					serial_error(SCPI_ERR_wrongParamType,PSTR("DIR <direction> - invalid direction value! Use CW or CCW."),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
			}
			else if(!strcmp_P(cmd,PSTR("SLEEP")))
			{
				// goto sleep now
				sleep_reset = 0;
				set_sleep_timeout(SLEEP_TIMEOUT_NOW);
			}
			else if(!strcmp_P(cmd,PSTR("SLEEP:TIMEOUT")))
			{
				// set sleep timeout [s]
				if(!par)
				{
					serial_error(SCPI_ERR_tooFewParameters,PSTR("SLEEP:TIMEOUT <timeout> - missing timeout value [s]!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				setup.sleep_timout = atoi(par);
			}
			else if(!strcmp_P(cmd,PSTR("SLEEP:TIMEOUT?")))
			{
				// get sleep timeout [s]
				snprintf_P(str, sizeof(str), PSTR("%u\n"), setup.sleep_timout);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("MOT:ACC")))
			{
				// set motor acceleration [rpm/s]
				if(!par)
				{
					serial_error(SCPI_ERR_tooFewParameters,PSTR("MOT:ACC <acc> - missing acceleration value [rpm/s]!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				float acc = atof(par);
				if(acc < 100.0 || acc > 2000.0)
				{
					serial_error(SCPI_ERR_wrongParamType,PSTR("MOT:ACC <acc> - allowed range is 100 to 2000 rpm/s!"),SCPI_ERR_STORE|SCPI_ERR_PSTR);
					goto SCPI_error;
				}
				mot.acc = acc;
			}
			else if(!strcmp_P(cmd,PSTR("MOT:ACC?")))
			{
				// get motor acceleration [rpm/s]
				snprintf_P(str, sizeof(str), PSTR("%.1f\n"), mot.acc);
				serial_tx_str(str);
			}
			else if(!strcmp_P(cmd,PSTR("SYST:EESAVE")))
			{
				// store current setup to EEPROM
				setup.mot_acc = mot.acc;
				setup.mot_speed_lim = mot.speed_lim;
				ee_save_cal(&setup);
			}
			else if(!strcmp_P(cmd,PSTR("*OPC?")))
			{
				// *OPC? - returns 1 when HW is set
				serial_tx_cstr(PSTR("1\n"));
			}
			else if(!strcmp_P(cmd,PSTR("*IDN?")))
			{
				// "*IDN?" to return IDN string
				serial_tx_cstr(PSTR("Merkur Motor controller by Stanislav Maslan V1.0, " __DATE__ "\n"));
			}			
			else if(!strcmp_P(cmd,PSTR("SYST:ERR?")))
			{
				// SYST:ERR? - return last error
				serial_error(SCPI_ERR_undefinedHeader,NULL,SCPI_ERR_SEND);
			}
			/*else if(!strcmp_P(cmd,PSTR("SYST:BOOT")))
			{
				// SYST:BOOT <password> - initiates bootloader

				// send ACK/NACK
				if(par && !strcmp_P(par,PSTR("17IND10")))
				{
					// send ACK
					serial_tx_cstr(PSTR("1\n"));

					// get extended fuse bits
					uint8_t fuex = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);

					// extract bootsize index (FUSE_BOOTSZ1:FUSE_BOOTSZ0)
					fuex = !!(fuex&(~FUSE_BOOTSZ0)) + (!!(fuex&(~FUSE_BOOTSZ1))<<1);

					// calculae bootloader start address in [Bytes]
					uint16_t addr = FLASHEND + 1 - (1024ul<<(3 - fuex));
								
					// jump to boot section
					cli();
					boot_start(addr);
				}
				else
					serial_tx_cstr(PSTR("0\n"));
			}*/
			else if(!strcmp_P(cmd,PSTR("CONNECTED")))
			{
				// fake command produced by BT module - ignore
			}
			else
			{
				// invalid
				serial_error(SCPI_ERR_undefinedHeader,cmd,SCPI_ERR_STORE|SCPI_ERR_STR);
				goto SCPI_error;
			}

			// jump site when error occurred
			SCPI_error:;
				    
		}



		// process ADC data
		adc_conv_done();

		static uint8_t uvlo_cycle = 0;
		if(adc_check_vmon())
		{
			// undervoltage detected
			led_state = LED_RED_BLINK;
			
			// stop motor
			mot.speed = 0.0;

			// start detection timeout
			if(check_uvlo_timeout())
			{
				set_uvlo_timeout(UVLO_DET_TIMEOUT);

				// uvlo cycle
				uvlo_cycle++;
				if(uvlo_cycle > 1)
				{
					set_sleep_timeout(SLEEP_TIMEOUT_NOW);
				}
			}
			sleep_reset = 0;
		}
		else if(!check_led_timeout())
		{
			led_state = LED_GREEN_BLINK;
		}
		else if(!check_uvlo_timeout())
		{
			led_state = LED_RED_BLINK;
			sleep_reset = 0;
		}
		else
		{
			led_state = LED_GREEN;
			uvlo_cycle = 0;
		}
		

		// process UI controls
		uint8_t flags = 0;
		uint8_t ui_flags;
		uint8_t ui_downs;
		uint8_t ui_holds;
		int8_t enc_delta = 0;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			flags = sys_flags; sys_flags = 0;
			ui_flags = ui.flags; ui.flags = 0;
			ui_downs = ui.downs; ui.downs = 0;
			ui_holds = ui.holds; ui.holds = 0;
			enc_delta = ui.enc_delta; ui.enc_delta = 0;
		}
		if(ui_downs)
			sleep_reset = 1;
		if(bit_is_set(ui_flags, UI_ENC_CHANGE))
		{
			// encoder move
			mot.speed -= (float)enc_delta*5.0;
			sleep_reset = 1;
		}
		if(bit_is_set(ui_downs, SW_A))
		{
			// left
			mot.speed = -fabs(mot.speed_mem);
		}
		if(bit_is_set(ui_downs, SW_B))
		{
			// right
			mot.speed = fabs(mot.speed_mem);
		}
		if(bit_is_set(ui_downs, SW_START))
		{
			// start
			mot.speed = mot.speed_mem;
		}
		if(bit_is_set(ui_downs, SW_STOP))
		{
			// stop
			mot.speed = 0.0;
		}
		if(bit_is_set(ui_holds, SW_STOP))
		{
			// goto sleep
			sleep_reset = 0;
			set_sleep_timeout(SLEEP_TIMEOUT_NOW);
		}
		
		// process motor acceleration stuff
		static uint8_t is_halt = 0;
		if(bit_is_set(flags, SYS_10MS_TICK))
		{
			// 10ms tick

			if(mot.speed > mot.speed_lim)
				mot.speed = mot.speed_lim;
			else if(mot.speed < -mot.speed_lim)
				mot.speed = -mot.speed_lim;

			if(mot.speed != 0.0)
				mot.speed_mem = mot.speed;
			
			float acc = 0.01*mot.acc;
			if(mot.speed_act < mot.speed)
				mot.speed_act = min(mot.speed_act + acc, mot.speed);
			else if(mot.speed_act > mot.speed)
				mot.speed_act = max(mot.speed_act - acc, mot.speed);			
			
			const float rpm_to_inc = 200.0/4.0/((float)F_CPU/256.0/256.0/256.0)/60.0;
			
			uint8_t amp = mot_get_amp(&mot, mot.speed_act);
			int16_t spd = (int16_t)(mot.speed_act*rpm_to_inc);
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				pwm.amp = amp;
				pwm.step = spd;
			}

			// motor in halt?
			is_halt = (mot.speed_act == 0.0);
		}

		
		// reset sleep timeout?
		if(sleep_reset)
			set_sleep_timeout(setup.sleep_timout);
					
		// goto sleep?
		if(check_sleep_timeout())
		{
			// timeout reached - go to sleep
			if(!is_halt)
			{
				// motor is running - stop it first
				mot.speed = 0.0;
			}
			else
			{
				// motor stopped - we can go to sleep
								
				// disable stuff
				led_state = LED_ORANGE;
				ctrl_idle();				
				serial_idle();

				// idle motor current, disable power
				pwm_idle();
	
				// wait till power drops
				while(adc_get_vmon() > VMON_POWER_IDLE)
					adc_conv_done();					

				// disable PWM
				pwm_disable();
				led_state = LED_IDLE;

				// stop ADC
				adc_idle();
				
				// wait a bit
				_delay_ms(50);
				
				// go to sleep
				cli();
				MCUCR = (1<<BODSE)|(1<<BODS);
				MCUCR = (1<<BODS);				
				set_sleep_mode(SLEEP_MODE_PWR_DOWN); // power down
				sleep_enable();
				sleep_bod_disable();
				sei();
				sleep_cpu();
				sleep_disable();								

				// re-enable stuff
				serial_init();
				pwm_enable();
				ctrl_init();
				adc_init();
				adc_set_vmon_alert(VMON_MIN_ALERT);

				// wait for power stage charge
				for(uint16_t k = 0; k < 2000; )
					k += adc_conv_done();

				// start in stopped mode
				mot.speed = 0.0;
				ui.downs = 0;
				
				// reset sleep timeout
				uvlo_cycle = 0;
				set_uvlo_timeout(0);
				set_sleep_timeout(setup.sleep_timout);
			}
		}
		

	}

}
