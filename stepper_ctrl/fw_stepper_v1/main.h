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

#ifndef MAIN_H
#define MAIN_H

// general macros
#define sbi(port,pin) {(port)|=(1<<pin);}
#define cbi(port,pin) {(port)&=~(1<<pin);}
#define min(a,b) ((a<b)?(a):(b))
#define max(a,b) ((a>b)?(a):(b))
#define bcopy(dreg,dpin,srreg,srpin) if(bit_is_set((srreg),(srpin))){sbi(dreg,dpin);}else{cbi(dreg,dpin);}
#define bcopy_v(dreg,dpin,srreg,srpin) if((srreg)&(1<<srpin)){sbi(dreg,dpin);}else{cbi(dreg,dpin);}
#define low(x) ((x) & 0xFFu)
#define high(x) (((x)>>8) & 0xFFu)

// PWR control (active high)
#define PWR_port PORTD
#define PWR_ddr DDRD
#define PWR PD3

// control switches (active low)
#define SW_A_port PORTB
#define SW_A_ddr DDRB
#define SW_A_pin PINB
#define SW_A PB0
#define SW_B_port PORTD
#define SW_B_ddr DDRD
#define SW_B_pin PIND
#define SW_B PD7
#define SW_START_port PORTB
#define SW_START_ddr DDRB
#define SW_START_pin PINB
#define SW_START PB2
#define SW_STOP_port PORTB
#define SW_STOP_ddr DDRB
#define SW_STOP_pin PINB
#define SW_STOP PB1
#define SW_START_PCIE PCIE0
#define SW_START_PCMSK_REG PCMSK0
#define SW_START_PCMSK (1<<SW_START)
#define SW_START_ISR PCINT0_vect

// encoder (active low)
#define ENC_port PORTC
#define ENC_ddr DDRC
#define ENC_pin PINC
#define ENC_phy PC2
#define ENA_phy PC4
#define ENB_phy PC3
#define ENC 3
#define ENA 4
#define ENB 5
#define ENC_PCIE PCIE1
#define ENC_PCMSK_REG PCMSK1
#define ENC_PCMSK (1<<ENC_phy)
#define ENC_ISR PCINT1_vect

// BT state (active high)
#define BT_STATE_port PORTC
#define BT_STATE_ddr DDRC
#define BT_STATE_pin PINC
#define BT_STATE PC5

// serial port
#define UART_port PORTD
#define UART_ddr DDRD
#define UART_RXD PD0
#define UART_TXD PD1

// state LED (dual color)
#define LED_port PORTC
#define LED_ddr DDRC
#define LED_A PC1
#define LED_C PC0

// pulse outputs
#define PWM_port PORTD
#define PWM_ddr DDRD
#define PWM_L1 PD6
#define PWM_L2 PD5

// analogue inputs
#define ADC_VMON_MUX 0x07
#define ADC_TEMP_MUX 0x08
// ADC averaging
#define ADC_AVG 32 /* max 10bit*64 to scale to full 16bit */

// ADC data and stuff
typedef struct{
	volatile uint16_t a_vmon;
	volatile uint16_t a_temp;
	uint16_t vmon_alert;
}TADC;

// state LED modes
#define LED_IDLE 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_RED_BLINK 3
#define LED_GREEN_BLINK 4
#define LED_RG_BLINK 5
#define LED_ORANGE 6

// system tick flag
#define SYS_10MS_TICK 0

// controls struct
typedef struct{
	volatile uint8_t downs;
	volatile uint8_t states;
	volatile uint8_t holds;
	volatile int8_t enc_delta;
	volatile uint8_t flags;
}TUI;

// encoder state changed (must not collide with button/encoder flags)
#define UI_ENC_CHANGE 6

// system setup to store to EEPROM
typedef struct{
	uint16_t sleep_timout;
	float mot_acc;
	float mot_speed_lim;
}TSetup;

// motor control
typedef struct{
	float speed;
	float speed_mem;
	float acc;
	float speed_act;
	float speed_lim;
	float speed_min;
	float speed_max;
	float amp_min;
	float amp_max;
	float amp_coef;
}TMotor;

// main system timer
#define SYS_TICK 0.001
#define SYS_TICK_DIV ((uint16_t)(F_CPU*SYS_TICK - 1))

// button hold off time [10ms]
#define BTN_HOLD_TIME 70

// battery undervolt alert voltage [V]
#define VMON_MIN_ALERT 4.2

// LED blink timeout for serial command [10ms]
#define LED_BLINK_TIMEOUT 25

// default sleep timeout [s]
#define SLEEP_TIMEOUT 60

// uvlo detection timeout [s]
#define UVLO_DET_TIMEOUT 5

// undervolt sleep timeout [s]
#define SLEEP_TIMEOUT_UVLO 3

// immediate sleep timeout [s]
#define SLEEP_TIMEOUT_NOW 0

// power stage voltage to allow sleep [V]
#define VMON_POWER_IDLE 2.0


#endif
