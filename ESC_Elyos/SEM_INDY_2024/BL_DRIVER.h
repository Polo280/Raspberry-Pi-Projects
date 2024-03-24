#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

// LIBRARIES
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// GATE DRIVER PINS
#define AH_PIN 16
#define AL_PIN 17
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 20
#define CL_PIN 21

// PWM SLICES (16 bit resolution = 65535 steps)
#define A_PWM_SLICE 0
#define B_PWM_SLICE 1
#define C_PWM_SLICE 2

// HALL SENSORS
#define HALL1_PIN 13
#define HALL2_PIN 14
#define HALL3_PIN 15
#define HALL_SAMPLES 8
#define HALL_THRESHOLD (HALL_SAMPLES / 2)

// ADC PINS (12 bit resolution = 4096 steps)
#define ISENSE_PIN 26     // ADC 0
#define VSENSE_PIN 27     // ADC 1
#define THROTTLE_PIN 28   // ADC 2

// STATUS LED
#define LED_PIN 25
#define LED_ON 1
#define LED_OFF 0

// PWM MACROS
#define PWM_FREQUENCY_HZ 20000   // ~ 2 Khz minimum
#define PWM_WRAP_VALUE 1023

// THROTTLE MACROS
#define THROTTLE_MIN 500
#define THROTTLE_MAX 3000

// TIME MACROS
#define TIME_TO_BLINK_US 1000000

// CONTROL VARIABLES  
volatile uint16_t adc_isense;  // (12 bit ADC)
volatile uint16_t adc_vsense;
volatile uint16_t adc_throttle; 

// Hall auxiliary variables
uint8_t *hall_states[3];
uint8_t *hall_count;

uint8_t* duty_cycle = 0;
uint8_t current_amps = 0;

// AUX VARIABLES
uint64_t time_aux1 = 0;
bool run_motor = false;
bool is_moving = false;

///////////// FUNCTIONS /////////////

/**
 * @brief Configure and intialize peripherals/interrupts
 * @return None
*/
void Setup(void);
/**
 * @brief Read throttle value with ADC
*/
void ThrottleFilter(void);
/**
 * @brief A method used when something goes crazy
*/
void ErrorHandler(void);
/**
 * @brief Get current motor state with hall sensors
 * @return Integer storing current motor state
*/
void GetHalls(void);
/**
 * @brief Map ADC throttle reading to PWM duty cycle and identify motor state through halls
 * @return None
*/
void WritePWM(uint16_t duty_cycle, uint8_t hall_state, bool synchronous);
/**
 * @brief Power with PWM to motor phases
 * @param ah The PWM value for phase A high side.
 * @param bh The PWM value for phase B high side.
 * @param ch The PWM value for phase C high side.
 * @param al The PWM value for phase A low side.
 * @param bl The PWM value for phase B low side.
 * @param cl The PWM value for phase C low side.
 * @return None
*/
void PowerPhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl);

// INTERRUPTS //
/**
 * @brief Interrupt to handle PWM
*/
void PwmInterrupt(void);
/**
 * @brief Interrupt to handle ADC readings
*/
void AdcInterrupt(void);

/////////////////////////////////////

#endif /*BLDC_DRIVER_H*/