/**
 * @file motor_control.c
 * @brief Motor control code using Raspberry Pi Pico.
 */

#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

// Master control 
bool runMotor = true;

//Flags
uint FLAG_PIN = 2;
uint FLAG_PIN_ADC = 4;

bool isMoving = false;

//LED
const uint LED_PIN = 25;

// Pin definitions
const uint AH_PIN = 16;     // PWM Slice 0A
const uint AL_PIN = 17;     // PWM Slice 0B
const uint BH_PIN = 18;     // PWM Slice 1A
const uint BL_PIN = 19;     // PWM Slice 1B
const uint CH_PIN = 20;     // PWM Slice 2A
const uint CL_PIN = 21;     // PWM Slice 0A

// Hall sensor pins
const uint HALL_1_PIN = 13;
const uint HALL_2_PIN = 14;
const uint HALL_3_PIN = 15;

//ADC pins
const uint ISENSE_PIN = 26;     //ADC 0
const uint VSENSE_PIN = 27;     //ADC 1
const uint THROTTLE_PIN = 28;   //ADC 2

// Constants
const uint HALL_OVERSAMPLE = 8;
const int HALL_IDENTIFY_DUTY_CYCLE = 25;
const uint F_PWM = 16000;
const int DUTY_CYCLE_MAX = 65535;
const int THROTTLE_LOW = 550;
const int THROTTLE_HIGH = 2600;
const int FULL_SCALE_CURRENT_MA = 3000;

// Constants
const int DUTY_CYCLE_STEP = 1;  // Step size for increasing/decreasing the duty cycle

//PWM Slices
const uint A_PWM_SLICE = 0;
const uint B_PWM_SLICE = 1;
const uint C_PWM_SLICE = 2;

//Proportions
const int CURRENT_SCALING = 3.3 / 0.001 / 20 / 4096 * 1000;
const int VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;

//Global variables
int adc_isense = 0;
int adc_vsense = 0;
int adc_throttle = 0;
int adc_bias = 0;
int duty_cycle = 0;     // Progresive duty 
int target_duty_cycle = 0;
int voltage_mv = 0;
int current_ma = 0;
int current_target_ma = 0;
uint64_t ticks_since_moving = 0;
uint64_t ticks_since_init = 0;
uint hallCounter = 0;
uint currentHall = 0;

// Control variables
const uint16_t SAFE_TIME_TO_INIT = 5000;     // Safe time to stabilize
const int CURRENT_LIMIT      = 5000;         // Limit current in mA
const int ADC_HIGH_SCALED_THRESHOLD = 166;   // ADC High threshold (throttle)
const int PWM_LOW_THRESHOLD = 35;            // PWM LOW Threshold
int SCALING_FACTOR_ADC = 0;                  // Scaling factor ADC (for variation 0 - 255)
int adc_scaled_thresh = 0;                   // Low threshold  ADC
int prev_duty = 0;                           // Previous duty cycle    
int isense_bias = 0;                         // Bias (threshold) for current sensor 

int debugThrottle = 0;

bool safe_to_init = false;                   // Is it stable?
bool exceeded_current_limit = false;         // Has current limit been exceeded?

//  ENCODER VARIABLES
int64_t hall_pulse_time = 0;
int64_t half_hall_pulse_time = 0;
int64_t prev_update_tick = 0;

absolute_time_t thicks_hall_begin;
absolute_time_t thicks_hall_end;
uint16_t pwm_triangle_ticks = 0;
uint8_t pwm_triangle_step = 0;

float rpm = 0;

/**
 * @brief Get the hall sensor state.
 *
 * This function reads the hall sensor pins and determines the hall sensor state. It returns the hall sensor state as an
 * unsigned integer. The hall sensor state represents the position of the motor's rotor.
 *
 * @return The hall sensor state as an unsigned integer.
 */
uint get_halls();

/**
 * @brief Initialize the hardware setup.
 */
void setup();

/**
 * @brief Identify the hall sensor states and store the results in the hallToMotor array.
 */
void identify_halls();

/**
 * @brief Set the PWM duty cycle for a specific motor state.
 *
 * This function sets the PWM duty cycle for a specific motor state. It takes the motor state, duty cycle value, and a
 * boolean flag indicating whether synchronous switching should be enabled. The motor state represents the desired state
 * of the motor, the duty cycle value represents the magnitude of the PWM signal, and the synchronous flag indicates
 * whether to enable synchronous switching. The function adjusts the duty cycle based on the current and target currents
 * and writes the PWM signals accordingly.
 *
 * @param halls Hall configuration as an unsigned integer 
 * @param duty The duty cycle value as an unsigned integer.
 * @param synchronous A boolean flag indicating whether synchronous switching should be enabled.
 */
void writePWM(uint halls, int duty, bool synchronous);

/**
 * @brief Write the PWM values to the motor phases.
 * @param ah The PWM value for phase A high side.
 * @param bh The PWM value for phase B high side.
 * @param ch The PWM value for phase C high side.
 * @param al The PWM value for phase A low side.
 * @param bl The PWM value for phase B low side.
 * @param cl The PWM value for phase C low side.
 */
void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl);

/**
 * @brief PWM wrap interrupt handler.
 */
void on_pwm_wrap();

/**
 * @brief Callback function for handling ADC FIFO interrupts.
 *
 * This function is called when an ADC FIFO interrupt occurs. It reads the ADC values for current, voltage, and throttle,
 * calculates the motor state, adjusts the duty cycle based on the current and target currents, and writes the PWM
 * signals accordingly. The function also updates relevant variables and flags. It assumes the ADC FIFO interrupt is
 * correctly triggered and configured.
 */
void on_adc_fifo();

/**
 * @brief Read the throttle value from the ADC
 *
 * This function reads and returns the throttle value from the ADC.
 *
 * @return The throttle value
 */
int read_throttle();

// Adjust actual duty cycle to desired through steps 
void adjustDuty();

// Calculate the triangular wave for PWM duty cycle 
int8_t pwm_calc(int);

// Callback for hall sensors changing state 
void hall_sensor_interrupt_handler(uint, uint32_t);

int main(){
    // Setup peripherals
    setup(); 
    thicks_hall_begin = get_absolute_time();

    // MAIN LOOP
    while (true)
    {
        // Update the watchdog each cycle
        watchdog_update();

        // LED
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Control variables
        safe_to_init = to_ms_since_boot(get_absolute_time()) >= SAFE_TIME_TO_INIT;

        // // If overcurrent event occurs reset the board
        // if(current_ma > 5){ 
        //     safe_to_init = false;
        //     sleep_us(250);
        //     if(current_ma > 5){
        //         exceeded_current_limit = true; 
        //         watchdog_reboot(0, 0, 0);   // DO NOT put a delay in the watchdog, it must operate instantly, if not the code crashes
        //     }
        // }

        // printf("Current: %d\n", debugThrottle);

        SCALING_FACTOR_ADC = (ADC_HIGH_SCALED_THRESHOLD - adc_scaled_thresh);
    }
    return 0;
}

uint get_halls() {
    uint hallCounts[] = {0, 0, 0};

    // Read all the hall pins repeatedly and tally the results
    for (uint i = 0; i < HALL_OVERSAMPLE; i++) {
        hallCounts[0] += gpio_get(HALL_1_PIN);
        hallCounts[1] += gpio_get(HALL_2_PIN);
        hallCounts[2] += gpio_get(HALL_3_PIN);
    }

    uint hall = 0;

    // If votes >= threshold, set the corresponding bit to 1
    if (hallCounts[0] > HALL_OVERSAMPLE / 2)
        hall |= (1 << 0);
    if (hallCounts[1] > HALL_OVERSAMPLE / 2)
        hall |= (1 << 1);
    if (hallCounts[2] > HALL_OVERSAMPLE / 2)
        hall |= (1 << 2);

    return hall;
}

void setup() {

    stdio_init_all();

    // Enable watchdog with a timer
    watchdog_enable(900, 1);

    //  Initialize Led
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    //  Initialize flag pin
    gpio_init(FLAG_PIN);
    gpio_init(FLAG_PIN_ADC);

    gpio_set_dir(FLAG_PIN, GPIO_OUT);
    gpio_set_dir(FLAG_PIN_ADC, GPIO_OUT);

    //  Initialize hall sensor pins
    gpio_init(HALL_1_PIN);
    gpio_set_dir(HALL_1_PIN, GPIO_IN);
    gpio_init(HALL_2_PIN);
    gpio_set_dir(HALL_2_PIN, GPIO_IN);
    gpio_init(HALL_3_PIN);
    gpio_set_dir(HALL_3_PIN, GPIO_IN);

    gpio_set_irq_enabled_with_callback(HALL_1_PIN, GPIO_IRQ_EDGE_RISE, true, &hall_sensor_interrupt_handler);
    gpio_set_irq_enabled_with_callback(HALL_2_PIN, GPIO_IRQ_EDGE_RISE, true, &hall_sensor_interrupt_handler);
    gpio_set_irq_enabled_with_callback(HALL_3_PIN, GPIO_IRQ_EDGE_RISE, true, &hall_sensor_interrupt_handler);

    //  Initialize predriver PWM pins
    gpio_set_function(AH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(AL_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BL_PIN, GPIO_FUNC_PWM);
    gpio_set_function(CH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(CL_PIN, GPIO_FUNC_PWM);

    //  PWM configuration
    float pwm_divider = (float)(clock_get_hz(clk_sys)) / (F_PWM * 255 * 2);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, pwm_divider);
    pwm_config_set_wrap(&config, 255 - 1);
    pwm_config_set_phase_correct(&config, true);
    pwm_config_set_output_polarity(&config, false, true);

    pwm_init(A_PWM_SLICE, &config, false);
    pwm_init(B_PWM_SLICE, &config, false);
    pwm_init(C_PWM_SLICE, &config, false);

    pwm_set_mask_enabled(0x07);

    //  Initialize IRQ
    pwm_clear_irq(A_PWM_SLICE);
    pwm_set_irq_enabled(A_PWM_SLICE, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_priority(PWM_IRQ_WRAP, 0);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    //  Initialize ADC
    adc_init();
    adc_gpio_init(ISENSE_PIN);  
    adc_gpio_init(VSENSE_PIN);
    adc_gpio_init(THROTTLE_PIN);
    adc_set_round_robin(0b111);
    adc_fifo_setup(true, false, 3, false, false);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, on_adc_fifo);
    irq_set_priority(ADC_IRQ_FIFO, 0);
    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    //  Adjust isense
    sleep_ms(200);
    for(uint i = 0; i < 127; i++) {
        adc_bias += adc_isense;   //Bias es el error cuando no hay corriente circulando por el motor 
        sleep_ms(5);
    }
    adc_bias /= 128;     // Calculate average value of 128 samplings
}

void identify_halls()
{
    int states[6] = {1, 3, 2, 6, 4, 5};
    for(uint i = 0; i < 6; i++)
    {
        int nextState = (i + 1) % 6;        // Calculate what the next state should be. This is for switching into half-states
        for(uint j = 0; j < 500; j++)       // For a while, repeatedly switch between states
        {
            sleep_ms(1);
            writePWM(i, HALL_IDENTIFY_DUTY_CYCLE, true);
            sleep_ms(1);
            writePWM(nextState, HALL_IDENTIFY_DUTY_CYCLE, true);
        }
    }
    writePWM(0, 0, true);                           // Turn phases off
}

void writePWM(uint halls, int duty, bool synchronous) {
    duty = (duty - PWM_LOW_THRESHOLD);  // Remove noise

    // Bound to 0 - 255
    duty = (duty < 0)? 0 : duty;
    duty = (duty > 255)? 255 : duty;

    // Update duty gradually
    if(duty > prev_duty){
        duty = prev_duty + DUTY_CYCLE_STEP;
    }else if(duty < prev_duty){
        duty = prev_duty - DUTY_CYCLE_STEP;
    }

    // Update previous duty
    prev_duty = duty;

    uint complement = 0;
    if(synchronous)
        complement = 255 - duty - 6;   // -6 to avoid overlapping pwms causing a short

    switch(halls){
        case 1: // Case 001
           writePhases(duty, 0, 0, complement, 0, 255);  //writePhases(0, 0, duty, 255, 0, 0);
           break;
        case 2: // Case 010
           writePhases(0, 0, duty, 0, 255, complement);  //writePhases(0, duty, 0, 0, 0, 255);
           break;
        case 3: // Case 011
           writePhases(duty, 0, 0, complement, 255, 0);  //writePhases(0, duty, 0, 255, 0, 0);
           break;
        case 4: // Case 100
           writePhases(0, duty, 0, 255, complement, 0);  //writePhases(duty, 0, 0, 0, 255, 0);
           break;
        case 5: // Case 101
           writePhases(0, duty, 0, 0, complement, 255);  //writePhases(0, 0, duty, 0, 255, 0);
           break;
        case 6: // Case 110
           writePhases(0, 0, duty, 255, 0, complement);  //writePhases(duty, 0, 0, 0, 0, 255);
           break;
        default: // Case 000 or error 
           writePhases(0, 0, 0, 0, 0, 0);
    }
}


void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl) {
    if(safe_to_init && !exceeded_current_limit){
        pwm_set_both_levels(A_PWM_SLICE, ah, 255 - al);
        pwm_set_both_levels(B_PWM_SLICE, bh, 255 - bl);
        pwm_set_both_levels(C_PWM_SLICE, ch, 255 - cl);
    }else{
        pwm_set_both_levels(A_PWM_SLICE, 0, 0);
        pwm_set_both_levels(B_PWM_SLICE, 0, 0);
        pwm_set_both_levels(C_PWM_SLICE, 0, 0);
    }
}


void on_pwm_wrap() {
    // gpio_put(FLAG_PIN, 1);
    adc_select_input(0);
    adc_run(true);
    pwm_clear_irq(A_PWM_SLICE);
    while(!adc_fifo_is_empty())
        adc_fifo_get();
    // gpio_put(FLAG_PIN, 0);
}


void on_adc_fifo() {
    adc_run(false);     // Stops sampling on adc interruption
    
    // Read adcs from fifo
    adc_isense = adc_fifo_get();
    adc_vsense = adc_fifo_get();
    adc_throttle = adc_fifo_get();
    debugThrottle = adc_throttle;

    uint hall = get_halls();          // 200 nanoseconds, read from hall states and store values in uint variable (which ranges from 1-6)
    
    // Get duty cycle
    int throttle = read_throttle();
    // debugThrottle = throttle;

    // Map analog input values
    adc_isense = (adc_isense - adc_bias) / (4095 - adc_bias);
    current_ma = (165 * adc_isense / 10);   // Current in miliAmps

    current_ma = (current_ma < 0)? 0 : current_ma;

    // current_target_ma = throttle * FULL_SCALE_CURRENT_MA / 256;
    voltage_mv = adc_vsense * VOLTAGE_SCALING;
    
    writePWM(get_halls(), pwm_calc(throttle) , true);
}

void hall_sensor_interrupt_handler(uint gpio, uint32_t events) {
    thicks_hall_end = get_absolute_time();
    int64_t time_difference_us = absolute_time_diff_us(thicks_hall_begin, thicks_hall_end);
    if (time_difference_us < 0) {
        /*no se que ocupe aqui*/
        return;
    }
    thicks_hall_begin = get_absolute_time();
    hall_pulse_time = time_difference_us;
    half_hall_pulse_time = hall_pulse_time >> 1;

    pwm_triangle_step = 255 / half_hall_pulse_time;    // Slope formula (y2 - y1) / (x2 - x1)
    pwm_triangle_ticks = hall_pulse_time >> 8;         // Separate in 128 sections each slope , number of ticks for each section
}

int8_t pwm_calc(int throttle_duty){
    int8_t pwm_vector = 0;
    absolute_time_t thicks_hall_now;
    thicks_hall_now = get_absolute_time();
    int64_t time_difference_us_now = absolute_time_diff_us(thicks_hall_begin, thicks_hall_now);
    
    // If its time to update triangle slope 
    if(time_difference_us_now < half_hall_pulse_time){
        /*Logica de subida*/ 
        pwm_vector = time_difference_us_now * pwm_triangle_step;
    }
    else{
        /*Complemento para bajada*/
        pwm_vector = 255 - time_difference_us_now * pwm_triangle_step;
    }
    return (pwm_vector * throttle_duty) >> 8;
}

int    read_throttle()
{
    if(safe_to_init){   // If system is already stable
        adc_scaled_thresh = (adc_scaled_thresh == 0)? (adc_throttle >> 4): adc_scaled_thresh;
        adc_scaled_thresh = (adc_scaled_thresh > 170)? 170 : adc_scaled_thresh;  // Bound high limit threshold
        int adc = adc_throttle >> 4;                                     // Bound ADC reading to 0 - 255
        adc = ((adc - adc_scaled_thresh) << 8) / (SCALING_FACTOR_ADC);  // Adjust adc reading to thresholds
        return (adc < 0)? 0 : adc;
    }
    return 0;
}