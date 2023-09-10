// Fariha Rabbi
// Embedded Systems II
// Final Project

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "nvic.h"
#include "adc0.h"
#include "uart0.h"
#include "wait.h"

// Pins
#define MIC1             PORTD,3
#define MIC2             PORTE,1
#define MIC3             PORTE,2
#define MIC4             PORTE,3

#define RED_LED          PORTF,1
#define BLUE_LED         PORTF,2
#define GREEN_LED        PORTF,3

// masks
#define MIC_1_SAMPLE     0,0x4
#define MIC_2_SAMPLE     1,0x2
#define MIC_3_SAMPLE     2,0x1
#define MIC_4_SAMPLE     3,0x8

#define MIC_1_OFFSET     50
#define MIC_2_OFFSET     40
#define MIC_3_OFFSET     10

#define INTERRUPT_MASK   30

#define VRMS             0.00631
#define SENSITIVITY      44
#define SPL_REF          94
#define GAIN             40
#define k                0.1


// global variables
uint16_t mic1_raw;
uint16_t mic2_raw;
uint16_t mic3_raw;

uint16_t mic1_avg;
uint16_t mic2_avg;
uint16_t mic3_avg;

uint32_t mic1_avg_SPL;
uint32_t mic2_avg_SPL;
uint32_t mic3_avg_SPL;

uint16_t mic1_samples[20];
uint16_t mic2_samples[20];
uint16_t mic3_samples[20];

uint16_t mic1_time;
uint16_t mic2_time;
uint16_t mic3_time;

uint16_t mic1_time_store;
uint16_t mic2_time_store;
uint16_t mic3_time_store;

float theta;
uint32_t tc;
uint32_t backoff;
uint32_t new_holdoff = 0;
uint32_t holdoff;
uint32_t track_time = 0;
uint32_t track_event = 0;
uint32_t tc_count = 0;

uint8_t peak_val;
uint8_t peak_mic;
uint16_t origin;

uint16_t T1;
uint16_t T2;

bool tdoa;;
bool fail;;
bool aoa;
bool detect_event = false;
bool event_detected;;
bool detection_fail = false;
bool peak_found = false;
bool H_flag = false;
bool aoa_always = true;


bool mic1_origin;
bool mic2_origin;
bool mic3_origin;

USER_DATA data;
char str[40];


//-----------------------------------------------------------------------------
// Interrupt Service Routine
//-----------------------------------------------------------------------------
uint8_t detectPeak(uint16_t mic1, uint16_t mic2, uint16_t mic3)
{
    if (mic1 < mic2 && mic1 < mic3)
    {
        origin = 0;
        peak_val = 1;
    }
    if (mic2 < mic1 && mic2 < mic3)
    {
        origin = 120;
        peak_val = 2;
    }
    if (mic3 < mic1 && mic3 < mic2)
    {
        origin = 240;
        peak_val = 3;
    }

    if (peak_val == 1)
    {
        if (mic2 > mic3)
        {
            T2 = mic2;
            T1 = mic3;
        }
        else
        {
            T2 = mic3;
            T1 = mic2;
        }

    }
    if (peak_val == 2)
    {
        if (mic1 > mic3)
        {
            T2 = mic1;
            T1 = mic3;
        }
        else
        {
            T2 = mic3;
            T1 = mic1;
        }

    }

    if (peak_val == 3)
    {
        if (mic2 > mic1)
        {
            T2 = mic2;
            T1 = mic1;
        }
        else
        {
            T2 = mic1;
            T1 = mic2;
        }

    }

    return peak_val;
}
void adcIsr()
{
    static uint8_t index = 0;
    static uint32_t mic1_sum = 0;
    static uint32_t mic2_sum = 0;
    static uint32_t mic3_sum = 0;

    // reading the raw values
    mic1_raw = readAdc0Ss0();
    mic2_raw = readAdc0Ss0();
    mic3_raw = readAdc0Ss0();
    readAdc0Ss0();
    readAdc0Ss0();
    readAdc0Ss0();

    if (tc_count == tc)
    {
        tc_count = 0;

        mic1_sum -= mic1_samples[index];
        mic1_sum += mic1_raw;
        mic1_samples[index] = mic1_raw;
        mic1_avg = mic1_sum/20;

        mic2_sum -= mic2_samples[index];
        mic2_sum += mic2_raw;
        mic2_samples[index] = mic2_raw;
        mic2_avg = mic2_sum/20;

        mic3_sum -= mic3_samples[index];
        mic3_sum += mic3_raw;
        mic3_samples[index] = mic3_raw;
        mic3_avg = mic3_sum/20;

        index = (index + 1) % 20;
    }

    else
    {
        tc_count++;
    }

    if (!H_flag)
    {
        if (peak_found && (track_time <= 23))
        {
            track_time++;
            if (mic1_raw > (mic1_avg + MIC_1_OFFSET - backoff) && mic1_time != 0)
            {
                mic1_time = track_time;
                track_event++;
            }
            track_time++;
            if (mic2_raw > (mic2_avg + MIC_2_OFFSET - backoff) && mic2_time != 0)
            {
                mic2_time = track_time;
                track_event++;
            }
            track_time++;
            if (mic3_raw > (mic3_avg + MIC_3_OFFSET - backoff) && mic3_time != 0)
            {
                mic3_time = track_time;
                track_event++;
            }
            ADC0_ISC_R |= ADC_ISC_IN0;
            return;
        }
        if (((mic1_raw > (mic1_avg + MIC_1_OFFSET)) && !peak_found))
        {
            peak_found = true;
            mic1_origin = true;
//            setPinValue(BLUE_LED,0);
//            setPinValue(GREEN_LED,0);
//            setPinValue(RED_LED,1);
            mic2_origin = false;
            mic3_origin = false;

            track_time = 0;
            mic1_time = track_time;
            track_time++;
            track_event++;

            if (mic2_raw > (mic2_avg + MIC_2_OFFSET - backoff))
            {
                mic2_time = track_time;
                track_event++;
            }

            track_time++;

            if (mic3_raw > (mic3_avg + MIC_3_OFFSET - backoff))
            {
                mic3_time = track_time;
                track_event++;
            }

            ADC0_ISC_R |= ADC_ISC_IN0;
            return;
        }

        if (((mic2_raw > (mic2_avg + MIC_2_OFFSET)) && !peak_found))
        {
            peak_found = true;
            mic2_origin = true;
//            setPinValue(GREEN_LED,0);
//            setPinValue(RED_LED,0);
//            setPinValue(BLUE_LED,1);
            mic1_origin = false;
            mic3_origin = false;

            track_time = 0;
            mic2_time = track_time;
            track_time++;

            if (mic3_raw > (mic3_avg + MIC_3_OFFSET - backoff))
            {
                mic3_time = track_time;
                track_event++;
            }

            ADC0_ISC_R |= ADC_ISC_IN0;
            return;
        }

        if (((mic3_raw > (mic3_avg + MIC_3_OFFSET)) && !peak_found))
        {
            peak_found = true;
            mic3_origin = true;
//            setPinValue(BLUE_LED,0);
//            setPinValue(RED_LED,0);
//            setPinValue(GREEN_LED,1);
            mic1_origin = false;
            mic2_origin = false;

            track_time = 0;
            mic3_time = track_time;
            track_time++;
        }


        if (track_event == 3)
        {
            holdoff = true;
            mic1_time_store = mic1_time;
            mic2_time_store = mic2_time;
            mic3_time_store = mic3_time;
            peak_found = false;
            track_time = 0;
            track_event = 0;
            mic1_time = 0;
            mic2_time = 0;
            mic3_time = 0;
            mic1_origin = false;
            mic2_origin = false;
            mic3_origin = false;

            event_detected = true;
            peak_mic = detectPeak(mic1_time, mic2_time, mic3_time);
//            setPinValue(GREEN_LED,0);
            //setPinValue(RED_LED,1);
//            setPinValue(BLUE_LED,1);
            ADC0_ISC_R |= ADC_ISC_IN0;
        }

        else if (track_time > 40)
        {
            mic1_time_store = mic1_time;
            mic2_time_store = mic2_time;
            mic3_time_store = mic3_time;
            detection_fail = true;
            peak_found = false;
            track_time = 0;
            track_event = 0;
            mic1_time = 0;
            mic2_time = 0;
            mic3_time = 0;
            mic1_origin = false;
            mic2_origin = false;
            mic3_origin = false;

            ADC0_ISC_R |= ADC_ISC_IN0;
        }
    }

    ADC0_ISC_R |= ADC_ISC_IN0;
}


//void detectEvent()
//{
//    if (((mic1_raw > (mic1_avg + MIC_1_OFFSET)) && !peak_found) || mic1_origin)
//    {
//        peak_found = true;
//        mic1_origin = true;
//        mic2_origin = false;
//        mic3_origin = false;
//
//        mic1_time = 0;
//
//        if (mic2_raw > (mic2_avg + MIC_2_OFFSET - 40))
//        {
//            mic2_time = track_time;
//            track_event++;
//        }
//
//        if (mic3_raw > (mic3_avg + MIC_3_OFFSET - 40))
//        {
//            mic3_time = track_time;
//            track_event++;
//        }
//
//        //track_time +=6;
//
//        if (track_event == 2)
//        {
//            event_detected = true;
//            setPinValue(BLUE_LED,0);
//            setPinValue(GREEN_LED,0);
//            setPinValue(RED_LED,1);
//            peak_found = false;
//            track_time = 0;
//            track_event = 0;
//            mic1_time = 0;
//            mic2_time = 0;
//            mic3_time = 0;
//            mic1_origin = false;
//            mic2_origin = false;
//            mic3_origin = false;
//            holdoff = new_holdoff;
//        }
//    }
//
//    if (((mic2_raw > (mic2_avg + MIC_2_OFFSET)) && !peak_found) || mic2_origin)
//    {
//        peak_found = true;
//        mic2_origin = true;
//        mic1_origin = false;
//        mic3_origin = false;
//
//        mic2_time = 0;
//
//        if ((track_time > 0) && (mic1_raw > (mic1_avg + MIC_1_OFFSET - 40)))
//        {
//            mic1_time = track_time;
//            track_event++;
//        }
//
//        if (mic3_raw > (mic3_avg + MIC_3_OFFSET - 40))
//        {
//            mic3_time = track_time;
//            track_event++;
//        }
//
//        //track_time += 6;
//
//        if (track_event == 2)
//        {
//            event_detected = true;
//            setPinValue(BLUE_LED,0);
//            setPinValue(RED_LED,0);
//            setPinValue(GREEN_LED,1);
//            peak_found = false;
//            track_time = 0;
//            track_event = 0;
//            mic1_time = 0;
//            mic2_time = 0;
//            mic3_time = 0;
//            mic1_origin = false;
//            mic2_origin = false;
//            mic3_origin = false;
//            holdoff = new_holdoff;
//        }
//    }
//
//    if (((mic3_raw > (mic3_avg + MIC_3_OFFSET)) && !peak_found) || mic3_origin)
//    {
//        peak_found = true;
//        mic3_origin = true;
//        mic1_origin = false;
//        mic2_origin = false;
//
//        mic3_time = 0;
//
//        if ((track_time > 0) && (mic1_raw > (mic1_avg + MIC_1_OFFSET - 40)))
//        {
//            mic1_time = track_time;
//            track_event++;
//        }
//
//        if ((track_time > 0) && (mic2_raw > (mic2_avg + MIC_2_OFFSET - 40)))
//        {
//            mic2_time = track_time;
//            track_event++;
//        }
//
//        //track_time += 6;
//
//        if (track_event == 2)
//        {
//            event_detected = true;
//            setPinValue(RED_LED,0);
//            setPinValue(GREEN_LED,0);
//            setPinValue(BLUE_LED,1);
//            peak_found = false;
//            track_time = 0;
//            track_event = 0;
//            mic1_time = 0;
//            mic2_time = 0;
//            mic3_time = 0;
//            mic1_origin = false;
//            mic2_origin = false;
//            mic3_origin = false;
//            holdoff = new_holdoff;
//
//        }
//    }
//}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    // Configure LEDS as a push-pull output
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(GREEN_LED);

    // Configure all MICs as analog inputs
    selectPinAnalogInput(MIC1);
    selectPinAnalogInput(MIC2);
    selectPinAnalogInput(MIC3);
}

//-----------------------------------------------------------------------------
// User Interface
//-----------------------------------------------------------------------------

void processShell()
{
    if (kbhitUart0())
    {
        getsUart0(&data);
        parseFields(&data);

        bool valid = false;

        // If “reset” is received, the hardware shall reset.
        if (isCommand(&data, "reset", 0))
        {
            valid = true;
            NVIC_APINT_R = (NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);
            return;
        }

        // If “average” is received, the average value (in DAC units and SPL) of each microphone should be displayed.
        else if (isCommand(&data, "average", 0))
        {
            float mic1_avg_volt = ((float)mic1_avg)/4096 * 3.3;
            float mic2_avg_volt = ((float)mic2_avg)/4096 * 3.3;
            float mic3_avg_volt = ((float)mic3_avg)/4096 * 3.3;

            float mic1_avg_SPL = (20 * log10(mic1_avg_volt / VRMS)) - SENSITIVITY + SPL_REF - GAIN;
            float mic2_avg_SPL = (20 * log10(mic2_avg_volt / VRMS)) - SENSITIVITY + SPL_REF - GAIN;
            float mic3_avg_SPL = (20 * log10(mic3_avg_volt / VRMS)) - SENSITIVITY + SPL_REF - GAIN;

            snprintf(str, sizeof(str), "Mic 1 Average: %.3f DAC unit\n", mic1_avg_volt);
            putsUart0(str);
            snprintf(str, sizeof(str), "               %.3f dB\n", mic1_avg_SPL);
            putsUart0(str);
            snprintf(str, sizeof(str), "Mic 2 Average: %.3f DAC unit\n", mic2_avg_volt);
            putsUart0(str);
            snprintf(str, sizeof(str), "               %.3f dB\n", mic2_avg_SPL);
            putsUart0(str);
            snprintf(str, sizeof(str), "Mic 3 Average: %.3f DAC unit\n", mic3_avg_volt);
            putsUart0(str);
            snprintf(str, sizeof(str), "               %.3f dB\n", mic3_avg_SPL);
            putsUart0(str);

            valid = true;
        }


        // If “tc T” is received, the time constant of the average filter should be set to T. Determine what reasonable values of this time constant should be
        else if (isCommand(&data, "tc", 0))
        {
            tc = getFieldInteger(&data,1);
            snprintf(str, sizeof(str), "Current TC: %7"PRIu32" \n", tc);
            putsUart0(str);
            valid = true;
        }

        // If “backoff B” is received, the backoff between the first and the subsequent microphone signal threshold levels can be set
        else if (isCommand(&data, "backoff", 0))
        {
            backoff = getFieldInteger(&data,1);
            snprintf(str, sizeof(str), "Current B: %7"PRIu32" \n", backoff);
            putsUart0(str);
            valid = true;
        }

        // If “holdoff H” is received, set the minimum time before the next event can be detected. If the argument H is missing, return the current setting.
        else if (isCommand(&data, "holdoff", 0))
        {

            holdoff = getFieldInteger(&data,1);
            H_flag = true;
            snprintf(str, sizeof(str), "Current H: %7"PRIu32" \n", holdoff);
            putsUart0(str);
            valid = true;
        }

        // If “aoa” is received, return the most current angle of arrival (theta and optionally theta and phi).
        else if (isCommand(&data, "aoa", 0))
        {
            aoa_always = false;
            aoa = true;
            uint16_t time_diff = T2-T1;
            theta = (float)(60*k*(time_diff));
            snprintf(str, sizeof(str), "AoA:    %.3f degrees\n", theta);

            putsUart0(str);
            if (tdoa)
            {
                snprintf(str, sizeof(str), "T1: %7"PRIu32" \n", T1);
                putsUart0(str);
                snprintf(str, sizeof(str), "T2: %7"PRIu32" \n", T2);
                putsUart0(str);
            }
            valid = true;
        }

        // If “aoa always” is received, display the AoA information of each “event” as it is detected.
        else if (isCommand(&data, "aoaalways", 0))
        {
            aoa_always = true;
            valid = true;
        }

        // If “tdoa ON|OFF” is received, enable or disable the display of the TDoA data for qualified events (all sensors see the signal within the possible time window) when AoA data is shown.
        else if (isCommand(&data, "tdoa", 1))
        {
            char* str1 = getFieldString(&data, 1);

            if (!strcmp(str1, "ON"))
            {
                tdoa = true;
                valid = true;
            }

            else if (!strcmp(str1, "OFF"))
            {
                tdoa = false;
                valid = true;
            }

            else
            {
                valid = false;
            }
        }

        // If “fail ON|OFF” is received, enable or disable the display of the partial data set of data from the sensors when you do not have a qualified event (the TDoA is greater than possible or is incomplete).
        else if (isCommand(&data, "fail", 1))
        {
            char* str2 = getFieldString(&data, 1);

            if (!strcmp(str2, "ON"))
            {
                fail = true;
                valid = true;
            }



            else if (!strcmp(str2, "OFF"))
            {
                fail = false;
                valid = true;
            }

            else
            {
                valid = false;
            }
        }

        else if (isCommand(&data, "help", 0))
        {
            putsUart0("Commands:\n");
            putsUart0("  reset\n");
            putsUart0("  average\n");
            putsUart0("  tc           T\n");
            putsUart0("  backoff      B\n");
            putsUart0("  holdoff      H\n");
            putsUart0("  hysteresis   Y\n");
            putsUart0("  aoa\n");
            putsUart0("  aoa          always\n");
            putsUart0("  tdoa         ON|OFF\n");
            putsUart0("  fail         ON|OFF\n");
            putsUart0("\n");
        }

        // Look for error
        else if (!valid)
        {
            putsUart0("Invalid command\n");
        }
    }

}




//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    initAdc0Ss0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Set SS0 analog inputs
    setAdc0Ss0Mux(MIC_1_SAMPLE);
    setAdc0Ss0Mux(MIC_2_SAMPLE);
    setAdc0Ss0Mux(MIC_3_SAMPLE);
    setAdc0Ss0Log2AverageCount(2);

    // enable NVIC interrupt
    disableNvicInterrupt(INTERRUPT_MASK);
    enableNvicInterrupt(INTERRUPT_MASK);

    // Greeting
    putsUart0("x-----CSE 4342 Final Project-----x\n");

    // Endless loop
    while(true)
    {
        processShell();

        if (H_flag)
        {
            waitMicrosecond(holdoff);
            putsUart0("Holdoff DONE!\n");
            H_flag = false;
        }

        if (fail && detection_fail)
        {
            snprintf(str, sizeof(str), "T1: %7"PRIu32" \n", mic1_time_store);
            putsUart0(str);
            snprintf(str, sizeof(str), "T2: %7"PRIu32" \n", mic2_time_store);
            putsUart0(str);
            snprintf(str, sizeof(str), "T3: %7"PRIu32" \n", mic3_time_store);
            putsUart0(str);
        }

        if (aoa_always && event_detected)
        {
            theta = (float)(60*k*(T2 - T1));
            snprintf(str, sizeof(str), "AoA:    %.3f degrees\n", theta);
            putsUart0(str);
            if (tdoa)
            {
                snprintf(str, sizeof(str), "T1: %7"PRIu32" \n", T2);
                putsUart0(str);
                snprintf(str, sizeof(str), "T2: %7"PRIu32" \n", T1);
                putsUart0(str);
            }
            event_detected = false;
        }

        if (mic1_origin)
        {
            setPinValue(GREEN_LED,0);
            setPinValue(BLUE_LED,0);
            setPinValue(RED_LED,1);
        }
        else if (mic2_origin)
        {
            setPinValue(GREEN_LED,0);
            setPinValue(RED_LED,0);
            setPinValue(BLUE_LED,1);
        }
        else if (mic3_origin)
        {
            setPinValue(RED_LED,0);
            setPinValue(BLUE_LED,0);
            setPinValue(GREEN_LED,1);
        }
//        snprintf(str, sizeof(str), "T2: %7"PRIu32" \n", mic3_raw);
//        putsUart0(str);

    }
}
