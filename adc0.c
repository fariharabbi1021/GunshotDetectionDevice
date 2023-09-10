// ADC0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// ADC0 SS0

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc0.h"

#define ADC_CTL_DITHER          0x00000040

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc0Ss0()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Configure ADC
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;                // disable sample sequencer 0 (SS0) for programming
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC0_EMUX_R = ADC_EMUX_EM0_ALWAYS;               // select SS0 bit in ADCPSSI as trigger
    ADC0_SSCTL0_R = ADC_SSCTL0_END5;                 // mark first sample as the end
    ADC0_SSCTL0_R = ADC_SSCTL0_IE5;                  //
    ADC0_IM_R |= ADC_IM_MASK0;
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;                 // enable SS0 for operation
}

// Set SS0 input sample average count
void setAdc0Ss0Log2AverageCount(uint8_t log2AverageCount)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;                // disable sample sequencer 0 (SS0) for programming
    ADC0_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC0_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC0_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;                 // enable SS0 for operation
}

// Set SS0 analog input
void setAdc0Ss0Mux(uint8_t mic, uint32_t input)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;                  // disable sample sequencer 0 (SS0) for programming
    if (mic == 0)
    {
        ADC0_SSMUX0_R |= input | (input << 12);        // Set analog input for single sample
    }
    else if (mic == 1)
    {
        ADC0_SSMUX0_R |= (input << 4) | (input << 16); // Set analog input for single sample
    }
    else if (mic == 2)
    {
        ADC0_SSMUX0_R |= (input << 8) | (input << 20); // Set analog input for single sample
    }
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;                   // enable SS3 for operation
}

// Request and read one sample from SS0
int16_t readAdc0Ss0()
{
    ADC0_PSSI_R |= ADC_PSSI_SS0;                     // set start bit
//    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS0 is not busy
//    while (ADC0_SSFSTAT0_R & ADC_SSFSTAT0_EMPTY);
    return ADC0_SSFIFO0_R;                           // get single result from the FIFO
}
