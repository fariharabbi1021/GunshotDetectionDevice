// Name: Fariha Rabbi
// ID: 1001749931
// Course: Embedded Systems I
// Lab 5


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);                  // wait if uart0 tx fifo full
    UART0_DR_R = c;                                     // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);                  // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                           // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char c;

    while (true)
    {
        c = getcUart0();

        // decrements counter if character received is backspace or delete
        if (((c == 8) || (c == 127)) && (count > 0))
        {
            count--;
        }

        // adds null terminator and returns if character received is a line feed
        else if (c == 13)
        {
            data->buffer[count] = 0;
            return;
        }

        // adds received character to buffer (if printable) and increments counter
        else if (c >= 32)
        {
            data->buffer[count] = c;
            count++;
            // if count equals max_chars, adds a null terminator and returns from function
            if (count == MAX_CHARS)
            {
                data->buffer[count] = 0;
                return;
            }
            // otherwise loops back until max_chars is reached
        }
    }
}

// LAB 5

void parseFields(USER_DATA *data)
{
    uint8_t bufferCount = 0;
    data->fieldCount = 0;

    while (bufferCount < MAX_CHARS)
    {
        // case for when new character is an alpha
        if (((data->buffer[bufferCount] >= 65) && (data->buffer[bufferCount] <= 90)) || ((data->buffer[bufferCount] >= 97) && (data->buffer[bufferCount] <= 122)))
        {
            // check if previous character has different field type
            if ((bufferCount == 0) || data->buffer[bufferCount-1] < 65 || data->buffer[bufferCount-1] >122 || ((data->buffer[bufferCount] >= 91) && (data->buffer[bufferCount] <= 96)))
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = bufferCount;
                data->fieldCount++;
            }
        }

        // case for when new character is a numeric
        else if ((data->buffer[bufferCount] >= 48) && (data->buffer[bufferCount] <= 57))
        {
            // check if previous character has a different field type
            if ((bufferCount == 0) || (data->buffer[bufferCount-1] < 48) || (data->buffer[bufferCount-1] > 57))
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = bufferCount;
                data->fieldCount++;
            }
        }

        // case for when new character is a delimeter
        else
        {
            data->buffer[bufferCount] = 0;
        }

        // increment buffer counter to keep moving through the buffer string
        bufferCount++;

        // return from function if field count equals max_fields
        if (data->fieldCount == MAX_FIELDS)
        {
            return;
        }
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    // if field number is in range, return the value of the field requested
    if ((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'a'))
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }

    //otherwise return null
    else
        return '\0';
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    // if field number is in range and field type is numeric, return the integer value of the field
    if ((fieldNumber <= data->fieldCount) && (data->fieldType[fieldNumber] == 'n'))
    {
        uint8_t bufferIndex = data->fieldPosition[fieldNumber];
        uint8_t asciiValue = 0;
        uint32_t number = 0;

        // convert string to integer
        while ((data->buffer[bufferIndex]) != 0)
        {
            asciiValue = (int)(data->buffer[bufferIndex] - '0');
            number = asciiValue + (10*number);
            bufferIndex++;
        }

        return number;
    }

    // otherwise return 0
    else
        return 0;

}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    bool equal = true;
    uint8_t bufferIndex = data->fieldPosition[0];
    uint8_t i = 0;
    uint8_t size = 0;

    // finds the length of strCommand
    while (strCommand[size] != 0)
    {
        size++;
    }

    // checks if strings are the same
    while (i < size)
    {
        if (strCommand[i] != data->buffer[bufferIndex])
        {
            equal = false;
        }

        bufferIndex++;
        i++;
    }

    /* return true if command matches the first field and number of arguments (excluding the command field)
       is greater than or equal to the requested number of minimum arguments */
    if (((data->fieldCount-1) >= minArguments) && equal)
    {
        return true;
    }

    // otherwise return false
    else
        return false;
}


