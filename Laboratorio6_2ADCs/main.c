/*
 * adc2_timer_uart.c
 * Programa que muestra el uso de dos canales ADC simultáneamente en la Tiva C.
 * El muestreo se hace dentro de la rutina de interrupción de un timer, para que
 * sea uniforme. Las conversiones se envían por UART, para poder ser leídas en
 * una computadora.
 * También ilustra el uso de floats, y cómo se pueden desplegar en la terminal.
 *
 * Basado en los ejemplos single_ended.c y timers.c de TivaWare
 * Modificado por Luis Alberto Rivera
 */

//*****************************************************************************
//
// single_ended.c - Example demonstrating how to configure the ADC for
//                  single ended operation.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

float v0, v1;   // para ilustrar el uso de variables tipo float

//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>Single Ended ADC (single_ended)</h1>
//!
//! Este ejemplo muestra cómo se puede muestrear en dos canales ADC de forma
//! simultánea. Se usan los puertos AIN0/PE3 y AIN1/PE2.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//! - GPIO Port E peripheral (for AIN0 and AIN1 pin)
//! - AIN0 - PE3
//! - AIN1 - PE2
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! ADC.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - Timer0IntHandler.
//
//*****************************************************************************


//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    // Notar que ahora necesitamos dos espacios para las conversiones.
    uint32_t pui32ADC0Value[2];

    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 2);  // Notar el cambio de "secuencia" (2 en lugar de 3).

    // Wait for conversion to be completed.
    while(!ADCIntStatus(ADC0_BASE, 2, false))  // Notar el cambio de "secuencia".
    {
    }

    // Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Ahora se leen dos valores, no sólo uno, según la configuración.
    // En pui32ADC0Value[0] se tendrá el valor del puerto AIN0 y
    // en pui32ADC0Value[1] se tendrá el valor del puerto AIN1, porque así se
    // configuró en el main.
    ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);  // Notar el cambio de "secuencia".

    v0 = pui32ADC0Value[0]*3.3/4095.0;  // Convertir a voltios
    v1 = pui32ADC0Value[1]*3.3/4095.0;  // Convertir a voltios

    // Se muestra el valor convertido de AIN0 (PE3) y AIN1 (PE2), que son enteros
    // entre 0 y 4095, y los valores en voltios, entre 0.00 y 3.30.
    // Nota: UARTprintf no soporta floats (%f). Acá se hace un truco para mostrar
    // la parte entera y dos posiciones decimales, como enteros.
    // Notar que el "casting" trunca, no redondea.
    // Si no es indispensable desplegar floats, mejor evitar hacerlo.
    UARTprintf("A0: %04d, v0: %d.%02d,   A1: %04d, v1: %d.%02d\n",
               pui32ADC0Value[0], (uint8_t)v0, (uint8_t)(100*(v0-(uint8_t)v0)),
               pui32ADC0Value[1], (uint8_t)v1, (uint8_t)(100*(v1-(uint8_t)v1)));
}


//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Configure ADC0 for a single-ended input and a single sample.  Once the
// sample is ready, an interrupt flag will be set.  Using a polling method,
// the data will be read then displayed on the console via UART0.
//
//*****************************************************************************
int
main(void)
{
    uint16_t freq_muestreo = 250;    // En Hz

    // Set the clocking to run at 80 MHz (200 MHz / 2.5) using the PLL.  When
    // using the ADC, you must either use the PLL or supply a 16 MHz clock source.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    //                 SYSCTL_XTAL_16MHZ); // 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 80 MHz

    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation.
    InitConsole();

    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // For this example ADC0 is used with AIN0 and AIN1.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.  GPIO port E needs to be enabled
    // so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);  // Configura el pin PE3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);  // Configura el pin PE2

    // Se configura la secuencia 2, que permitiría hasta cuatro muestras (aunque
    // se usarán dos).
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

    // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

    // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
    // y configura la bandera de interrupción (ADC_CTL_IE) para "setearse"
    // cuando se tenga esta muestra. También se indica que esta es la última
    // conversión en la secuencia 2 (ADC_CTL_END).
    // Para más detalles del módulo ADC, consultar el datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

    // Since sample sequence 2 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Enable processor interrupts.
    IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // El tercer argumento determina la frecuencia. El reloj se puede obtener
    // con SysCtlClockGet().
    // La frecuencia está dada por SysCtlClockGet()/(valor del 3er argumento).
    // Ejemplos: Si se pone SysCtlClockGet(), la frecuencia será de 1 Hz.
    //           Si se pone SysCtlClockGet()/1000, la frecuencia será de 1 kHz
    //TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_muestreo));

    // Setup the interrupt for the timer timeout.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Las conversiones se hacen al darse la interrupción del timer, para que
    // el muestreo sea preciso. Luego de las configuraciones, el programa se
    // queda en un ciclo infinito haciendo nada.
    while(1)
    {
    }
}
