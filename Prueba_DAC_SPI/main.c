/*
 * diente_de_sierra_spi.c
 * Programa que muestra el uso del SPI en la Tiva C. Se usa un Timer para incre-
 * mentar un contador, que se envía por SPI a un DAC (MCP4921). El resultado
 * es una señal de diente de sierra.
 *
 * Basado en los ejemplos spi_master.c y timers.c de TivaWare
 * Modificado por Luis Alberto Rivera
 */


//*****************************************************************************
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

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup ssi_examples_list
//! <h1>SPI Master (spi_master)</h1> --- MODIFICADO
//!
//! This example shows how to configure the SSI0 as SPI Master.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - Timer0IntHandler.
//
//*****************************************************************************
//                              Variables
//*****************************************************************************
float y1_n_1 = 0;
float y2_n_1 = 0;
float x2_n_1 = 0;
float y1 = 0;
float y2 = 0;
int caso = 0;
//*****************************************************************************
// Definiciones para configuración del SPI y variable global
//*****************************************************************************
#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define VALOR_MAX    4095  // Valor máximo del contador
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16

uint16_t cont = 0;

//*****************************************************************************
// The interrupt handler for the timer interrupt.
//*****************************************************************************
void
Timer0IntHandler(void)
{
    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;

    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

//*********************************ADC***************************************
    // This array is used for storing the data read from the ADC FIFO. It
        // must be as large as the FIFO for the sequencer in use.  This example
        // uses sequence 3 which has a FIFO depth of 1.  If another sequence
        // was used with a deeper FIFO, then the array size must be changed.
        uint32_t pui32ADC0Value[1];

        // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Wait for conversion to be completed.
        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }

        // Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 3);

        // Read ADC Value.
        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

        // implementacion para H1
        y1 = 0.1*pui32ADC0Value[0] + 0.9*y1_n_1;
        y1_n_1 = y1;
        //************************************************************************
        // implementacion para H2
        y2 = 0.9*(pui32ADC0Value[0]-x2_n_1) + 0.9*y2_n_1;
        y2_n_1 = y2;
        x2_n_1 = pui32ADC0Value[0];
        //************************************************************************
        switch(caso){ //switch para intercambiar entre H1 y H2
            case 0: //Formato para que se vean ambas señales en el serial plotter
                UARTprintf("%d,%d\n", pui32ADC0Value[0],(int)y1);
                break;
            case 1:
                UARTprintf("%d,%d\n", pui32ADC0Value[0],(int)y2);
                break;
        }
        // Display the AIN0 (PE3) digital value on the console.
//*****************************DAC*********************************************
    // Se pudo crear una fución para el envío al DAC (ej. DAC_write)

    // Se pudo usar una bandera: se activa acá en el handler, y se verifica en el
    // ciclo del main. Si la bandera está en alto, se manda al DAC. Al terminar,
    // se apagaría la bandera. Así, el handler únicamente reiniciaría el tempo-
    // rizador y encendería la bandera.
    // Es importante que lo que se haga en el handler no tome más tiempo que el
    // período del temporizador.


        // El DAC espera 16 bits: 4 de configuraciones y 12 del dato
        pui32DataTx[0] = (uint32_t)((0b0111 << 12) | (0x0FFF & (int)y1));

        // Send data
        for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
        {
            // Send the data using the "blocking" put function.  This function
            // will wait until there is room in the send FIFO before returning.
            // This allows you to assure that all the data you send makes it into
            // the send FIFO.
            SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
        }

        // Wait until SSI0 is done transferring all the data in the transmit FIFO.
        while(SSIBusy(SSI0_BASE))
        {
        }

        // Resetear el contador después de VALOR_MAX
        cont = (cont + 1) % (VALOR_MAX + 1);
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
    UARTStdioConfig(0, 230400, 16000000);
}

//*****************************************************************************
// Configure SSI0 in master Freescale (SPI) mode.
// Se configura el timer y su interrupción. El SPI se configura para enviar 16
// bits. El envío se hace en la rutina de interrupción del timer.
//*****************************************************************************
int
main(void)
{
    uint32_t pui32residual[NUM_SPI_DATA];
    uint16_t freq_timer = 1000;    // En Hz

// ------ Configuración del reloj ---------------------------------------------
    // Set the clock to run at 80 MHz (200 MHz / 2.5) using the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 80 MHz. SÍ FUNCIONA
// ----------------------------------------------------------------------------

    // Set up the serial console to use for displaying messages.  This is
        // just for this example program and is not needed for ADC operation.
        InitConsole();
    // The ADC0 peripheral must be enabled for use.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//habilita el puerto F
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3);//Pines de salida para los leds


    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END). Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 3);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 4MHz SSI frequency, and 16-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);

    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);

    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32residual[0]))
    {
    }

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
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_timer));

    // Setup the interrupt for the timer timeout.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);

    // El envío se hace al darse la interrupción del timer, para que sea en
    // intervalos regulares. Luego de las configuraciones, el programa se
    // queda en un ciclo infinito haciendo nada, esperando la interrupción.
    while(1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        SysCtlDelay(50 * (SysCtlClockGet() / 3 / 1000));
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
        SysCtlDelay(50 * (SysCtlClockGet() / 3 / 1000));
    }
}
