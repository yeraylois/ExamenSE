/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "MKL46Z4.h"
#include "components_functions.h"
#include "lcd.h"


#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
// MODOS DE FRECUENCIAS
#define FREQ_0HZ   0
#define FREQ_05HZ  1
#define FREQ_1HZ   2
#define FREQ_2HZ   3

// Frecuencia de Inicio
static volatile int freqMode = FREQ_1HZ; // Arranca en 1 Hz

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_enviroment(void);
void leds_off(void);
void led_green_toggle(void);
void LPTMR0_IRQHandler(void);
/*******************************************************************************
 * Code
 ******************************************************************************/


static void lptmr_set_compare(uint16_t compareVal)
{
    //Desactivar LPTMR para poder cambiar configuración
    LPTMR0->CSR = 0;

    // Poñer o valor deseado
    LPTMR0->CMR = compareVal;

    //Limpar posible flag previo
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

    //Activar interrupción e arrincar o timer
    LPTMR0->CSR |= LPTMR_CSR_TIE_MASK | LPTMR_CSR_TEN_MASK;
}

/*
   Función que actualiza o LPTMR segundo o modo freqMode:
   0 Hz -> desactiva o timer e apaga LED
   0.5 Hz -> 2000
   1 Hz   -> 1000
   2 Hz   ->  500
*/
void actualizar_lptmr_freq(int modo)
{
    switch(modo)
    {
        case FREQ_0HZ:
            // Desactivar timer => LED apagado
            LPTMR0->CSR = 0;
            leds_off();
            break;
        case FREQ_05HZ:
            lptmr_set_compare(2000);  // 2 s
            break;
        case FREQ_1HZ:
            lptmr_set_compare(1000);  // 1 s
            break;
        case FREQ_2HZ:
            lptmr_set_compare(500);   // 0.5 s
            break;
        default:
            break;
    }
}

/*
   Mostrar en consola (ou LCD) a frecuencia
*/
void mostrar_frecuencia(int f)
{
    switch(f)
    {
        case FREQ_0HZ:
            PRINTF("Frecuencia: 0 Hz\r\n");
            lcd_display_dec(0);
            break;
        case FREQ_05HZ:
            PRINTF("Frecuencia: 0.5 Hz\r\n");
            lcd_display_dec(5);
            break;
        case FREQ_1HZ:
            PRINTF("Frecuencia: 1 Hz\r\n");
            lcd_display_dec(10);
            break;
        case FREQ_2HZ:
            PRINTF("Frecuencia: 2 Hz\r\n");
            lcd_display_dec(20);
            break;
        default:
            break;
    }
}


void PORTC_PORTD_IRQHandler(void)
{
    // Se xera interrupción no pin 3
    if (PORTC->ISFR & (1 << 3))
    {
        if (freqMode >= FREQ_0HZ)
        {
            freqMode++;
            mostrar_frecuencia(freqMode);
            actualizar_lptmr_freq(freqMode);
        }
        PORTC->ISFR |= (1 << 3);
    }

    // Se xera interrupción no pin 12
    if (PORTC->ISFR & (1 << 12))
    {
        if (freqMode <= FREQ_2HZ)
        {
            freqMode--;
            mostrar_frecuencia(freqMode);
            actualizar_lptmr_freq(freqMode);
        }
        PORTC->ISFR |= (1 << 12);
    }
}

//Interrupcion del LPTMR
void LPTMR0_IRQHandler(void)
{
    // Limpar o flag TCF
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

    if (freqMode != FREQ_0HZ)
    {
        led_green_toggle();
    }
}

/*
   Inicializa o LPTMR con LPO ~1 kHz como fonte.
*/
void lptmr_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;

    // Desactivar timer
    LPTMR0->CSR = 0;

    LPTMR0->PSR = LPTMR_PSR_PBYP_MASK
                | LPTMR_PSR_PCS(1);

    // Habilitar interrupción en NVIC
    NVIC_EnableIRQ(LPTMR0_IRQn);
}

/*!
 * @brief Main function
 */

int main(void)
{
    char ch;
    
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("Plantilla exame Sistemas Embebidos: 1a oportunidade 24/25 Q1\r\n");

    init_enviroment();
    leds_off();
    
    irclk_ini();
    lcd_ini();

    PORTC->PCR[3]  |= PORT_PCR_IRQC(0xA);
    PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);

    lptmr_init();

    mostrar_frecuencia(freqMode);
    actualizar_lptmr_freq(freqMode);

    while (1)
    {
        ch = GETCHAR();  // Lemos un carácter por UART
        PUTCHAR(ch);     // Ecoamos o carácter lido
    }
}
