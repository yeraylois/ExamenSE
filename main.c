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

#include "pin_mux.h"

#include "MKL46Z4.h"
#include "components_functions.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static volatile int door1 = 1; // 1 = aberta, 0 = pechada
static volatile int door2 = 1; // 1 = aberta, 0 = pechada

void update_leds() {

    if (door1 == 0 && door2 == 0) {
        leds_off();
        led_red_toggle();   // Encende LED vermello
    } else {
        leds_off();
        led_green_toggle(); // Encende LED verde
    }
}

void PORTC_PORTD_IRQHandler(void) {
    // SW1
    if (PORTC->ISFR & (1 << 3)) {
        door1 = !door1;       // Toggle porta1
        update_leds();
        PORTC->ISFR |= (1 << 3);
    }

    // SW2
    if (PORTC->ISFR & (1 << 12)) {
        door2 = !door2;       // Toggle da porta 2
        update_leds();
        PORTC->ISFR |= (1 << 12);
    }
}

// Configura interrupcións nos pines dos botóns
void init_interrupts() {
    // As funcións sw1_init() e sw2_init() xa configuran os pines como entrada con pull-up
    // atópanse en (drivers/components_functions.h)

    PORTC->PCR[3]  |= PORT_PCR_IRQC(0xA);
    PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);

    // Habilita interrupcions
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
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

    update_leds();

    // Configura interrupcións
    init_interrupts();

    // Bucle principal
    while (1)
    {
        ch = GETCHAR();  // Lemos un carácter por UART
        PUTCHAR(ch);     // Ecoamos o carácter lido
    }
}
