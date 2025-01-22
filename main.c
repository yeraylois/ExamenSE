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

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern unsigned int reverse_int1(unsigned int x);
extern unsigned int reverse_int2(unsigned int x);
//extern unsigned int reverse_int3(unsigned int x);
extern unsigned int reverse_int4(unsigned int x);


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void SysTick_init(void)
{
    // Poñemos o contador no máximo (24 bits => 0xFFFFFF)
    SysTick->LOAD = 0x00FFFFFF;
    // Escribimos VAL para limpar
    SysTick->VAL = 0;
    // CLKSOURCE=CPU clock, ENABLE=1, sen interrupción
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t measure_cycles_uint32(uint32_t (*func)(uint32_t), uint32_t val)
{
    // Ler antes
    uint32_t start = SysTick->VAL;
    // Chamar á función
    uint32_t result = func(val);
    // Ler despois
    uint32_t end = SysTick->VAL;

    uint32_t cycles = (start - end);

    return cycles;
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
    
    SysTick_init();

    uint32_t val = 0x5CA1AB1E;

    //reverse1.c
    uint32_t c1 = measure_cycles_uint32(reverse_int1, val);

    uint32_t r1 = reverse_int1(val);
    PRINTF("Elapsed ticks with reverse_int1(): %u (%u)\r\n", c1, r1);
    
    //reverse4.c
    uint32_t c4 = measure_cycles_uint32(reverse_int4, val);
    uint32_t r4 = reverse_int4(val);
    PRINTF("Elapsed ticks with reverse_int4(): %u (%u)\r\n", c4, r4);

    //reverse2.s
    uint32_t c2 = measure_cycles_uint32(reverse_int2, val);
    uint32_t r2 = reverse_int2(val);
    PRINTF("Elapsed ticks with reverse_int2(): %u (%u)\r\n", c2, r2);

   /* //reverse3.s
    uint32_t c3 = measure_cycles_uint32(reverse_int3, val);
    uint32_t r3 = reverse_int3(val);
    PRINTF("Elapsed ticks with reverse_int3(): %u (%u)\r\n", c3, r3);
*/

    while (1)
    {
        __NOP();
    }

}
