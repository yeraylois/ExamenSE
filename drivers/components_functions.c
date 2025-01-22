#include "MKL46Z4.h"
#include "components_functions.h"

// LED_GREEN = PTD5
void led_green_init() {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // HABILITAR EL RELOJ PARA EL PUERTO D
    PORTD->PCR[5] = PORT_PCR_MUX(1);    // CONFIGURAR PTD5 COMO GPIO
    GPIOD->PDDR |= (1 << 5);            // CONFIGURAR PTD5 COMO SALIDA
    GPIOD->PSOR |= (1 << 5);            // APAGAR INICIALMENTE
}

void led_green_toggle() {
    GPIOD->PTOR |= (1 << 5);            // CONMUTAR EL LED VERDE
}

void led_green_off() {
    GPIOD->PSOR |= (1 << 5);            // APAGAR LED VERDE
}

// LED_RED = PTE29
void led_red_init() {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; // HABILITAR EL RELOJ PARA EL PUERTO E
    PORTE->PCR[29] = PORT_PCR_MUX(1);   // CONFIGURAR PTE29 COMO GPIO
    GPIOE->PDDR |= (1 << 29);           // CONFIGURAR PTE29 COMO SALIDA
    GPIOE->PSOR |= (1 << 29);           // APAGAR INICIALMENTE
}

void led_red_toggle() {
    GPIOE->PTOR |= (1 << 29);           // CONMUTAR EL LED ROJO
}

void led_red_off() {
    GPIOE->PSOR |= (1 << 29);           // APAGAR LED ROJO
}

// SW1 = PTC3
void sw1_init() {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; // HABILITAR RELOJ PARA PORTC
    PORTC->PCR[3] &= ~PORT_PCR_MUX_MASK; // LIMPIAR 3 BITS MUX
    PORTC->PCR[3] |= PORT_PCR_MUX(1);    // CONFIGURAR COMO GPIO
    PORTC->PCR[3] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // HABILITAR PULL-UP
    GPIOC->PDDR &= ~(1 << 3);            // CONFIGURAR PTC3 COMO ENTRADA
}

// SW2 = PTA20 (ANTERIORMENTE RESET_B)
void sw2_init() {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; // HABILITAR RELOJ PARA PORTA
    PORTA->PCR[20] &= ~PORT_PCR_MUX_MASK; // LIMPIAR LOS BITS DEL MUX
    PORTA->PCR[20] |= PORT_PCR_MUX(1);    // CONFIGURAR COMO GPIO
    PORTA->PCR[20] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // HABILITAR PULL-UP
    GPIOA->PDDR &= ~(1 << 20);            // CONFIGURAR PTA20 COMO ENTRADA
}

// SW3 = PTC12
void sw3_init() {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; // HABILITAR RELOJ PARA PORTC
    PORTC->PCR[12] &= ~PORT_PCR_MUX_MASK; // LIMPIAR 3 BITS MUX
    PORTC->PCR[12] |= PORT_PCR_MUX(1);    // CONFIGURAR COMO GPIO
    PORTC->PCR[12] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // HABILITAR PULL-UP
    GPIOC->PDDR &= ~(1 << 12);            // CONFIGURAR PTC12 COMO ENTRADA
}

// VERIFICA SI EL BOTÓN SW1 ESTÁ PRESIONADO
int is_pressed_sw1() {
    return !(GPIOC->PDIR & (1 << 3));   // RETORNA 1 SI EL BOTÓN ESTÁ PRESIONADO (NIVEL BAJO)
}

// VERIFICA SI EL BOTÓN SW2 (PTA20) ESTÁ PRESIONADO
int is_pressed_sw2() {
    return !(GPIOA->PDIR & (1 << 20)); // RETORNA 1 SI EL BOTÓN ESTÁ PRESIONADO (NIVEL BAJO)
}

// VERIFICA SI EL BOTÓN SW3 (PTC12) ESTÁ PRESIONADO
int is_pressed_sw3() {
    return !(GPIOC->PDIR & (1 << 12)); // RETORNA 1 SI EL BOTÓN ESTÁ PRESIONADO (NIVEL BAJO)
}

// INICIALIZA LEDS Y BOTONES
void init_enviroment() {
    led_green_init();
    led_red_init();
    sw1_init();
    sw2_init();
    sw3_init();

    leds_off(); // ASEGURARSE DE QUE LOS LEDS ESTÉN APAGADOS
}

// APAGA TODOS LOS LEDS
void leds_off() {
    GPIOD->PSOR |= (1 << 5);  // APAGAR EL LED VERDE
    GPIOE->PSOR |= (1 << 29); // APAGAR EL LED ROJO
}
