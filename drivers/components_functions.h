// COMPONENTS_FUNCTIONS.H
#ifndef COMPONENTS_FUNCTIONS_H
#define COMPONENTS_FUNCTIONS_H

// INICIALIZA EL LED VERDE (PTD5)
void led_green_init(void);

// CONMUTA EL ESTADO DEL LED VERDE
void led_green_toggle(void);

// APAGA EL LED VERDE
void led_green_off(void);

// INICIALIZA EL LED ROJO (PTE29)
void led_red_init(void);

// CONMUTA EL ESTADO DEL LED ROJO
void led_red_toggle(void);

// APAGA EL LED ROJO
void led_red_off(void);

// INICIALIZA EL BOTÓN SW1 (PTC3)
void sw1_init(void);

// INICIALIZA EL BOTÓN SW2 (PTA20, ANTES RESET_B)
void sw2_init(void);

// INICIALIZA EL BOTÓN SW3 (PTC12)
void sw3_init(void);

// VERIFICA SI EL BOTÓN SW1 (PTC3) ESTÁ PRESIONADO
int is_pressed_sw1(void);

// VERIFICA SI EL BOTÓN SW2 (PTA20) ESTÁ PRESIONADO
int is_pressed_sw2(void);

// VERIFICA SI EL BOTÓN SW3 (PTC12) ESTÁ PRESIONADO
int is_pressed_sw3(void);

// INICIALIZA EL ENTORNO (LEDS Y BOTONES)
void init_enviroment(void);

// APAGA AMBOS LEDS
void leds_off(void);

#endif // COMPONENTS_FUNCTIONS_H
