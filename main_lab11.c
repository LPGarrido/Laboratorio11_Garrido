/*
 * File:   SPI_S10.c
 * Author: Christopher Chiroy
 *
 * Comunicación SPI, con contador en maestro que incrementa cada segundo y envia
 * dato al escalvo y el esclavo lo muestra en el PORTD.
 *
 * Created on 3 mei 2022, 19:00
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define B1 PORTBbits.RB6
#define B2 PORTBbits.RB7
#define FLAG_SPI 0x0F
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 5               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 30             // Valor máximo de ancho de pulso de señal PWM

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t val_PWM = 0;           // Contador que envía el maestro al esclavo
uint8_t val_POT = 0;
uint8_t old_val_POT = 0;
uint8_t conteo = 0;
char val_temporal = 0;
unsigned short CCPR = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PORTBbits.RB5){                  // ¿Es maestro?
        // interrupciones del maestro
        if(PIR1bits.ADIF){              // Fue interrupci?n del ADC?
            val_POT = ADRESH;
            //PORTD = val_POT;
            PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupci?n
        }
    }
    else {                              // ¿Es esclavo?
        // interrupciones del esclavo
        if(PIR1bits.SSPIF){             // ¿Recibió datos el esclavo?
            val_temporal = SSPBUF;
            if (val_temporal != FLAG_SPI){  // Es envío solo para generar los pulsos de reloj?
                PORTD = val_temporal;       // Mostramos valor recibido en PORTD
                SSPBUF = conteo;        // Cargamos contador del esclavo al buffer
            }
            else {
                SSPBUF = conteo;
            }
            PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
        }
        else if(INTCONbits.RBIF){            // Fue interrupción del PORTB
            if(!B1)                     // Verificamos si fue RB0 quien generó la interrupción
                conteo++;                // Incrementar PORTA
            else if (!B2)               // Si es RB2
                conteo--;                // Decrementar PORTA
            INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        // El RA0 se configuró como entrada y si está encendida, quiere decir
        //  que el pic debe funcionar en modo maestro
        if(PORTBbits.RB5){          // ¿Es maestro?
            if(ADCON0bits.GO == 0){             // No hay proceso de conversion
                ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
            }
            
            // ENVIAR POT
            
            PORTAbits.RA6 = 1;      // Deshabilitamos el ss del esclavo
            __delay_ms(10);         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
            PORTAbits.RA6 = 0;      // habilitamos nuevamente el escalvo
            __delay_ms(10);
            
            if(val_POT != old_val_POT){
                SSPBUF = val_POT;   // Cargamos valor del contador al buffer
                while(!SSPSTATbits.BF){}// Esperamos a que termine el envio
                old_val_POT = val_POT;
            }
            PORTAbits.RA6 = 1;
            __delay_ms(10);
            //---------------------
            
            // PREPAR PARA RECIBIR CONTEO
            
            PORTAbits.RA7 = 1;      // Deshabilitamos el ss del esclavo
            __delay_ms(10);         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
            PORTAbits.RA7 = 0;      // habilitamos nuevamente el escalvo
            __delay_ms(10);

            SSPBUF = FLAG_SPI;      // Se envía cualquier cosa, Esto es para que el maestro
                                    //  genere los pulsos del reloj que necesita el esclavo
                                    //  para transmitir datos.
            while(!SSPSTATbits.BF){}// Esperamos a que se reciba un dato
            PORTD = SSPBUF;         // Mostramos dato recibido en PORTD
            
            PORTAbits.RA7 = 1;
            
        }
        else{
            CCPR = map(PORTD, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x01;               // AN0 como entrada analogica
    ANSELH = 0;                 // I/O digitales

    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno

    TRISA = 0b00100001;         // SS y RA0 como entradas
    PORTA = 0;

    TRISB = 0xFF;               // Puerto B como entrada

    TRISD = 0;
    PORTD = 0;

    // Configuración de SPI
    // Configs de Maestro
    if(PORTBbits.RB5){

        // Configuraciones del SPI
        TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
        PORTC = 0;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
        SSPBUF = val_POT;              // Enviamos un dato inicial


        // Configuraci?n ADC
        ADCON0bits.ADCS = 0b00;     // Fosc/2
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);             // Sample time

        // Configuracion interrupciones
        PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
        INTCONbits.GIE = 1;         // Habilitamos int. globales
    }
    // Configs del esclavo
    else{
        TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
        PORTC = 0;

        OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
        WPUB = 0xC0;                // Habilitamos resistencia de pull-up de RB0

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
        
        // Configuración PWM
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        PR2 = 62;                  // periodo de 4ms

        // Configuración CCP
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM

        CCPR1L = 5>>2;
        CCP1CONbits.DC1B = 5 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo

        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Encendemos TMR2
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM

        // Interrupciones
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
        PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
        IOCB = 0xC0;                // Habilitamos interrupción por cambio de estado para RB0 y RB1
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
    }
}

/*------------------------------------------------------------------------------
 * FUNCIONES 
 ------------------------------------------------------------------------------*/

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}