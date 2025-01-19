/*
 * BCD_Trial.c
 *
 *  Created on: Jan 19, 2025
 *      Author: Mihai
 */

#include <msp430.h>

// Definirea segmentelor pentru cifrele 0-9 (în format binar pentru 7 segmente)
// Segmentele sunt ordonate ca: a b c d e f g
const unsigned char digit_segments[10] = {
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110  // 9
};

// Variabile globale pentru afișare
unsigned int counter = 1; // Numărătoare de la 1 la 9999
unsigned char digits[4];  // Cifrele de afișat

void initGPIO(void) {
    // Configurare pini pentru segmentele 7-segmente (P1.0 - P1.5 și P3.6, P3.7)
    P1DIR |= 0x3F; // Setăm pinii P1.0 - P1.5 ca ieșire
    P1OUT &= ~0x3F; // Oprire segmente a-f

    P3DIR |= 0xC0; // Setăm pinii P3.6 și P3.7 ca ieșire
    P3OUT &= ~0xC0; // Oprire segmente g și dp

    // Configurare pini pentru selecția cifrelor (P2.0 - P2.2, P2.4)
    P2DIR |= 0x17; // Setăm P2.0, P2.1, P2.2, P2.4 ca ieșire
    P2OUT &= ~0x17; // Dezactivăm toate cifrele
}

void delay_ms(unsigned int ms) {
    while (ms--) {
        __delay_cycles(1000); // Delay aproximativ 1 ms (la 1 MHz)
    }
}

void updateDigits(unsigned int num) {
    // Descompunem numărul în cifre BCD
    for (int i = 0; i < 4; i++) {
        digits[i] = num % 10;
        num /= 10;
    }
}

void displayNumber(void) {
    for (int i = 0; i < 4; i++) {
        P2OUT &= ~0x17; // Dezactivăm toate cifrele
        P1OUT = digit_segments[digits[i]] & 0x3F; // Activăm segmentele a-f
        P3OUT = (P3OUT & ~0xC0) | ((digit_segments[digits[i]] & 0xC0)); // Activăm segmentele g și dp

        if (i == 3) {
            P2OUT |= 0x10; // Activăm cifra 4 (P2.4)
        } else {
            P2OUT |= (1 << i); // Activăm cifra curentă
        }

        delay_ms(2); // Timp de afișare pentru multiplexare
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Dezactivăm Watchdog Timer-ul

    initGPIO(); // Inițializăm GPIO
    updateDigits(counter); // Setăm valoarea inițială

    while (1) {
        displayNumber(); // Afișăm numărul curent

        // Incrementăm numărul o dată pe secundă
        if (counter < 9999) {
            counter++;
        } else {
            counter = 1; // Resetăm la 1
        }
        updateDigits(counter); // Actualizăm cifrele
        delay_ms(1000); // Pauză de 1 secundă
    }
}



