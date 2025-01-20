/*
//
//                MSP430FR2355
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//            |                 |
//            |     P4.3/UCA1TXD|----> PC (echo)
//            |     P4.2/UCA1RXD|<---- PC
//            |                 |
 *
 * //         |     P1.7/UCA0TXD|----> LIDAR
//            |     P1.6/UCA0RXD|<---- LIDAR
//            |                 |
//            |     P4.1/       |<---- Buton get_info, start
//            |     P2.3/       |<---- Buton force_start
//
 * */

// Baud Rate A1 PC    @ 115200bps
// Baud Rate A0 LIDAR @ 115200bps

#include <msp430.h>

volatile unsigned char get_health_status[2]={0xA5,0x52}; // get health request
volatile unsigned char stop_scan[2]={0xA5,0x25}; // stop scan request
volatile unsigned char reset_scan[2]={0xA5,0x40}; // reset request
volatile unsigned char start_scan[2]={0xA5,0x20}; // start scan request
volatile unsigned char start_scan_resp[7]={0xA5,0x5A,0x05,0x00,0x00,0x40,0x81}; // start scan respons
volatile unsigned char force_scan[2]={0xA5,0x21}; // force scan request
volatile unsigned char express_scan[2]={0xA5,0x82}; // express scan request

volatile unsigned char dataBuffer[5] = {0x00};
volatile unsigned int indexDataBuffer = 0;
volatile unsigned int startScan = 0;
volatile unsigned int comparatorIndexDataLocal = 6;

volatile unsigned char get_info[2]={0xA5,0x50}; // get info request
volatile unsigned char get_sample_rate[2]={0xA5,0x59}; // get sample rate request
volatile unsigned char get_lidar_conf[2]={0xA5,0x84}; // get lidar conf request


volatile unsigned char express_scan_legacy[9]={0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}; // express scan legacy request
volatile unsigned char express_scan_legacy_resp[7]={0xA5,0x5A,0x54,0x00,0x00,0x40,0x82}; // express scan legacy response

volatile unsigned char express_scan_extended[9]={0xA5,0x82,0x05,'M',0x00,0x00,0x00,0x00,'C'}; // express scan legacy request
volatile unsigned char express_scan_extended_resp[7]={0xA5,0x5A,0x84,0x00,0x00,0x40,0x84}; // express scan legacy response

volatile unsigned char express_scan_dens[9]={0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}; // express scan legacy request
volatile unsigned char express_scan_dens_resp[7]={0xA5,0x5A,0x54,0x00,0x00,0x40,0x85}; // express scan legacy response

volatile unsigned char express_scan_dens_data[1]={0xA5};
volatile unsigned   int indexLocal = 0;

volatile unsigned int Angle = 0;
volatile float Distance;
volatile unsigned char s;
volatile unsigned char sNeg;

typedef short int uint16_t;


volatile unsigned char frame_count;
volatile unsigned char cabin_count;
volatile unsigned char rec_byte_count;

volatile unsigned char oRotatie[360] = {0};

volatile char valoareChar[6];
volatile int indexChar = 0;

volatile unsigned char inputAngle[3] = {0};
volatile unsigned char inputAngleIndex = 0;

volatile unsigned i;
volatile unsigned rotationNumber = 0;
void Init_GPIO();

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer

  // Variabile
  i=0;  frame_count=0;  cabin_count=0;  rec_byte_count=0;


  // Configurare CS SMCLK=1MHz CS_09.c
  FRCTL0 = FRCTLPW | NWAITS_2;

  __bis_SR_register(SCG0);                           // disable FLL
  CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
  CSCTL0 = 0;                                        // clear DCO and MOD registers
  CSCTL1 |= DCORSEL_6;  //                             // Set DCO = 20MHz
  CSCTL2 = FLLD_0 + 610; //                            // DCOCLKDIV = 20MHz
  __delay_cycles(3);
  __bic_SR_register(SCG0);                           // enable FLL
  while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked

  CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;        // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
  CSCTL5 = DIVM_0 | DIVS_0; //                                                  // default DCOCLKDIV as MCLK and SMCLK source


  PM5CTL0 &= ~LOCKLPM5;                     // Disable the GPIO power-on default high-impedance mode
                                            // to activate 1previously configured port settings

  // Configuram P3.0 ca MCLK si P3.4 ca SMCLK
  P3DIR |= BIT0 | BIT4; // pini de iesire digitala
  P3SEL0 |= BIT0 | BIT4; // selectam primary function (P3.0 ca MCLK si P3.4 ca SMCLK)

  // Configure UART pins

  P4SEL0 |= BIT2 | BIT3; // selectam functiile UCA1  TxD si RxD
  P1SEL0 |= BIT6 | BIT7; // selectam functiile UCA0  TxD si RxD

  /// Configuram P4.1  ca buton
  P4OUT |= BIT1;                          // Configure P1.3 as pulled-up
  P4REN |= BIT1;                          // P1.3 pull-up register enable
  P4IES |= BIT1;                          // P1.3 Hi/Low edge
  P4IE |= BIT1;                           // P1.3 interrupt enabled
  /// Configuram P2.3  ca buton
   P2OUT |= BIT3;                          // Configure P1.3 as pulled-up
   P2REN |= BIT3;                          // P1.3 pull-up register enable
   P2IES |= BIT3;                          // P1.3 Hi/Low edge
   P2IE |= BIT3;                           // P1.3 interrupt enabled


  // Configure UART UCA1 PC
  UCA1CTLW0 |= UCSWRST;
  UCA1CTLW0 |= UCSSEL_2;    // UCSSEL_1 set ACLK as BRCLK=32768Hz

  // Configure UART UCA0 LIDAR
  UCA0CTLW0 |= UCSWRST;
  UCA0CTLW0 |= UCSSEL_2;
                              // UCSSEL_2 set SMCLK
  // Baud Rate calculation. Referred to UG 17.3.10
  // (1) N=32768/4800=6.827
  // (2) OS16=0, UCBRx=INT(N)=6
  // (4) Fractional portion = 0.827. Refered to UG Table 17-4, UCBRSx=0xEE.

  // Baud Rate A1 PC @ 115200bps
  UCA1BR0 = 0x0A;// @115200bps
  UCA1BR1 = 0x00;// @115200bps
  UCA1MCTLW = 0xADD1; // @115200bps

  // Baud Rate A0 LIDAR @ 115200bps
  UCA0BR0 = 0x0A;// @115200bps
  UCA0BR1 = 0x00;// @115200bps
  UCA0MCTLW = 0xADD1; // @115200bps

  UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt

  UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

  P4IFG &= ~BIT1;                         // P4.1 IFG cleared
  P2IFG &= ~BIT3;                         // P2.3 IFG cleared
  __bis_SR_register(GIE);         // Enter LPM3, interrupts enabled
  __no_operation();                         // For debugger
}

// Port 4 interrupt service routine

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~BIT3;                         // Clear P2.3 IFG

    for(i=0;i<2;i++)
     {
     while(!(UCA1IFG&UCTXIFG));// trimite catre PC
     //UCA1TXBUF = stop_scan[i];
     }

    for(i=0;i<2;i++)
     {
     while(!(UCA0IFG&UCTXIFG));// verifica daca nu se transmite ceva
     UCA0TXBUF = stop_scan[i];
     startScan = 0;
     indexLocal = 0;
     }
}

// Port 4 interrupt service routine

#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    P4IFG &= ~BIT1;                         // Clear P4.1 IFG

   for(i=0;i<2;i++)
    {
    while(!(UCA0IFG&UCTXIFG));// verifica daca nu se transmite ceva
    UCA0TXBUF = start_scan[i];
    }

    startScan = 0;
    indexLocal = 0;

}

unsigned char valoareLocala;

// Rutina de tratare a intreruperilor UART A1 PC
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      while(!(UCA1IFG&UCTXIFG));
      valoareLocala = UCA1RXBUF;
      //UCA1TXBUF = valoareLocala;
      int result = 0;
      for (i = 0; i < 3; i++) {
         result = result * 10 + inputAngle[i];
      }
      if(valoareLocala == 13 && (result <360 && result > 0)){
             for(i=0;i<2;i++)
              {
              while(!(UCA0IFG&UCTXIFG));
              UCA0TXBUF = start_scan[i];
              }
      }
      else{
        if(valoareLocala > 47 && valoareLocala < 58){
                    inputAngle[inputAngleIndex] = valoareLocala - '0';
        inputAngleIndex++;

        if(inputAngleIndex == 3){
          inputAngleIndex = 0;
        }
        }

      }


      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}

void processMessage(unsigned long long *result) {
    // Declare variables

    // Extract individual bytes from the 40-bit result
    unsigned long long buffer[5];
    buffer[0] = (*result >> 32) & 0xFF; // Most significant byte
    buffer[1] = (*result >> 24) & 0xFF;
    buffer[2] = (*result >> 16) & 0xFF;
    buffer[3] = (*result >> 8) & 0xFF;
    buffer[4] = *result & 0xFF;         // Least significant byte

    // Extract bits 6 and 7 from the first byte
    sNeg = (buffer[4] & 0x40) >> 6; // Bit 6
    s = (buffer[4] & 0x80) >> 7;    // Bit 7

    // Calculate the value for Angle
    uint16_t rawAngle = ((buffer[3] & 0xFE) << 8) | buffer[2]; // extragem informatia de la bitul 7 la bitul 1 din primul byte
                                                               // facem append la sirul de informatie pentru unghi
    Angle = rawAngle / 64;              //calculam valoarea fizica a unghiului

    // Calculate the value for distance
    uint16_t rawDistance = (buffer[1] << 8) | buffer[0]; // facem append la sirul de informatie pentru distanta
    Distance = (float)(unsigned int)rawDistance / 4000.0;// calculam distanta fizica

}

void floatToCharArray(float value, char *buffer, unsigned int bufferSize) {
    int i2 = 0;
    if (bufferSize < 2) return; // Bufferul trebuie să aibă cel puțin 2 caractere (un număr și null-terminator)

    int integerPart = (int)value; // Partea întreagă
    float fractionalPart = value - integerPart; // Partea fracțională

    // Convertim partea întreagă în șir
    int index = 0;
    if (integerPart == 0) {
        buffer[index++] = '0';
    } else {
        char temp[20]; // Buffer temporar pentru partea întreagă
        int tempIndex = 0;

        while (integerPart > 0) {
            temp[tempIndex++] = '0' + (integerPart % 10); // Extragem cifrele
            integerPart /= 10;
        }

        // Inversăm cifrele pentru partea întreagă
        while (tempIndex > 0 && index < bufferSize - 1) {
            buffer[index++] = temp[--tempIndex];
        }
    }

    if (index < bufferSize - 1) {
        buffer[index++] = '.'; // Adăugăm punctul zecimal
    }

        // Convertim partea fracțională în șir
    for (i2 = 0; ((i2 < 6) && (index < (bufferSize - 1))); i2++) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;   // Extract the next digit
        buffer[index++] = '0' + digit;     // Convert to character
        fractionalPart -= digit;           // Remove the digit from fractionalPart
    }

    buffer[index] = '\0'; // Adăugăm null-terminatorul
}

// Rutina de tratare a intreruperilor UART A0 LIDAR
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)

{
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
     while(!(UCA1IFG&UCTXIFG));// verifica daca poate transmite catre PC


    dataBuffer[0] = dataBuffer[1];
    dataBuffer[1] = dataBuffer[2];
    dataBuffer[2] = dataBuffer[3];
    dataBuffer[3] = dataBuffer[4];
    dataBuffer[4] = UCA0RXBUF;

    if(dataBuffer[0] == 0x05 && dataBuffer[1] == 0x00 && dataBuffer[2] == 0x00 && dataBuffer[3] == 0x40 && dataBuffer[4] == 0x81){
        startScan  = 1;
    }

    if(startScan){
        indexLocal++;
    }

    if(indexLocal == comparatorIndexDataLocal){
        indexLocal = 0;
        //comparatorIndexDataLocal = 4;
            // Am creat un unsigned long long pentru a pastra datele din mesaj
        unsigned long long result = 0;
        unsigned int resultInput = 0;

        // formam rezultatul din dataBuffer-ul de 40 biti
        result |= (unsigned long long)dataBuffer[0] << 32; // Byte 4 (most significant 8 bits)
        result |= (unsigned long long)dataBuffer[1] << 24; // Byte 3
        result |= (unsigned long long)dataBuffer[2] << 16; // Byte 2
        result |= (unsigned long long)dataBuffer[3] << 8;  // Byte 1
        result |= (unsigned long long)dataBuffer[4];       // Byte 0

        processMessage(&result);// extragem valorile reale

        for (i = 0; i < 3; i++) {
         resultInput = resultInput * 10 + inputAngle[i];
      }
        if(Angle == resultInput){
                    if(Distance != 0){
                        floatToCharArray(Distance, valoareChar, sizeof(valoareChar));
                        while(indexChar < 5){
                            while(!(UCA1IFG&UCTXIFG));
                            UCA1TXBUF = (unsigned char)valoareChar[indexChar];
                            //UCA1TXBUF = '1';
                            indexChar++;
                        }


                        for(i=0;i<2;i++)
                        {


                        while(!(UCA0IFG&UCTXIFG));// verifica daca nu se transmite ceva
                        UCA0TXBUF = stop_scan[i];

                        }

                        for(i= 0; i<3; i++){
                          inputAngle[i] = 0;
                        }
                        startScan = 0;
                    }

        }



    }



    //UCA1TXBUF = UCA0RXBUF;

     indexChar = 0;
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}
