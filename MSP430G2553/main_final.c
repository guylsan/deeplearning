#include  "msp430.h"
#include <string.h>
#include <stdlib.h>


#define UltraPortOut P1OUT
#define UltraPortDirection P1DIR
#define UltraFrontPin BIT4
#define UltraRightPin BIT4
#define UltraEcho BIT1

volatile unsigned int measure; //this is the difference in counts measured by the Ultrasonic timer

volatile unsigned int up=0; //helps the timer determine which edge

int detection;

//can these be switched to static within the timer?  they don't need to be available outside of the timer
unsigned int measure_1 = 0;
unsigned int measure_2 = 0;

void Init_Timer0();
void Attente_Timer0();
void Init_Timer1(unsigned int vM ,unsigned int vA , unsigned int vB);
void Init_moteur();
void Avancer_robot(unsigned int vM ,unsigned int vA ,unsigned int vB);
void Acc_robot(unsigned int vM,unsigned int vA);
void Desc_robot(unsigned int vM,unsigned int vA);
void Tourner_Gauche_robot(unsigned int vM, unsigned int vA);
void Rotation_90G_robot();
void Tourner_Droite_robot(unsigned int vM, unsigned int vA);
void Rotation_90D_robot();
void Reculer_robot(unsigned int vM, unsigned int vA);
void Stop_robot ();
void Homologation (unsigned int vM , unsigned int vA);
void Init_Opto();


/*
 * Generation d'un timer de periode 0.1s f= 10Hz
 */

/*
void Init_Timer0(){
    BCSCTL1 = CALBC1_1MHZ; //frequence d’horloge 1MHz
    DCOCTL = CALDCO_1MHZ;
    TA0CTL |= ( TASSEL_2 | ID_0 | MC_1);
    TA0CCR0 = 25000;
    TA0CTL &= ~TAIFG;
}
*/

void Attente_Timer0(){
    Init_Timer0();
    while( !(TA0CTL & TAIFG));
    TA0CTL &= ~TAIFG;
}


/*
 * Fonction prenant vA et vB en parametre ce qui correspond a la vitesse du moteur A et Moteur B (vA et vB entre 0 et 8)
 */
void Init_Timer1(unsigned int vM ,unsigned int vA , unsigned int vB){
    BCSCTL1 = CALBC1_1MHZ; //frequence d’horloge 1MHz
    DCOCTL = CALDCO_1MHZ;
    TA1CTL |= ( TASSEL_2 | ID_0 | MC_1);
    TA1CCTL1 |= OUTMOD_7;
    TA1CCTL2 |= OUTMOD_7;
    TA1CCR0 = vM;
    TA1CCR1 = vA;
    TA1CCR2 = vB;
}
/*
 * Initialisation des fonction timer pour les moteurs et E/S pour le sens du moteur
 */
void Init_moteur(){
    P2DIR |= ( BIT1 | BIT2 | BIT4 | BIT5 );
    P2SEL |= ( BIT2 | BIT4 );
    P2SEL &= ~( BIT1 | BIT5 );
    P2SEL2 &= ~( BIT1 | BIT2 | BIT4 |BIT5 );

}
//time en 0.1 sec
void Avancer_robot(unsigned int vM ,unsigned int vA , unsigned int vB){
    int i;
    Init_Timer1(vM,vA,vB);	// vB
    Init_moteur();
    P2OUT |= ( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT &= ~( BIT1 );

}


void Tourner_Gauche_robot(unsigned int vM, unsigned int vA){
    Init_Timer1(vM,vA,vA);
    Init_moteur();
    P2OUT |= ( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT |= ( BIT1 );
    Attente_Timer0();
}

void Rotation_90G_robot(){
    int i;
    for ( i = 0 ; i<11 ; i++ ){
        Tourner_Gauche_robot(105,20);
    }
}

void Tourner_Droite_robot(unsigned int vM, unsigned int vA){
    Init_Timer1(vM,vA,vA);
    Init_moteur();
    P2OUT &= ~( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT &= ~( BIT1 );
    Attente_Timer0();
}

void Rotation_90D_robot(){
    int i;
    for ( i = 0 ; i<11 ; i++ ){
        Tourner_Droite_robot(105,20);
    }
}

void Reculer_robot(unsigned int vM, unsigned int vA){
    Init_Timer1(vM,vA,vA);
    Init_moteur();
    P2OUT &= ~( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT |= ( BIT1 );
    Attente_Timer0();
}

void Stop_robot (){
    int i;
    Init_Timer1(10,0,0);
    Init_moteur();
    for( i = 0 ; i < 25 ; i++){
     __delay_cycles(5000);
    }

}


/*
 * Configuration for Trigger Pins - these drive the Ultrasonic device
 */



void get_measure() {
    UltraPortOut |= UltraFrontPin;
    up = 1; //Next catch on Timer1A0 should be rising edge - helps with capture timer
    UltraPortOut &= ~UltraFrontPin;
}
//envoyer
void TXdata( unsigned int c )
{
    while (!(IFG2 & UCA0TXIFG));  // USCI_A0 TX buffer ready?
    UCA0TXBUF = c;              // TX -> RXed character
}


void communication_UART(){
	if (RXD & BIT2)

}
//recevoir
unsigned int UART_RXdata()
{
	while(!(IFG2 & UCA0RXIFG)); 				//receptionUSCI_A0 ? --wait flag
	return UCA0RXBUF;
}
void clignottement_Led(int nb){
	unsigned int i;
	for (i=0; i<(nb) ;i++){
		P1OUT |= BIT0;
		__delay_cycles(1000000);
		P1OUT &= ~BIT0;
		__delay_cycles(1000000);
	}
}
void communication_UART_Led(){
	unsigned int RX;
	RX= UART_RXdata();
	if ((RX & BIT1)==0 && (RX & BIT2)==0){
		clignottement_Led(4);
	}
	if ((RX & BIT1)==2 && (RX & BIT2)==0){
		clignottement_Led(3);
	}
	if ((RX & BIT1)==0 && (RX & BIT2)==4){
		clignottement_Led(2);
	}
	if ((RX & BIT1)==2 && (RX & BIT2)==4){
		clignottement_Led(1);
	}
}
void pilote(){
	Stop_robot ();
	TXdata(detection);
	
	while((UART_RXdata() & BIT0)==0);
	communication_UART_Led();
	__delay_cycles(1000000);
	Rotation_90D_robot();
	detection = 0;
	TXdata(detection);
}



int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT
  UltraPortDirection &= ~UltraEcho;

   //***Timer1A? capture configuration
   //rising & falling edge + synchronous + P2.0 (CCI1A) + capture + capture/compare interrupt enable
   TA0CCTL0 |= CM_3 + CCIS_0 + CAP + CCIE + SCS;
   //select smclock for timer a source + make ta1ccr0 count continously up + no division
  TA0CTL |= TASSEL_2 + MC_2 + ID_0;

  //***Set up pins for Ultrasonic sensing
 UltraPortDirection |= BIT4;
 UltraPortOut &= ~BIT4;//turn off trigger pins to make sure they're in the correct state
 //Set P2.0 to pick up echo from the HC-SR04
 //Not using a #define element for this - it's tied to the timer
  P1SEL |= UltraEcho;
  P1SEL2 &= ~UltraEcho;


  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 104;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600

  P1DIR &= ~BIT3;
  P1IE |= BIT3;
  P1OUT |= BIT3; // pull-up
  P1REN |= BIT3;	// activer la résistance
  P1IES &= ~BIT3;
  P1IFG &= ~(BIT3);
  P1DIR |= BIT0;
  P1OUT &= ~BIT0;

  __enable_interrupt();

/*
 * 	Gestion du bouton poussoir
 */

  while (1) {
      __delay_cycles(50000);
       get_measure();
       if (detection ==1)
    	   	  pilote();
       


  }
}

//Timer1_A Capture
//P1.4 ends up triggering this timer


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer1A0(void)
{
   if(up==1) //is this the rising edge?
   {
      measure_1=TA0CCR0;  // take first time measure
//      serialPrint("{");
   }
   else //is this the falling edge?
   {
      measure_2=TA0CCR0; //take second time measure
      measure=(measure_2-measure_1)/58; // microseconds / 58 = centimeters

      if (measure <20){
    	  P1OUT |= BIT0;
      }
      else
    	  P1OUT &= ~BIT0;

   }
   up=!up; //if this was the rising edge, the next one will be a falling edge, and vice-versa
   TA0CTL &= ~TAIFG; //clear timer A interrupt flag, so the chip knows we handled the interrupt

}

#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void) {

			Avancer_robot(100,100,100);

		P1IFG &= ~(BIT3);

}
