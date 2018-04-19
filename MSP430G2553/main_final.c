#include  "msp430.h"
#include <string.h>
#include <stdlib.h>


#define UltraPortOut P1OUT
#define UltraPortDirection P1DIR
#define UltraFrontPin BIT4
#define UltraRightPin BIT4
#define UltraEcho BIT1

volatile unsigned int compteur; //this is the difference in counts measured by the Ultrasonic timer

volatile unsigned int up=0; //helps the timer determine which edge

unsigned int detection=0;

unsigned int attente=0;

unsigned int measure_cm; //distance mesuré par le capteur ultrason en centimètre

unsigned char RX;		//Variable recu du raspberry


void Attente_Timer0();
void Init_Timer1(unsigned int vA , unsigned int vB);
void Init_moteur();
void Avancer_robot(unsigned int vA ,unsigned int vB);
void Acc_robot(unsigned int vM,unsigned int vA);
void Desc_robot(unsigned int vM,unsigned int vA);
void Tourner_Gauche_robot( unsigned int vA);
void Rotation_90G_robot();
void Tourner_Droite_robot( unsigned int vA);
void Rotation_90D_robot();
void Reculer_robot( unsigned int vA);
void Stop_robot ();
void Homologation ( unsigned int vA);
void Init_Opto();


/*
 * Generation d'un timer de periode 0.1s f= 10Hz
 */


void Init_Timer0(){
    BCSCTL1 = CALBC1_1MHZ; //frequence d’horloge 1MHz
    DCOCTL = CALDCO_1MHZ;
    TA0CTL |= ( TASSEL_2 | ID_0 | MC_1);
    TA0CCR0 = 10;
    TA0CTL &= ~TAIFG;
}
void Attente_Timer0(){

    while( !(TA0CTL & TAIFG));
    TA0CTL &= ~TAIFG;
}

/*
 * Fonction prenant vA et vB en parametre ce qui correspond a la vitesse du moteur A et Moteur B (vA et vB entre 0 et 8)
 */


void Init_Timer1(unsigned int vA , unsigned int vB){
     //frequence d’horloge 1MHz

    TA1CTL |= ( TASSEL_2 | ID_0 | MC_1);
    TA1CCTL1 |= OUTMOD_7;
    TA1CCTL2 |= OUTMOD_7;
    TA1CCR0 = 50000;	//50 ms
    TA1CCR1 = vA;
    TA1CCR2 = vB;
    TA1CTL &= ~TAIFG;
}
void Attente_Timer1(){

    while( !(TA1CTL & TAIFG));
    TA1CTL &= ~TAIFG;
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
void Avancer_robot(unsigned int vA , unsigned int vB){

    Init_Timer1(vA,vB);	// vB
    Init_moteur();
    P2OUT |= ( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT &= ~( BIT1 );

}


void Tourner_Gauche_robot( unsigned int vA){
    Init_Timer1(vA,vA);
    Init_moteur();
    P2OUT |= ( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT |= ( BIT1 );

}

void Rotation_90G_robot(){
    int i;
    for ( i = 0 ; i<11 ; i++ ){
        Tourner_Gauche_robot(40000);
    }
}

void Tourner_Droite_robot( unsigned int vA){
    Init_Timer1(vA,vA);
    Init_moteur();
    P2OUT &= ~( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT &= ~( BIT1 );

}

void Rotation_90D_robot(){

    Tourner_Droite_robot(40000);
    __delay_cycles(450000);
    Stop_robot ();
}

void Reculer_robot( unsigned int vA){
    Init_Timer1(vA,vA);
    Init_moteur();
    P2OUT &= ~( BIT5 ); // Dans ce cas P2.5 = 1 et P2.1 = 0 avancer
    P2OUT |= ( BIT1 );

}

void Stop_robot (){
    int i;
    Init_Timer1(0,0);
    Init_moteur();
    for( i = 0 ; i < 25 ; i++){
     __delay_cycles(5000);
    }

}

void InitUART(void)
{
    P1SEL |= (BIT1 | BIT2);                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= (BIT1 | BIT2);                    // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 = UCSWRST;                         // SOFTWARE RESET
    UCA0CTL1 |= UCSSEL_3;                       // SMCLK (2 - 3)

    UCA0CTL0 &= ~(UCPEN | UCMSB | UCDORM);
    UCA0CTL0 &= ~(UC7BIT | UCSPB | UCMODE_3 | UCSYNC); // dta:8 stop:1 usci_mode3uartmode
    UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

    UCA0BR0 = 104;                     			// 1MHz, OSC16, 9600 (8Mhz : 52) : 8/115k
    UCA0BR1 = 0;                                // 1MHz, OSC16, 9600
    UCA0MCTL = 10;

    /* Enable USCI_A0 RX interrupt */
    IE2 |= UCA0RXIE;
}
/*
 * Fonction qui permet d'envoyer un trigger sur le port P1.4
 */
void get_measure() {
    UltraPortOut |= UltraFrontPin;
    up = 1;
    UltraPortOut &= ~UltraFrontPin;
}
/*
 * Envoyer UART
 */
void TXdata( unsigned char c )
{
    while (!(IFG2 & UCA0TXIFG));  // USCI_A0 TX buffer ready?
    UCA0TXBUF = c;              // TX -> RXed character
}

/*
 * Recevoir UART
 */
unsigned int UART_RXdata()
{
	while(!(IFG2 & UCA0RXIFG)); 				//receptionUSCI_A0 ? --wait flag
	P1OUT ^= BIT0;
	IFG2 &= ~UCA0RXIFG;
	return UCA0RXBUF;
}

// Clignottement de la lED Rouge
void clignottement_Led_R(int nb){
	unsigned int i;
	for (i=0; i<(nb) ;i++){
		P1OUT |= BIT0;
		__delay_cycles(1000000);
		P1OUT &= ~BIT0;
		__delay_cycles(1000000);
	}
}
// Clignottement de la lED Verte
void clignottement_Led_V(int nb){
	unsigned int i;
	for (i=0; i<(nb) ;i++){
		P1OUT |= BIT6;
		__delay_cycles(1000000);
		P1OUT &= ~BIT6;
		__delay_cycles(1000000);
	}
}
void test_reception(int test){
	if (test != 0)
		clignottement_Led_R(5);

}
/*
 * Fonction traduction ascii
 */
unsigned int traduction_ascii(unsigned char c){
	unsigned int valeur;
	if (c=='a') valeur=1;
	if (c=='b') valeur=2;
	if (c=='c') valeur=3;
	if (c=='d') valeur=4;
	if (c=='e') valeur=0;
	return valeur;
}



// Clignottement de la lED en fonction du message recu du Raspberry
void communication_UART_Led(char detection){

	P1OUT &= ~BIT0;
	clignottement_Led_V(1);
	clignottement_Led_R(detection);
	clignottement_Led_V(2);
}

/*
 * Fonction cerveau robot
 */
void pilote(){
	unsigned int valeur;
	Stop_robot ();
	detection = BIT0;
 	TXdata(detection);
	while(attente==0);
	 __disable_interrupt();
	detection = 0x00;
	TXdata(detection);
	P1OUT ^= BIT0;
	valeur= traduction_ascii(RX);
	communication_UART_Led(valeur);
	//__delay_cycles(100000);

	Rotation_90D_robot();
	//Rotation_90G_robot();
	__delay_cycles(500000);
	Avancer_robot(50000,50000);
	attente=0;
	__enable_interrupt();
}



int main(void)
{
   WDTCTL = WDTPW + WDTHOLD;
  /*
   * Initialisation des timer
   */
  Init_Timer0();
  Init_Timer1(0,0);

  IFG2 &= ~UCA0RXIFG;

  TA1CTL |= TAIE;   //Autorisation des interruptions sur le timer

  UltraPortDirection |= BIT4; //Gestion du port P1.4 pour le trigger
  UltraPortOut &= ~BIT4;  //turn off trigger pins

  P1SEL |= UltraEcho;
  P1SEL2 &= ~UltraEcho;

  /*
   * Gestion du port P1.5 pour la réception du signal d'echo et l'autorisation d'interruption sur celui-ci.
   */
  P1DIR &= ~BIT5;
  P1IE |= BIT5;
  P1IES &= ~BIT5;
  P1IFG &= ~BIT5;

  /*
   * Gestion des horloges de l'UART et baudrate
   */
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 104;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600

  /*
   * Bouton poussoir
   */
  P1DIR &= ~BIT3;	// Port P1.3 en entrée
  P1IE |= BIT3;		// Interruption sur le bouton poussoir
  P1OUT |= BIT3;    // pull-up
  P1REN |= BIT3;	// activer la résistance
  P1IES &= ~BIT3;
  P1IFG &= ~(BIT3);	//flag a 0

  /*
   * LEDs
   */
  P1DIR |= (BIT0|BIT6);
  P1OUT &= ~(BIT0|BIT6);

  InitUART();
  __enable_interrupt();


  while (1)

  {
	 // Avancer_robot(50000,50000);
	   if (detection == 0x01)
    	   	  pilote();
	   	   	 //  Avancer_robot(50000,50000);
  }

}


#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1A0(void){
   get_measure();
    TA1CTL &= ~TAIFG;
}
#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void) {

	if (P1IFG & BIT5){
	compteur=0;
		while ((P1IN & BIT5)!=0){

			compteur++;
		}
		measure_cm = (compteur*10)/58;
		if (measure_cm <10){
		    	  detection = 0x01;
		    	  P1OUT |= BIT0;


		      }
		      else{
		    	  P1OUT &= ~BIT0;
		      }
		P1IFG &= ~(BIT5);
	}
	if(P1IFG & BIT3)
	{
		Avancer_robot(50000,50000);
		P1IFG &= ~(BIT3);
	}

}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
RX = UART_RXdata();
attente=1;
}


