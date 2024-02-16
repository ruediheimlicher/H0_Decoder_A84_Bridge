//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "defines.h"
//***********************************
						
uint8_t  LOK_ADRESSE = 0xCC; //	11001100	Trinär (0101)


//									
//***********************************

/*
 commands
 LO     0x0202  // 0000 0010   0000 0010
 OPEN   0x02FE  // 0000 0010   1111 1110
 HI     0xFEFE  // 1111 1110   1111 1110
 */



#define INPORT   PORTB  // Input signal auf INT0
#define INPIN    PINB  // Input signal

#define DATAPIN  2 // PB2, INT0



//volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[buffer_size];



volatile uint8_t	INT0status=0x00;				
volatile uint8_t  pausestatus=0x00;

volatile uint8_t   ablaufstatus=0x00; // Startdlay
volatile uint16_t   startwaitcounter = STARTWAIT;

volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   



volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
//volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;

volatile uint8_t   deffunktiondata = 0;

volatile uint8_t   oldlokdata = 0;
//volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;

//volatile uint16_t   startdelaycounter = 0; // 
//volatile uint16_t   newlokdata = 0;

//volatile uint16_t   blinkWait = 0x2FFF; 
//volatile uint16_t   blinkOK = 0x1FFF; 

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;
//volatile uint32_t   oldrawdata = 0;

volatile uint8_t     oldspeedcode = 0;
volatile uint8_t     speedcode = 0;
volatile uint8_t     speed = 0;
volatile uint8_t     oldspeed = 0;
volatile uint8_t     newspeed = 0;
volatile uint8_t     minspeed = 0; // Unterster Wert in speedlookup-tabelle
volatile uint8_t     startspeed = 0; // Anlaufimpuls

volatile int8_t      speedintervall = 0;

volatile uint8_t   dimmcounter = 0; // LED dimmwertcounter
volatile uint8_t   ledpwm = 0; // LED PWM
volatile uint8_t   ledstatus=0; // status LED

volatile uint8_t   ledonpin = LAMPEA_PIN; // Stirnlampe ON
volatile uint8_t   ledoffpin = LAMPEB_PIN; // Stirnlampe OFF


volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t pwmpin = MOTORA_PIN;           // Motor PWM
volatile uint8_t richtungpin = MOTORB_PIN;      // Motor Richtung

//volatile uint8_t	Potwert=45;
			//	Zaehler fuer richtige Impulsdauer
//uint8_t				Servoposition[]={23,33,42,50,60};
// Richtung invertiert
//volatile uint8_t				Servoposition[]={60,50,42,33,23};

//volatile uint16_t	taktimpuls=0;

volatile uint8_t   motorPWM=0;
volatile uint8_t   motorPWM_n=0;


volatile uint8_t   wdtcounter = 0;

volatile uint8_t   taskcounter = 0;

volatile uint8_t   speedlookup[15] = {};
uint8_t speedlookuptable[10][15] =
{
   {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252},
   {0,30,40,50,60,70,80,90,100,110,120,130,140,150,160},
   {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140},
   {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100},
   {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80},
   
   {0,41,42,44,47,51,56,61,67,74,82,90,99,109,120},
   {0,41,43,45,49,54,60,66,74,82,92,103,114,127,140},
   {0,41,44,48,53,59,67,77,87,99,113,128,144,161,180},
   {0,42,45,50,57,65,75,87,101,116,134,153,173,196,220},
   {0,42,45,51,58,68,79,93,108,125,144,165,188,213,240}
};

volatile uint8_t speedindex = 8;


volatile uint8_t   maxspeed =  0; //speedlookuptable[speedindex][14];

volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0x40;
uint16_t speedchangetakt = 0x400; // takt fuer beschleunigen/bremsen


// https://stackoverflow.com/questions/70049553/best-way-to-handle-multiple-pcint-in-avr
volatile uint8_t portahistory = 0xFF;     // default is high because the pull-up

void slaveinit(void)
{
 	OSZIPORT |= (1<<OSZIA);	//Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZIA);	//Ausgang fuer OSZI A

   LOOPLEDDDR |=(1<<LOOPLED); // HI
   LOOPLEDPORT |=(1<<LOOPLED);

   DDRA |= (1<<PA4); // output
   PORTA &= ~(1<<PA4);// LO

   DDRA |= (1<<AUXA); // output
   PORTA |= (1<<AUXA);// HI
   DDRA |= (1<<AUXB); // output
   PORTA |= (1<<AUXB);// HI

 //  DDRA |= (1<<5);
 //  DDRA |= (1<<6);
   PORTA &= ~(1<<5);
   PORTA &= ~(1<<6);
 
   MOTORDDR |= (1<<MOTORA_PIN);  // Output Motor A 
   MOTORPORT |= (1<<MOTORA_PIN); // HI
   
   MOTORDDR |= (1<<MOTORB_PIN);  // Output Motor B 
   MOTORPORT |= (1<<MOTORB_PIN); // HI


   LAMPEDDR |= (1<<LAMPEA_PIN);  // Lampe A
   LAMPEPORT &= ~(1<<LAMPEA_PIN); // LO

   LAMPEDDR |= (1<<LAMPEB_PIN);  // Lampe B
   LAMPEPORT &= ~(1<<LAMPEB_PIN); // LO

   
   
   maxspeed =  254;

   pwmpin = MOTORA_PIN;
   richtungpin = MOTORB_PIN;
   ledonpin = LAMPEA_PIN;
   ledoffpin = LAMPEB_PIN;
   

}


void int0_init(void)
{
   
   GIMSK |= (1<<INT0); // enable external int0
   MCUCR = (1<<ISC00 | (1<<ISC01)); // raise int0 on rising edge
   
   //INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   //sei();
}

void timer0 (uint8_t wert) 
{ 
   // set up timer with prescaler = 1 and CTC mode
   TCCR0A = 0;
   TCCR0B = 0;

   TCCR0A |= (1<<WGM01);
   TCCR0B |= (1<<CS01);
   
   // initialize counter
   TCNT0 = 0;
   
   // initialize compare value
   OCR0A = wert; //
   
   // clear interrupt flag as a precaution
   TIFR0 |= 0x01;   
   
   // enable compare interrupt
   TIMSK0 |= (1 << OCIE0A);
   
   // enable global interrupts
   //sei();
} 

/*
void pcint7_init(void)
{
   DDRA &= ~(1<<PA7); // PA7 input
   PORTA |= (1<<PA7); // HI
   
   GIMSK |= (1<<PCIE0);    // General Interrupt Mask Register enable for pin 0:7
   PCMSK0 |= (1<<PCINT7);  // Pin Change Mask Register  for PA7
}
*/

// MARK: ISR(PCINT7)
/*
ISR(PCINT0_vect) 
{
   if(PINA & (1 << PA7)) // Source OK
   {
      //LOOPLEDPORT &= ~(1<<LOOPLED);
      PORTA &= ~(1<<PA4);
      
   }
      else // source down
      {
      //LOOPLEDPORT |= (1<<LOOPLED);
      PORTA |= (1<<PA4);
         
   }
   
    GIFR |= (1<<PCIF0);
}
*/

// MARK: ISR(EXT_INT0_vect) 
ISR(EXT_INT0_vect) 
{
   //OSZIATOG;
   if (INT0status == 0) // neue Daten beginnen
   {
      //OSZIALO; 
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
      
      INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
      //OSZIPORT &= ~(1<<PAKETA); 
      //TESTPORT &= ~(1<<TEST2);
      
 //     OSZIBLO;
      
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      funktion = 0;
 //     HIimpulsdauer = 0;
      //OSZIAHI;
   } 
   
   else // Data in Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
 //     OSZIALO;
   }
}



// MARK: ISR Timer0
ISR(TIM0_COMPA_vect) // Schaltet Impuls an MOTORB_PIN LO wenn speed
{
   //OSZIATOG;
   //return;
   if (speed)
   {
      motorPWM++;
   }
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      MOTORPORT |= (1<<pwmpin);      

   }
   
   if (motorPWM >= 254) //ON, neuer Motorimpuls
   {
       MOTORPORT &= ~(1<<pwmpin);

      motorPWM = 0;
      
   }
   
   
   // MARK: TIMER0 TIMER0_COMPA INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++; 
      if (waitcounter > 2)// Impulsdauer > minimum, nach einer gewissen Zeit den Stauts abfragen
      {
         //OSZIAHI;
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
            if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseA |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseA &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // Funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionA |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionA &= ~(1<<(tritposition-8)); // bit ist 0
               }
            }

            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<((tritposition-10))); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
         }
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
            if (tritposition < 8) // Adresse)
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseB |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseB &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionB |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionB &= ~(1<<(tritposition-8)); // bit ist 0
               }
               
            }
            
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<(tritposition-10)); // bit ist 1
               }
               else 
               {
                  rawdataB &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
            
            if (!(lokadresseB == LOK_ADRESSE))
            {
               
            }
         }
         
         // Paket anzeigen
         if (INT0status & (1<<INT0_PAKET_B))
         {
            //           TESTPORT |= (1<<TEST2);
         }
         if (INT0status & (1<<INT0_PAKET_A))
         {
            //           TESTPORT |= (1<<TEST1);
         }
         
         
         if (tritposition < 17)
         {
            tritposition ++;
         }
         else // Paket gelesen
         {
            // Paket A?
            if (INT0status & (1<<INT0_PAKET_A)) // erstes Paket, Werte speichern
            {
               oldfunktion = funktion;
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               
               OSZIATOG; // zwei pakete da
// MARK: EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     //OSZIALO;
                     // Daten uebernehmen
                     //   STATUSPORT |= (1<<DATAOK); // LED ON
                     //  STATUSPORT |= (1<<ADDRESSOK); // LED ON
                     
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     //deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;
                     
                     
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // change setzen
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen
                     }
                     // deflokdata aufbauen
                     for (uint8_t i=0;i<8;i++)
                     {
                        //if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1<<i)))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                     
                     
                     // Richtung
                     if (deflokdata == 0x03) // Wert 1, > Richtung togglen
                     {
                        if (!(lokstatus & (1<<RICHTUNGBIT))) // Start Richtungswechsel
                        {
                           lokstatus |= (1<<RICHTUNGBIT); // Vorgang starten, speed auf 0 setzen
                           richtungcounter = 0;
                           //oldspeed = speed; // behalten
                           //speed = 0;
                           
                           lokstatus |= (1<<LOK_CHANGEBIT); // lok-change setzen
                           ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen

                        } // if !(lokstatus & (1<<RICHTUNGBIT)
                        
                        
                        /* TODO
                        else // repetition 0x03
                        {
                           richtungcounter++;
                           if (richtungcounter > 4)
                           {
                              lokstatus &= ~(1<<RICHTUNGBIT); // Vorgang Richtungsbit wieder beenden, 
                              richtungcounter = 0;
                           }
                        }
                         */
                     } // deflokdata == 0x03
                     else 
                     {  
                        
                        lokstatus &= ~(1<<RICHTUNGBIT); // Vorgang Richtungsbit wieder beenden, 
// MARK: speed           
                         {
                           switch (deflokdata)
                           {
                              case 0:
                                 speedcode = 0;
                                 lokstatus &= ~(1<<STARTBIT);
                                 break;
                              case 0x0C:
                                 speedcode = 1;
                                 break;
                              case 0x0F:
                                 speedcode = 2;
                                 break;
                              case 0x30:
                                 speedcode = 3;
                                 break;
                              case 0x33:
                                 speedcode = 4;
                                 break;
                              case 0x3C:
                                 speedcode = 5;
                                 break;
                              case 0x3F:
                                 speedcode = 6;
                                 break;
                              case 0xC0:
                                 speedcode = 7;
                                 break;
                              case 0xC3:
                                 speedcode = 8;
                                 break;
                              case 0xCC:
                                 speedcode = 9;
                                 break;
                              case 0xCF:
                                 speedcode = 10;
                                 break;
                              case 0xF0:
                                 speedcode = 11;
                                 break;
                              case 0xF3:
                                 speedcode = 12;
                                 break;
                              case 0xFC:
                                 speedcode = 13;
                                 break;
                              case 0xFF:
                                 speedcode = 14;
                                 break;
                              default:
                                 speedcode = 0;
                                 break;
                                 
                           }
                           //speed = speedlookup[speedcode];
                            
                            if(speedcode && (speedcode < 2) && !(lokstatus & (1<<STARTBIT))  && !(lokstatus & (1<<RUNBIT))) // noch nicht gesetzt
                            {
                               startspeed = speedlookup[speedcode] + 1; // kleine Zugabe
                                lokstatus |= (1<<STARTBIT);
                            }

                           oldspeed = speed; // behalten
                        
                           speedintervall = (newspeed - speed)>>2; // 4 teile
                            
                           newspeed = speedlookup[speedcode]; // zielwert
                           
                            if(speedcode > 0)
                            {
                               lokstatus |= (1<<RUNBIT); // lok in bewegung
                            }
                            else
                            {
                               lokstatus &= ~(1<<RUNBIT); // lok steht still
                            }
                           
                        }
                     }
                     
                  }
                  else 
                  {
                     // aussteigen
                     //deflokdata = 0xCA;
                     INT0status = 0;
                     return;
                  }
               }
               else 
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
                  //deflokdata = 0xCA;
                  INT0status = 0;
                  return;
                  
               }
               
               INT0status |= (1<<INT0_END);
               //     OSZIPORT |= (1<<PAKETB);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
            } // End Paket B
         }
         
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
//      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else //if (abstandcounter ) // Paket 2
      {
         abstandcounter = 0;
           // OSZIAHI;
         //     OSZIPORT |= (1<<PAKETA); 
         //    OSZIPORT &= ~(1<<PAKETB);   
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else 
      {
         //OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
      
   } // input LO
} // TIM0


int main (void) 
{
   //WDT ausschalten 
   MCUSR = 0;
   wdt_disable();
     //   lastDIR = 1;
   slaveinit();
   int0_init();
   
   timer0(4);
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   
   
   //_delay_ms(2);
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCC; // 
   
   // WDT
   // https://bigdanzblog.wordpress.com/2015/07/20/resetting-rebooting-attiny85-with-watchdog-timer-wdt/
   /*
    WDTCSR|=(1<<WDCE)|(1<<WDE);  // https://www.instructables.com/ATtiny85-Watchdog-reboot-Together-With-SLEEP-Andor/
    WDTCSR=0x00; // disable watchdog
    */
    #define WDTO_15MS   0
   
     WDTCSR = 0xD8 | WDTO_15MS;
   
   
   //wdt_enable(WDTO_15MS);  // Set watchdog timeout to 15 milliseconds
   wdt_reset();
   ledpwm = LEDPWM;
   minspeed = 0;//speedlookup[1];
   maxspeed = speedlookup[14];
   
   
   uint8_t i = 0;
   for (i=0;i<15;i++)
   {
      speedlookup[i] = speedlookuptable[speedindex][i];
   }

   sei();
   while (1)
   {	
      //OSZIATOG;
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      {
         //PORTA &= ~(1<<PA4); // LED on
         
         
         if(lokstatus & (1<<FUNKTIONBIT))
         {
            
            if(dimmcounter == 3)
            {
               LAMPEPORT |= (1<<ledonpin); // Lampe-PWM  ON
               
            }
            dimmcounter++;
            if(dimmcounter > 32)
            {
               LAMPEPORT &= ~(1<<ledonpin); // Lampe-PWM  OFF
               dimmcounter = 0;
            }
            
         }

         
         loopcount1++;
         if (loopcount1 >= speedchangetakt)
         {
            //MOTORPORT ^= (1<<pwmpin); 
            LOOPLEDPORT ^= (1<<LOOPLED); // Kontrolle lastDIR
            loopcount1 = 0;
            //OSZIATOG;
            
            // speed var
            if((newspeed > oldspeed)) // beschleunigen, speedintervall positiv
            {
               if(speed < (newspeed + speedintervall))
               {
                  if((startspeed > speed) && (lokstatus & (1<<STARTBIT))) // Startimpuls
                  {
                     speed = startspeed;
                     lokstatus &= ~(1<<STARTBIT);
                  }
                  
                  speed += speedintervall;
               }
               else 
               {
                  speed = newspeed;
               }
            }
            else if((newspeed < oldspeed)) // bremsen, speedintervall negativ
            {
               if((speed > newspeed) && ((speed + 2*speedintervall) > 0))
               {
                  speed += 2*speedintervall;
               }
               else 
               {
                  speed = newspeed;
               }
            }
            // end speed var
         } // loopcount1 >= speedchangetakt
         
      }// Source OK
      
      
      loopcount0++;
      if (loopcount0>=refreshtakt)
      {
         //OSZIATOG;
         //LOOPLEDPORT ^= (1<<LOOPLED); 
         
         loopcount0=0;
         
         if(lokstatus & (1<<LOK_CHANGEBIT)) // Motor-Pins tauschen
         {
            if(pwmpin == MOTORA_PIN)
            {
               pwmpin = MOTORB_PIN;
               richtungpin = MOTORA_PIN;
               ledonpin = LAMPEB_PIN;
               ledoffpin = LAMPEA_PIN;
            }
            else // auch default
            {
               pwmpin = MOTORA_PIN;
               richtungpin = MOTORB_PIN;
               ledonpin = LAMPEA_PIN;
               ledoffpin = LAMPEB_PIN;
            }
            MOTORPORT |= (1<<richtungpin); // Richtung setzen
            
            lokstatus &= ~(1<<LOK_CHANGEBIT);
            
         } // if changebit
         
         
         
         // Lampen einstellen
         if(ledstatus & (1<<LED_CHANGEBIT))
         {
            if(lokstatus & (1<<FUNKTIONBIT))
            {
               LAMPEPORT |= (1<<ledonpin); // Lampe  ON
               LAMPEPORT &= ~(1<<ledoffpin); // // Lampe  OFF
               
            }
            else
            {
               // beide lampen OFF
               LAMPEPORT &= ~(1<<ledonpin); // // eingeschaltete Lampe  OFF
              //LAMPEPORT &= ~(1<<LAMPEB_PIN); // Lampe B OFF
              // LAMPEPORT &= ~(1<<LAMPEA_PIN); // Lampe A OFF
            }
            ledstatus &= ~(1<<LED_CHANGEBIT);
         }

         
         
         
             
         if (deflokadresse == LOK_ADRESSE)
         {
            //OSZIATOG;
         }
         else
         {
            //OSZIAHI;
         }
         
         
      }  // loopcount0>=refreshtakt
      
      //OSZIAHI;
   }//while
   return 0;
}
