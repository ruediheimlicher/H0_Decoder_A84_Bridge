//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//

#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <avr/sleep.h>
#include <avr/wdt.h>
#include "defines.h"
#include "lcd.c"
//***********************************
						
uint8_t  LOK_ADRESSE = 0xCC; //	11001100	Trinär DIP: (0101)  Ae6/6 GR

//uint8_t  LOK_ADRESSE = 0xF0; //   11110000   Trinär (1100)  Dampflok Vinv (?)

//uint8_t  LOK_ADRESSE = 0b11110000; //   11110000 


//n ^ ((1 << 31) - 1)
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

#define STARTKICK 4 // Verlaengerung erster Puls

//volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[buffer_size];



volatile uint8_t	INT0status=0x00;				
volatile uint8_t  pausestatus=0x00;

volatile uint8_t   ablaufstatus=0x00; // Startdlay
volatile uint16_t  startwaitcounter = STARTWAIT;

volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   


volatile uint16_t    saveEEPROM_Addresse = 0;
volatile uint8_t     EEPROM_savestatus=0x00;   
volatile uint8_t     EEPROM_lastsavedstatus = 0;


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
volatile uint8_t   lokstatus=0x00; // Funktion

volatile uint8_t   richtungstatus=0x00; //  Richtung

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
volatile uint8_t     startspeed = 1; // Anlaufimpuls


// 16bit
volatile uint16_t     oldspeed16 = 0;
volatile uint16_t     newspeed16 = 0;
volatile uint16_t     minspeed16 = 0; // Unterster Wert in speedlookup-tabelle
volatile uint16_t     startspeed16 = 0; // Anlaufimpuls
volatile uint16_t     speedintervall16 = 0;

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
   {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252},  // 0
   {0,30,40,50,60,70,80,90,100,110,120,130,140,150,160},    // 1
   {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140},      // 2
   {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100},           // 3
   {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80},           // 4
   
   {0,41,42,44,47,51,56,61,67,74,82,90,99,109,120},         // 5
   {0,41,43,45,49,54,60,66,74,82,92,103,114,127,140},       // 6
   
   {0,60,65,70,77,90,105,122,140,159,170,188,200,210,220},  // 7
   
   {0,32,38,46,57,65,75,87,101,116,134,153,173,196,220},    // 8
   {0,25,28,32,38,46,55,65,77,90,105,122,140,159,180}       // 9 
};
// {0,25,28,32,38,46,55,65,77,90,105,122,140,159,180}
volatile uint8_t speedindex = 8;


volatile uint8_t   maxspeed =  0; //speedlookuptable[speedindex][14];

volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0x50;
uint16_t speedchangetakt = 0x400; // takt fuer beschleunigen/bremsen


// https://stackoverflow.com/questions/70049553/best-way-to-handle-multiple-pcint-in-avr
volatile uint8_t portahistory = 0xFF;     // default is high because the pull-up

uint16_t lasteepromaddress = MAX_EEPROM - 1; // letzte benutzte Adresse, max je nach typ
uint8_t lasteepromdata = 0;



volatile uint8_t   loopstatus=0x00;
uint16_t firstruncount0=0;
uint16_t firstruncount1=0;

void watchdogSetup(void) 
{
    // Reset the watchdog reset flag
    wdt_reset();
    wdt_enable(WDTO_4S);
    // Enable interrupts instead of reset
    WDTCSR |= (1 << WDIE);
}

ISR(WDT_vect) {
  // OSZIATOG;
/* ReEnable the watchdog interrupt,
 * as this gets reset when entering this ISR and automatically enables the WDE signal
 * that resets the MCU the next time the  timer overflows */
  WDTCSR |= (1<<WDIE);
}

void slaveinit(void)
{
 	
	OSZIDDR |= (1<<OSZIA);	//Ausgang fuer OSZI A
   OSZIPORT |= (1<<OSZIA);   //Ausgang fuer OSZI A

   LOOPLEDDDR |=(1<<LOOPLED); // HI
   LOOPLEDPORT |=(1<<LOOPLED);

//   DDRA |= (1<<PA4); // output
//   PORTA &= ~(1<<PA4);// LO


 
   MOTORDDR |= (1<<MOTORA_PIN);  // Output Motor A 
   MOTORPORT |= (1<<MOTORA_PIN); // HI
   
   MOTORDDR |= (1<<MOTORB_PIN);  // Output Motor B 
   MOTORPORT |= (1<<MOTORB_PIN); // HI

   LAMPEDDR |= (1<<LAMPEA_PIN);  // Lampe A
   LAMPEPORT &= ~(1<<LAMPEA_PIN); // LO

   LAMPEDDR |= (1<<LAMPEB_PIN);  // Lampe B
   LAMPEPORT &= ~(1<<LAMPEB_PIN); // LO
 
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
ISR(TIM0_COMPA_vect) // max 10 us   Schaltet Impuls an MOTORB_PIN LO wenn speed
{
   if (speed)
   {
      motorPWM++;
   }
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      MOTORPORT |= (1<<pwmpin);    // Motor OFF 
   }
   
   if (motorPWM >= 250) //ON, neuer Motorimpuls
   {
      MOTORPORT &= ~(1<<pwmpin); // Motor ON
      motorPWM = 0;      
   }
   
   // MARK: TIMER0 TIMER0_COMPA INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++; 
      if (waitcounter > 2)// Impulsdauer > minimum, nach einer gewissen Zeit den Stauts abfragen
      {
         //OSZIATOG;
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
               
               //OSZIATOG; // zwei pakete da
// MARK: EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     // Daten uebernehmen
                     //OSZIATOG;
                     //LOOPLEDPORT ^=(1<<LOOPLED);
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
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
                     
                    // MARK: RICHTUNG 
                     // Richtung
                     if (deflokdata == 0x03) // Wert 1, > Richtung togglen
                     {
                        if (richtungstatus == 0) // letzter Richtungswechel ist abgeschlossen
                        {
                           richtungstatus |= (1<<RICHTUNGCHANGESTARTBIT); // Vorgang starten, speed auf 0 setzen
                        
                           lokstatus |= (1<<LOK_CHANGEBIT); // lok-change setzen
                           ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen

                        } // if !(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT)
                        
                     } // deflokdata == 0x03
                     
                     else  // speed anpassen
                     {  
                        // richtungswechselauftrag resetten bei erfolg
                        if(richtungstatus &(1<<RICHTUNGCHANGEOKBIT)) // richtungswechsel ist erfolgt
                        {
                           richtungstatus = 0;
                        }
// MARK: speed           
                        {
                           switch (deflokdata)
                           {
                              case 0:
                                 speedcode = 0;
                                 lokstatus &= ~(1<<STARTBIT); // Stillstand markieren, bereit fuer Start
                                 lokstatus &= ~(1<<RUNBIT); 
                                 
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
                           // speedcode ist 1, lok kommt aus stillstand
                           
                           oldspeed = speed; // behalten
                           newspeed = speedlookup[speedcode]; // solllwert
                           // MARK: STARTBEDINGUNG
                           if((speedcode == 1) && !(lokstatus & (1<<STARTBIT))  && !(lokstatus & (1<<RUNBIT))) // Start, noch nicht gesetzt  
                           {
                              startspeed = speedlookup[STARTINDEX] ;
                              oldspeed = speedlookup[1] / 2;
                              newspeed = speedlookup[1]; // kleine Zugabe
                              lokstatus |= (1<<STARTBIT);
                           }
                           else
                           {
                              newspeed = speedlookup[speedcode]; // zielwert
                           }
                           
                           speedintervall = (newspeed - oldspeed)>>2; // 4 teile
                           if((speedcode > 2) && (speedintervall > 4) )
                           {
                              speedintervall = 4;
                           }
                           
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
               //if (INT0status & (1<<INT0_PAKET_B))
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
   //OSZIAHI;
} // ISR (TIM0)

// EEPROM

// Funktion, um ein Byte in den EEPROM zu schreiben
void EEPROM_Write(uint16_t address, uint8_t data) {
    eeprom_update_byte((uint8_t*)address, data);
}

// Funktion, um ein Byte aus dem EEPROM zu lesen
uint8_t EEPROM_Read(uint16_t address) {
    return eeprom_read_byte((uint8_t*)address);
}

void EEPROM_Clear(void) 
{
   uint16_t addr = 0;
   while (addr++ < MAX_EEPROM)
   {
      EEPROM_Write(addr, 0xFF);
   }
}
// Function to read a byte from the EEPROM from ChatGPT
uint8_t EEPROM_read(uint16_t address) {
    // Wait for completion of previous write
    while (EECR & (1 << EEPE));

    // Set up address register
    EEAR = address;

    // Start EEPROM read by writing EERE
    EECR |= (1 << EERE);

    // Return data from the data register
    return EEDR;
}



int main (void) 
{
   //loopstatus |= (1<<FIRSTRUNBIT);
   
   uint8_t value = 0xCC;  // Example value: 170 in decimal
   uint8_t inverted = ~value;   // Bitwise NOT operation

   printf("Original:  0x%02X (%u)\n", value, value);
   printf("Inverted:  0x%02X (%u)\n", inverted, inverted);


   
   slaveinit();
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   
   
   uint8_t i = 0;
   for (i=0;i<15;i++)
   {
      speedlookup[i] = speedlookuptable[speedindex][i];
   }
   maxspeed = speedlookup[14];
   minspeed = speedlookup[1];
   
   // EEPROM
   // naechste Adresse fuer save Status
    for (uint16_t loc = 0;loc < MAX_EEPROM; loc++)
   {   
      uint8_t locdata = EEPROM_Read(loc);
      if(locdata == 0xFF)
      {
         saveEEPROM_Addresse = loc;
         break;
      }
   }
   wdt_reset();
   
   if(saveEEPROM_Addresse)
   {
      EEPROM_lastsavedstatus = EEPROM_Read(saveEEPROM_Addresse - 1);
      // last data
      uint8_t lastdircode = (EEPROM_lastsavedstatus & 0x0C) >> 2;
      
      
      if (lastdircode == 2)
      {
         pwmpin = MOTORA_PIN;
         richtungpin = MOTORB_PIN;
         ledonpin = LAMPEA_PIN;
         ledoffpin = LAMPEB_PIN;
      }
      else if (lastdircode == 1)
      {
         pwmpin = MOTORB_PIN;
         richtungpin = MOTORA_PIN;
         ledonpin = LAMPEB_PIN;
         ledoffpin = LAMPEA_PIN;
      }
    }
   else // default
   {
      pwmpin = MOTORA_PIN;
      richtungpin = MOTORB_PIN;
      ledonpin = LAMPEA_PIN;
      ledoffpin = LAMPEB_PIN;
   }
   // EEPROM
   
   wdt_reset();
   int0_init();
   timer0(4);
   sei();
   
   
   // MARK: WHILE
   while (1) // 150us
   {	
      if(loopstatus & (1<<FIRSTRUNBIT))
      {
         //OSZIALO;
         _delay_ms(2);
         //OSZIAHI;
         firstruncount0++;
         if (firstruncount0>=0x02)
         {
              
            firstruncount0=0;
            // Takt for display
            firstruncount1++;
              //OSZI_A_HI();
            loopstatus &= ~(1<<FIRSTRUNBIT);
         }
         
      }// end firstrun
        // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      
      {
         loopcount1++;
         
         // MARK: SPEEDCHANGETAKT
         // ************************************************
         // speedchangetakt
         // ************************************************
         if (loopcount1 >= speedchangetakt) // speed aendern
         {
             loopcount1 = 0;
            LOOPLEDPORT ^=(1<<LOOPLED);
            // MARK: SPEED VAR
            // speed var
            if((newspeed > speed)) // beschleunigen, speedintervall positiv
            {
               //OSZIALO;
               //OSZI_B_LO();
               if(speed < (newspeed - speedintervall)) 
               {
                  if((startspeed > speed) && (lokstatus & (1<<STARTBIT))) // Startimpuls
                  {
                     speed = startspeed;
                     lokstatus &= ~(1<<STARTBIT);
                  }
                  else 
                  {
                     speed += speedintervall;
                  }
                  
               }
               else 
               {
                  
                  speed = newspeed;
                  
               }
               //OSZIAHI;
               
            }
            else if((newspeed < speed)) // bremsen, speedintervall negativ
            {
               uint8_t bremsintervall = (speedintervall / 4 * 3);
               if((speed > newspeed ) && (bremsintervall > 0))
               {
                  speed += bremsintervall; // : war 2*speedintervall
                  if(speed <= minspeed)
                  {
                     if((newspeed == 0) ) // Motor soll abstellen
                     {
                        speed = 0; // Motor OFF
                     } // if newspeed == 0
                     
                  } //  if(speed <= minspeed/4)
                  
               }
               else 
               {
                  speed = newspeed;
               }
               //OSZI_A_HI();
            } // newspeed < speed
            
            // speed == 0 start
            
            if (speed == 0) // Stillstand erreicht
            {
               // MARK: RICHTUNG CHANGE
               if(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT))
               {
                  wdt_reset();
                  if(lokstatus & (1<<LOK_CHANGEBIT)) // Motor-Pins tauschen
                  {
                     // EEPROM sichern
                     EEPROM_savestatus &= ~0xF0;
                     EEPROM_savestatus |= ((speedcode & 0x0F) << 4);
                     if(pwmpin == MOTORA_PIN) // > auf MOTORB wechseln
                     {
                        pwmpin = MOTORB_PIN;
                        richtungpin = MOTORA_PIN;
                        EEPROM_savestatus &= ~(1<<MOTORA_PIN);
                        EEPROM_savestatus |= (1<<MOTORB_PIN);
                        
                        if(lokstatus & (1<<FUNKTIONBIT)) // Funktion ist 1, einschalten
                        {
                           EEPROM_savestatus |= (1<<LAMPEA_PIN);
                           EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                        }
                        else // funktion ist 0, ausschalten
                        {
                           // beide lampen OFF
                           EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                           EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                        }
                     }
                     else  if(pwmpin == MOTORB_PIN)// auch default
                     {
                        pwmpin = MOTORA_PIN;
                        richtungpin = MOTORB_PIN;
                        EEPROM_savestatus &= ~(1<<MOTORB_PIN);
                        EEPROM_savestatus |= (1<<MOTORA_PIN);
                        
                        if(lokstatus & (1<<FUNKTIONBIT)) // Funktion ist 1, einschalten
                        {
                           EEPROM_savestatus |= (1<<LAMPEB_PIN);
                           EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                        }
                        else // funktion ist 0, ausschalten
                        {
                           // beide lampen OFF
                           EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                           EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                        }
                        
                     }
                     //OSZIALO;
                     EEPROM_Write(saveEEPROM_Addresse,EEPROM_savestatus);
                     //OSZIAHI;
                     if(saveEEPROM_Addresse > 20)
                     {
                        EEPROM_Clear();
                        saveEEPROM_Addresse = 0;
                     }
                     else 
                        
                     {
                        
                        saveEEPROM_Addresse++;
                     }
                     
                     MOTORPORT |= (1<<richtungpin); // Richtung wechseln
                     
                     lokstatus &= ~(1<<LOK_CHANGEBIT);
                     lokstatus &= ~(1<<RUNBIT);
                     
                     richtungstatus |= (1<<RICHTUNGCHANGEOKBIT); // Richtungswechsel erfolgt
                     
                  } // if changebit
                  
                  
                  
                  
                  //OSZI_A_HI();
                  
               } // if(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT))
               
               
               
            } // if speeed == 0
            
            // speed == 0 end
            
            
            // Lampen einstellen
            wdt_reset();
            if(ledstatus & (1<<LED_CHANGEBIT))
            {
               if(richtungpin == MOTORA_PIN)
               {
                  ledonpin = LAMPEB_PIN;
                  ledoffpin = LAMPEA_PIN;
               }
               else 
               {
                  ledonpin = LAMPEA_PIN;
                  ledoffpin = LAMPEB_PIN;
               }
               //OSZI_B_LO();
               if(lokstatus & (1<<FUNKTIONBIT))
               {
                  
                  LAMPEPORT |= (1<<ledonpin); // Lampe         +
                  LAMPEPORT &= ~(1<<ledoffpin); // // Lampe    0
                  
               }
               else
               {
                  // beide lampen OFF
                  //OSZI_A_HI();
                  LAMPEPORT &= ~(1<<ledonpin); // // eingeschaltete Lampe  OFF
                  
               }
               //OSZI_B_HI();
               ledstatus &= ~(1<<LED_CHANGEBIT);
            }
            
            //displaydata[SPEED] = speed;
            
         } // loopcount1 >= speedchangetakt // loopcount1 >= speedchangetakt
         
      }// Source OK
      
      
      loopcount0++;
      
      //OSZIAHI;
   }//while
   return 0;
}
