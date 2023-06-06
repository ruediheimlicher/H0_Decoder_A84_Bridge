//
//  usi.c
//  H0_Decoder85
//
//  Created by Ruedi Heimlicher on 18.11.2020.
//

#include "usi.h"

/*********************************************************
 USI to SPI communication routine
 Controller: ATtiny 44, Osc. = 1 MHz
 *********************************************************/

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

void SPI_init(void)
{
   DDRA |= (1<<PA4)|(1<<PA5); //Output: PA4=USCK, PA5=DO
   DDRA &= ~(1<<PA6); //Input: PA6=DI
   PORTA |= (1<<PA6); //int. Pull-up an PA6 aktivieren
   DDRA |= (1<<PA7); //LED auf Ausgang
   
   USICR = (1<<USIWM0) | (1<<USICLK)|(1<<USITC)| (1<<USICS1);
   // USIWM0 = 1: SPI-Betrieb mit DO, DI und USCK
   // USICLK = 1, USITC = 1, USICS1 = 1: generiert den Taktimpuls für das Schieberegister und
   // den 4bit Timer. Der Takt wird am USCK-Pin (PA4) ausgegeben.
}

void SPI_transfer(int data)
{
   USIDR = data; //Daten in Data Register laden
   USISR |= (1<<USIOIF); //Überlaufflag löschen
   
   while (!(USISR & (1<<USIOIF))) //solange kein Timerüberlauf...
   {
      USICR |= (1<<USITC); //Toggle clk (2x)
      USICR |= (1<<USITC);
   }
}

int main (void)
{
   SPI_init();
   
   while(1)
   {
      SPI_transfer(0xAA); // Daten (AA) senden
      
      PORTA |= (1<<PA7); // Led on
      _delay_ms(1000);
      PORTA &= !(1<<PA7); // Led off
      _delay_ms(1000);
   }
}
