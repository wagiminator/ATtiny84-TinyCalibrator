#include <avr/io.h>
#include <avr/eeprom.h>

int main(void) {
  OSCCAL = eeprom_read_byte(0);
  DDRB   = 0b00000001;
  TCCR0A = 0b01000010;
  TCCR0B = 0b00000001;
  while(1);
}
