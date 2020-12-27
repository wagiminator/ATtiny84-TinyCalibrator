# TinyCalibrator - OSC Calibrator and High Voltage Fuse Resetter
Because the 8-pin ATtinys only have a few GPIO pins available, they are usually operated without an external clock. The internal oscillator does a good job in most applications, but when it comes to precise timing, it's +/-10% accuracy is often insufficient. Fortunately, the oscillator can be calibrated, increasing its accuracy to +/-2% or better. There are a few ways to perform this manual calibration, but several steps are required. The TinyCalibrator does this fully automatically by a push of a button. In order to make the device more versatile, a high voltage fuse resetter was also integrated, with which "bricked" ATtinys can be reset to the factory state.

- Project Files (EasyEDA): https://easyeda.com/wagiminator/attiny84-tinycalibrator

![pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyCalibrator/main/documentation/TinyCalibrator_pic2.jpg)

# Hardware
The TinyCalibrator is supplied with 5V via a [Micro USB connector](https://aliexpress.com/wholesale?SearchText=micro+usb+2pin+dip). Since the frequency of the oscillator depends on the supply voltage of the ATtiny, an [HT7333](https://datasheet.lcsc.com/szlcsc/Holtek-Semicon-HT7333-A_C21583.pdf) voltage regulator was integrated. A switch can then be used to choose whether the oscillator should be calibrated for 3.3V or 5V.

The [ATtiny84](http://ww1.microchip.com/downloads/en/devicedoc/Atmel-7701_Automotive-Microcontrollers-ATtiny24-44-84_Datasheet.pdf) was chosen as the microcontroller for the TinyCalibrator because it has exactly the necessary number of GPIO pins. For accurate frequency measurements, the ATtiny84 is operated with an external 12 MHz crystal. Since the current software version only requires about 3.7 kByte, an ATtiny44 can also be used.

To generate the 12V for the High Voltage Serial Programmer, an [ST662A](https://datasheet.lcsc.com/szlcsc/2004081608_STMicroelectronics-ST662ACD-TR_C500661.pdf) charge pump IC was chosen, which was specially designed for such applications and needs only a few external components. The 12V is controlled by a MOSFET and applied to the RESET pin of the target ATtiny if necessary. The remaining programming lines to the target are protected against a short circuit with resistors.

The user interface utilizes three buttons and a [128x64 pixels OLED display](http://aliexpress.com/wholesale?SearchText=128+64+0.96+oled+new+4pin).

![pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyCalibrator/main/documentation/TinyCalibrator_pic1.jpg)

# Software
## Basic Principle
To carry out the calibration, a program is first uploaded to the target ATtiny using the integrated High Voltage Serial Programmer. In addition, the factory oscillator calibration value (OSCCAL) is written to the EEPROM. The program on the target ATtiny reads the EEPROM and writes the value to the OSCCAL register. Then it applies an oscillating signal with half the clock frequency to pin PB0. Since the fuses were previously set so that the target ATtiny runs with a prescaler of 8, a signal with 1/16 of the oscillator frequency is applied to PB0.

```c
#include <avr/io.h>
#include <avr/eeprom.h>

int main(void) {
  OSCCAL = eeprom_read_byte(0);
  DDRB   = 0b00000001;
  TCCR0A = 0b01000010;
  TCCR0B = 0b00000001;
  while(1);
}
```

This frequency is measured by the TinyCalibrator and compared with the target value. The oscillator calibration value (OSCCAL) is then adjusted accordingly and written to the EEPROM of the target ATtiny. This process is repeated until the OSCCAL value, which leads to the lowest frequency deviation, has been found.

![calibrating.png](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyCalibrator/main/documentation/TinyCalibrator_calibrating.png)

## High Voltage Serial Programmer
The code for the High Voltage Serial Programmer (HVSP) is quite unspectacular. Simply put, for each action, a series of instructions are sent over the data lines to the target ATtiny and the corresponding response is read. The process and the instructions are well described in the data sheet.

![hvsp.png](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyCalibrator/main/documentation/TinyCalibrator_hvsp.png)

## Frequency Measurement
The timer/counters of the ATtiny84 are used for the frequency measurement. PB0 of the target ATtiny, which outputs a signal with 1/16 of its oscillator frequency, is connected to the T0 input of the ATtiny84. Timer0 counts the pulses at T0 and timer1 stops the measurement after a time of 32 milliseconds. From this, the oscillator frequency of the target ATtiny can finally be calculated.

## I²C OLED Implementation
The I²C protocol implementation is based on a crude bitbanging method. It was specifically designed for the limited resources of ATtiny10 and ATtiny13, but it works with some other AVRs (including the ATtiny84) as well. The functions for the OLED are adapted to the SSD1306 OLED module, but they can easily be modified to be used for other modules. In order to save resources, only the basic functionalities which are needed for this application are implemented. For a detailed information on the working principle of the I²C OLED implementation visit [TinyOLEDdemo](https://github.com/wagiminator/attiny13-tinyoleddemo).

## Compiling and Uploading
### If using the Arduino IDE
- Make sure you have installed [ATtinyCore](https://github.com/SpenceKonde/ATTinyCore).
- Go to **Tools -> Board -> ATtinyCore** and select **ATtiny24/44/84(a) (No bootloader)**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny84(a)
  - **Clock:**          12 MHz (external)
  - **Millis/Micros:**  disabled
  - Leave the rest at the default settings
- Connect your programmer to your PC and to the ICSP header of the device.
- Go to **Tools -> Programmer** and select your ISP programmer (e.g. [USBasp](https://aliexpress.com/wholesale?SearchText=usbasp)).
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open TinyCalibrator sketch and click **Upload**.

### If using the precompiled hex-file
- Make sure you have installed [avrdude](https://learn.adafruit.com/usbtinyisp/avrdude).
- Connect your programmer to your PC and to the ICSP header of the device.
- Open a terminal.
- Navigate to the folder with the hex-file.
- Execute the following command (if necessary replace "usbasp" with the programmer you use):
  ```
  avrdude -c usbasp -p t84 -U lfuse:w:0xff:m -U hfuse:w:0xd5:m -U efuse:w:0xff:m -U flash:w:main.hex
  ```

### If using the makefile (Linux/Mac)
- Make sure you have installed [avr-gcc toolchain and avrdude](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Connect your programmer to your PC and to the ICSP header of the device.
- Open the makefile and change the programmer if you are not using usbasp.
- Open a terminal.
- Navigate to the folder with the makefile and main.c.
- Run "make install" to compile, burn the fuses and upload the firmware.

# References, Links and Notes
1. [TinyHVSP](https://github.com/wagiminator/ATtiny84-TinyHVSP)
2. [I²C OLED Tutorial](https://github.com/wagiminator/attiny13-tinyoleddemo)
3. [ATtiny84 Datasheet](http://ww1.microchip.com/downloads/en/devicedoc/Atmel-7701_Automotive-Microcontrollers-ATtiny24-44-84_Datasheet.pdf)
4. [ATtiny85 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)
5. [ATtiny13A Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf)
