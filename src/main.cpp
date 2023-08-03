/*
 * Note-Started using template Jan 2016 for compiler V 1.6.7
 * @TODO-always remember to update version history and programed compiled with below
 *
 * Circuit details below:
 *
 *
 */

//*****library inclusions below
// needed for platforIO, not needed if using arduino IDE
#include <Arduino.h>
// #include <CRC32.h> //@TODO-implement CRC for all EEPROM values

//*****global variables below
// general global variables
boolean infoON = true; // enable things to see status of device, wastes battery if not needed (example- print data to serial and blinkLED). Does not apply to first part of setup()
float firmwareV;
boolean debug = true; // enable verbose debug messages to serial
// pin to show sending of message
#define LEDPIN 9

//*******************************************for EEPROM**************************************************
#include <EEPROM.h>
//NOTE-EEPROM values are ints that start at 0
const int nodeIdAddr = 0;     // starting EEPROM address of the 4 bit nodeID and its 4 bit CRC32
//const int minRawAccAddr = 8;  // starting EEPROM address of the 2 bit minium acceleration (but 4 bytes reserved) to trigger interupt and its 4 bit CRC32
//const int maxRawAccAddr = 16; // starting EEPROM address of the 2 bit max acceleration (but 4 bytes reserved) and its 4 bit CRC32
//*******************************************END EEPROM**************************************************

//*******************************************for lora radio******************************************************
//#include <SPI.h> //SPI used by lora and flash chip
#include <RH_RF95.h>
#define RFM95_CS 10                      // chip select for SPI
#define RFM95_INT 2                      // interupt pin
unsigned long nodeID = 5;                // up to 2 million for lorawan?
const boolean forceChangeNodeID = false; // if you want to force the IMU to write this hard coded nodeID to EEPROM and use it. Only needed if you need to change it after it has been programmed
float frequency = 904.0;                 // Specify the desired frequency in MHz
// the transmit power in dB. -4 to 20 in 1 dB steps. NOTE-function itself seems to indicate a range of 2-20 dB or 0-15 dB @TODO-research
int8_t TXpower = 20;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//*******************************************END lora radio**************************************************

//******functions- if declared before used, don't need Function Prototypes (.h) file? @TODO-is this preferred format for C++?
void writeDataWithChecksumToEEPROM(int address, unsigned long data)
{
  // Write the integer data to EEPROM
  EEPROM.put(address, data);

  // Calculate the CRC32 checksum of the data and save it to EEPROM
  //@TODO-get this to work
  // uint32_t checksum = CRC32::calculate(data, sizeof(data));
  /* this works
   *uint8_t byteBuffer[] = "Hello World";
   *uint32_t checksum = CRC32::calculate(byteBuffer, sizeof(data));
   */

  uint32_t checksum = 3545618; // good enough for now
  EEPROM.put(address + sizeof(data), checksum);
}

boolean testEEPROMchecksum(int address)
{
  //NOTE- for now it uses a static 4 byte value for checksum, not CRC32
  //reads 4 bytes starting at address, then looks at the next 4 bytes for CRC32 value (@TODO)
  //returns- true if calculated CRC32 matches the data read, false if checksum does not match
  
  unsigned long data;
  // read the data from EEPROM
  EEPROM.get(address, data); // sets 'data' to value here (also returns a refernece to data)
  uint32_t checksumEEPROM;
  EEPROM.get(address + sizeof(data), checksumEEPROM);

  // Calculate the CRC32 checksum of the data and compare it to data
  //@TODO-get this to work
  // uint32_t checksumCalc = CRC32::calculate(data, sizeof(data));
  /* this works
   *uint8_t byteBuffer[] = "Hello World";
   *uint32_t checksum = CRC32::calculate(byteBuffer, sizeof(data));
   */

  uint32_t checksumCalc = 3545618; // good enough for now
  if (checksumEEPROM == checksumCalc)
  {
    return true;
  }
  else
  {
    return false;
  }
}

long readVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

void fadeLED()
{
  delay(100);
  // NOTE-fade value needs to be int or it will get stuck, depending if fadeSpeed%255==0 or not
  byte fadeSpeed = 40;
  byte delayTime = 30;
  // fade in from min to max
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += fadeSpeed)
  {
    analogWrite(LED_BUILTIN, fadeValue); // sets the value (range from 0 to 255):
    delay(delayTime);                    // wait to see the dimming effect
  }
  // fade out from max to min
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= fadeSpeed)
  {
    analogWrite(LED_BUILTIN, fadeValue); // sets the value (range from 0 to 255)
    delay(delayTime);                    // wait to see the dimming effect
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void fadeLED(unsigned int number)
{
  /*
  call this to fade LED a certian number of times
  */

  // pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("fading LED this number of times:"));
  Serial.println(number);

  delay(700);
  for (unsigned int i = 0; i < number; i++)
  {
    fadeLED();
  }
  delay(700);

  /*old code to fade then blink
  //get attention of user by fading first once
  fadeLED();

    if (infoON)
    {
      Serial.print(F("Blinking LED this number of times:"));
      Serial.println(number);
      }
      //actually blink LED
    for (unsigned int i = 0; i < number; i++) // blink
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
    //end attention of user by fading first once
    fadeLED();
    */
}


boolean setupRadio()
{
  /*
  Starts the LoRa radio
  Returns: 1 if successful, 0 if failed
  */

  boolean worked = 1; // start at 1 and set to 0 if errors

  //@TODO-put in function
  // set the nodeID to value in EEPROM if one
  if (!testEEPROMchecksum(nodeIdAddr) || forceChangeNodeID) // if the checksum is invalid or we are being forced to change nodeID, then set node ID to whatever was hard coded when firmware flashed, else read node ID from EEPROM
  {
    if (debug)
    {
      Serial.print(F("no nodeID found in EEPROM (according to checksum or being forced to update nodeID with forceChangeNodeID), setting hard coded one. Its value:"));
      Serial.println(nodeID);
      Serial.print(F("forceChangeNodeID value:"));
      Serial.println(forceChangeNodeID);
    }
    writeDataWithChecksumToEEPROM(nodeIdAddr, nodeID);
  }
  else
  {
    EEPROM.get(nodeIdAddr, nodeID); // sets data to value here (also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("valid nodeID found in EEPROM, set my nodeID to:"));
      Serial.println(nodeID);
    }
  }

  // start RFM95 radio
  if (!rf95.init())
  {
    if (infoON)
    {
      Serial.println(F("RFM95 initialization failed"));
    }
    worked = 0;
  }

  if (worked)
  {
    // Set RFM95 transmission parameters
    // These 5 paramenter settings results in max range. The allowed values also come from this source (source:M. Bor and U. Roedig, “LoRa Transmission Parameter Selection,” in 2017 13th International Conference on Distributed Computing in Sensor Systems (DCOSS), Jun. 2017, pp. 27–34. doi: 10.1109/DCOSS.2017.10.)
    // Note- some set functions do not return anything so don't need to check for sucess.

    //  Set transmit power to the maximum (20 dBm), @TODO-double check docs for 2nd value. false is if PA_BOOST pin is used and true is RFO pin is used on RFM95 module
    // seems like true (and any value) results in weak signal, false (and any value) results in strong signal
    rf95.setTxPower(TXpower, false);
    // rf95.setTxPower(20, true);
    //  Set coding rate to the highest (4/8)
    rf95.setCodingRate4(8); // 5 to 8. offers protection against bursts of interference, A higher CR offers more protection, but increases time on air.
    // Set spreading factor to the highest (SF12)
    rf95.setSpreadingFactor(12); // 6 to 12. A higher spreading factor increases the Signal to Noise Ratio (SNR), and thus sensitivity and range, but also increases the airtime of the packet. The number of chips per symbol is calculated as 2SF . For example, with an SF of 12 (SF12) 4096 chips/symbol are used. Each increase in SF halves the transmission rate and, hence, doubles transmission duration and ultimately energy consumption.
    // Set bandwidth to the lowest (125 kHz). 15600 and 20800 Hz results in fun sounds to be heard on SDR
    rf95.setSignalBandwidth(125000); // in Hz. Allowed range:7.8 to 500 kHz. typical 125,250 or 500 kHz. Higher BW gives a higher data rate (thus shorter time on air), but a lower sensitivity.  Lower BW also requires more accurate crystals (less ppm).
    // Set the desired frequency @TODO-how to set for optimal range?
    if (!rf95.setFrequency(frequency))
    {
      if (infoON)
      {
        Serial.println(F("Failed to set frequency!"));
      }
      worked = 0;
    }
  } // end if(success)

  if (infoON)
  {
    Serial.println(F("RFM95 initialized."));
    if (debug)
    {
      Serial.println(F("All device registers:"));
      rf95.printRegisters();
      Serial.println();
    }
    Serial.print(F("Device version from register 42:"));
    Serial.println(rf95.getDeviceVersion());
    Serial.println(F("nodeID set to:"));
    Serial.println(nodeID);
    //@TODO-consider printing other things like maxMessageLength() (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab273e242758e3cc2ed2679ef795a7196)
  }

  return worked;
}


void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VScode and PlatformIO")); //@TODO-update this whenever compiling or get platformIO to do it automatically
  Serial.println(F("and librarys:mikem/RadioHead, SPI.h"));
  Serial.println(F("RFM9595TEST-SEND V 1.0"));
  Serial.println(F("This program sends dummy packets to test the transmission ability of the REM95 module."));

  /*
   * Version history:
   * V1.0-initial
   * V1.1-added ability to read stuff from EEPROM
   *
   * @TODO:
   * allow frequency set from serial
   * store nodeID in EEPROM
   */

  firmwareV = 1.1;

  // set pin(s) to output mode and do a test
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  digitalWrite(LEDPIN, LOW);

  // start RFM95 radio
  if (!setupRadio())
  {
    if (infoON)
    {
      Serial.println("RFM95 initialization failed");
    }
    digitalWrite(LEDPIN, HIGH);
    fadeLED(99999);
    while (1)
      ;
  }

  if (infoON)
  {
    Serial.println(F("RFM95 initialized."));
    if (debug)
    {
      Serial.println(F("All device registers:"));
      rf95.printRegisters();
      Serial.println();
    }
    Serial.print(F("Device version from register 42:"));
    Serial.println(rf95.getDeviceVersion());
    Serial.println(F("nodeID set to:"));
    Serial.println(nodeID);
    //@TODO-consider printing other things like maxMessageLength() (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab273e242758e3cc2ed2679ef795a7196)
  }

  // turn off the battery wasting stuff if not requested
  if (!infoON)
  {
    Serial.end();
    pinMode(LEDPIN, INPUT);
  }

} // end setup fcn

void loop()
{
  static unsigned long counter = 1;

  long supplyV = readVcc();
  char message[47]; //@TODO-make this shorter to save memory. unsigned long:4,294,967,295. long: -2,147,483,648 (approximate assume all fields are 11 chars and one for comma: 11+1+11+1+11+1+11=47)
  //@TODO-get this to dynamically update the firmware version number
  snprintf(message, sizeof(message) / sizeof(char), "1.0,%lu,%ld,%lu", firmwareV, nodeID, supplyV, counter); // snprintf should add the \0 at the end

  if (infoON)
  {
    Serial.println(F("Sending data:"));
    Serial.println(message);
    //fadeLED(counter); // blink the LED a certain number of times before sending message.
    digitalWrite(LEDPIN, HIGH);//turn on LED to indicate radio is on
  }

  rf95.send((uint8_t *)message, strlen(message));
  rf95.waitPacketSent();
  if (infoON)
  {
    digitalWrite(LEDPIN, LOW);
    Serial.println(F("Packet is either almost done sending (& will finish soon) or is already sent."));
  }

  delay(4000);
  counter++;

} // end loop fcn