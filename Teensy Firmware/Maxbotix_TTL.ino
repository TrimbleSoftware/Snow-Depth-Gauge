/*

 Teensy 3.1 sketch that implements a snow depth gauge for use with XBee wireless communication with meteohub to log
 snow depth data.
 
 Written:  Version 1.0  10-June-2014 Trimble Software, Anchorage, AK
 Modified: Version 1.0a 19-June-2014 Changed error codes to 1000x the values to make them more obvious. Also changed how
                                     serial device readiness is detected to be able to tell if Maxbotix sensor has failed or
                                     is disconnected. Also added XBee reset functionality.
           Version 1.1 25-June-2014 Added watchdog timer per http://forum.pjrc.com/threads/25370-Teensy-3-0-Watchdog-Timer
           Version 1.2 6-Aug-2014   Fixed bug with errors not being retunred on Range command.
           Version 1.3 17-Sep-2014  Added lowpower sleep

 Maxbotix HRXL-Maxsonar MB7354 Teensy 3.1 TTL interface
 
 Hookup:
 
 Teensy 3.1 Pin    Maxbotix MB7354 Pin
 ---------------    ---------------
    GND             7  GND via NPN transistor to have 3.3 VDC pin switch 5.0 VDC supply to Maxbotix sensor via low side switching
    Vin             6  V+
 1  RX1             5  Serial Output TTL
 23 Digital Out     to Base of NPN transistor via 220 ohm 1/4 pullup resister used to switch 5.0 VDC supply of Maxbotix sensor
 
 Teensy 3.1 Pins:
                     220 Ohm
     23 +-----------/\/\--------------+
                                      | B
                                     ___      NPN NTE123AP
                                  E /   \ C
                                   |    |
                                   |    |
    Gnd +--------------------------+    |
                                        |
Maxbotix Pins                           |
    7   +-------------------------------+
    
    6   +---------+ Vin                                   
 
                    Sparkfun XBee Serial explorer WRL-11373
                    ---------------------------------------
    GND             GND
    Vin             5V
 9  RX2             DOUT
 10 TX2             DIN
 
                  1K Ohm
 21 +------------/\/\------------+
                                 | B
                                ___     NPN NTE123AP
                             E /   \ C
                               |   |
                               |   |
 Gnd +-------------------------+   |
                                   |
                                   |
Xbee Pin 5 RESET +-----------------+
 
 */
#define SWVER "1.3"
#define HWVER "1D"
#include <math.h>
#include <EEPROM.h>
#include <LowPower_Teensy3.h> // duff's Teensy 3 low power library https://github.com/duff2013/LowPower_Teensy3
// #define DEBUG 1
#define ABOUT "Trimble Ultrasonic Wireless Snow Depth Guage - Ver"
#define CHAR_CR    0x0d
#define CHAR_R     0x52

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define RANGE_MAX 4999
#define RANGE_MIN 500
#define RANGE_NO_TARGET 5000
#define AUTO_RANGE_DELAY 500

// commands
#define CMD_GET_ABOUT 'A'
#define CMD_RESTART 'B' // restart Teensy CPU
#define CMD_SET_CALIBRATE 'C'
#define CMD_GET_DEPTH 'D'
#define CMD_GET_CALIBRATION 'G'
#define CMD_GET_RANGE 'R'
#define CMD_SET_MANUAL_CALIBRATE 'S'
#define CMD_GET_BATT_VOLTS 'V'

// errors
#define ERR_NONE       0
#define ERR_NO_DATA   -1000
#define ERR_NO_TARGET -2000
#define ERR_TOO_CLOSE -3000
#define ERR_BAD_DATUM -4000
#define ERR_NO_SENSOR -5000 

#define MAXBOTIXPOWERPIN 23 // pin to drive Base of NPN transistor to control power to Maxbotix sensor
#define VOLTAGEPOWERPIN  22 // pin to drive Base of NPN transistor to control power to voltage divider circuit
#define XBEERESETPIN     21 // pin to drive base of NPN transistor to pull low to cause XBee module to reset
#define LED_DIM 128
#define LED_BRIGHT 255
#define BLINK_LONG 500
#define BLINK_SHORT 1
#define ADCPIN A0
// to take cpu out of low power mode to be able to write to EEPROM 
#define TWENTYFOUR_MHZ 24000000

// watchdog
#define RCM_SRS0_WAKEUP                     0x01
#define RCM_SRS0_LVD                        0x02
#define RCM_SRS0_LOC                        0x04
#define RCM_SRS0_LOL                        0x08
#define RCM_SRS0_WDOG                       0x20
#define RCM_SRS0_PIN                        0x40
#define RCM_SRS0_POR                        0x80

#define RCM_SRS1_LOCKUP                     0x02
#define RCM_SRS1_SW                         0x04
#define RCM_SRS1_MDM_AP                     0x08
#define RCM_SRS1_SACKERR                    0x20

// globals
TEENSY3_LP lp = TEENSY3_LP();
HardwareSerial_LP Uart1 = HardwareSerial_LP();
HardwareSerial2_LP Uart2 = HardwareSerial2_LP();
const char ascii_0 = '0';
const char ascii_9 = '9';

const char outputFormat[] = "%c%04.4d\n";
IntervalTimer wdTimer;

int getRange(HardwareSerial_LP port)
{
  char buf[6] = {0,0,0,0,0,0};
  #define READING_COUNT 9
  int sensorReading[READING_COUNT];
  int range = ERR_NO_DATA;
  analogWrite(LED_BUILTIN, LED_DIM);
  digitalWrite(MAXBOTIXPOWERPIN, HIGH); // turn on/boot Maxbotix Sensor
  delay(AUTO_RANGE_DELAY); // delay to allow full auto fange filtering to take place
  for(int i = 0; i < READING_COUNT; i++)
  {
    port.setTimeout(20);
    if(port.readBytes(buf, 1) != 0) // sensor or data available on the TTL UART interface?
    {
      port.clear();
      delay(10);
      while(port.peek() != CHAR_R)
        port.read(); // read until 'R' is found
      port.setTimeout(1000);  
      port.readBytesUntil(CHAR_CR, buf, sizeof(buf)); // read range from Maxbotix sensor
      if
      (
          buf[0] == CHAR_R &&
          (buf[1] >= ascii_0 && buf[1] <= ascii_9) && 
          (buf[2] >= ascii_0 && buf[2] <= ascii_9) &&
          (buf[3] >= ascii_0 && buf[3] <= ascii_9) && 
          (buf[4] >= ascii_0 && buf[4] <= ascii_9)
      )
        sensorReading[i] = (buf[1] - ascii_0) * 1000 + (buf[2] - ascii_0) * 100 + (buf[3] - ascii_0) * 10 + (buf[4] - ascii_0); // convert 4 digit ASCII char number to binary number
      else
        sensorReading[i] = ERR_NO_DATA; // error getting data from sensor
    }
    else
      sensorReading[i] = ERR_NO_SENSOR;
    
  }
  digitalWrite(MAXBOTIXPOWERPIN, LOW); // turn off Maxbotix Sensor
  digitalWrite(LED_BUILTIN, LOW);

  range = mode(sensorReading, READING_COUNT); // get the mode common reading out of mulitple

  if(range == RANGE_NO_TARGET)
    range = ERR_NO_TARGET;
  else
  if(range == RANGE_MIN)
      range = ERR_TOO_CLOSE;

  port.clear();
  port.flush();
  
  return range; 
}

// read sensor to get current height and then store in Teensy EEPROM
int setDatum (HardwareSerial_LP port)
{
  int datum = 0;
  datum = getRange(port);
#ifdef DEBUG
  //
  Serial.print("ranged datum: ");
  Serial.println(datum);
  //
#endif
  setDatum(datum);
}

// store 16 bit sensor mounting height datum into Teensy EEPROM at locations 0 and 1
void setDatum (int datum)
{
  lp.CPU(TWENTYFOUR_MHZ); // exit low power mode to be able to write EEPROM
  EEPROM.write(0, (datum & 0xff00) / 0x100);
  EEPROM.write(1, datum & 0xff);
  lp.CPU(TWO_MHZ); // go back to low power mode after EEPROM write
}

// get 16 bit sensor datum from Teensy EEPROM at locationa 0 and 1
int getDatum()
{
  int datum = 0;
  datum = (EEPROM.read(0) * 0x100) + EEPROM.read(1); 
#ifdef DEBUG
  //
  Serial.print("raw EEPROM bytes: ");
  Serial.print(EEPROM.read(0), HEX);
  Serial.print(" ");
  Serial.println(EEPROM.read(1), HEX);
  Serial.print("datum read from EEPROM: ");
  Serial.println(datum, DEC);
  //
#endif
  return datum;
}

// get Maxbotix sensor boot info
void getSensorInfo (HardwareSerial_LP port, HardwareSerial2_LP port2)
{
  char buf[64];

  digitalWrite(MAXBOTIXPOWERPIN, HIGH); // turn on/boot Maxbotix Sensor  
  delay(100);
  if(port.available())
  {
    for(int i = 0; i < 6; i++) // 6 lines of boot data
    {
      port.readBytesUntil(CHAR_CR, buf, sizeof(buf)); // read boot messages from Maxbotix sensor  
      port2.println(buf);
    }
  }
  digitalWrite(MAXBOTIXPOWERPIN, LOW); // turn off Maxbotix Sensor
}

void blinkLED_BuiltIn(int duration, int brightness)
{
  analogWrite(LED_BUILTIN, brightness); // on
  delay(duration);
  digitalWrite(LED_BUILTIN, LOW); // off
}

// get statistical mode of an array of ints
// adapapted from http://playground.arduino.cc/Main/Average
int mode(int *data, int count)
{
  int pos;
  int inner;
  int most;
  int mostcount;
  int current;
  int currentcount;

  most = 0;
  mostcount = 0;
  for(pos = 0; pos < count; pos++)
  {
    current = data[pos];
    currentcount = 0;
    for(inner = pos + 1; inner < count; inner++)
    {
      if(data[inner] == current)
        currentcount++;
    }
    if(currentcount > mostcount)
    {
      most = current;
      mostcount = currentcount;
    }
    // If we have less array slices left than the current
    // maximum count, then there is no room left to find
    // a bigger count.  We have finished early and we can
    // go home.
    if(count - pos < mostcount)
      break;
  }
  return most;
}

// read ADC to get battery volts via voltage divider to scale 5 VDC to 3.3 VDC
float getVolts ()
{
  /*
  
  Voltage divider circuit for 5VDC USB battery powered Teensy 3.1
  uses high side NPN / PNP switching
  
    R1: 100k 1/4 w
    R2: 10K 1/4 w
    R3: 11K 1/4 w
    R4: 20K 1/4 w
    C1: 0.1uF filter cap
    Q1: NTE123AP NPN transistor
    Q2: NTE159 PNP Trnasistor
    
           R1
    23 +--/\/\----------------------+
                                    | B
                                    __    Q1
                                 E /  \ C
                                   |  |     R2
                                   |  +---/\/\/-------+
                                   |                  | B
                                   + Gnd              __     Q2
                                                   C /  \ E
                                                     |  |
                                                     /  +---------------+ to Vin 5.1 VDC
                                                R3   \
                                                     /
    A0 +---------------------------------------------+-----+
          Vout 3.25 VDC                              /     |
                                                R4   \     _ 
                                                     /     _  C1
                                                     \     |
   AGND +--------------------------------------------+     |
                                                           +---+ Gnd
  */
  const int adcRes = 10;  // 10 bit adc
  int adcValue = 0;
  const float vIn = 4.95;
  const float vOut = 3.20;
  const float vRef = 3.26;
  const int adcSteps = pow(2, adcRes);
  
  lp.CPU(TWENTYFOUR_MHZ); // exit low power mode to be able to read ADC
  
  digitalWrite(VOLTAGEPOWERPIN, HIGH); // turn on voltage divider circuit  
  analogReference(DEFAULT);
  analogReadResolution(adcRes);
  analogReadAveraging(32);
  delay(10); // allow filter cap to charge
  adcValue = analogRead(ADCPIN);
  //delay(60000);
  digitalWrite(VOLTAGEPOWERPIN, LOW); // turn off voltage divider circuit  
  
  lp.CPU(TWO_MHZ); // go back to low power mode after ADC read
#ifdef DEBUG
  Uart2.printf("adcValue: %d\n", adcValue);
#endif  
  return (vRef / adcSteps) * (vIn / vOut) * adcValue;
}

void ResetXbee(int restPin)
{
  digitalWrite(restPin, HIGH);
  
#ifdef DEBUG  
  delay(60000);
#else
  delay(250); // have to hold Xbee pin 5 low for atleast 200ms
#endif
  digitalWrite(restPin, LOW);  
}

void PrintAbout(HardwareSerial2_LP port)
{
  char buf[64] = {0};
  strcat(buf, ABOUT);
  strcat(buf," ");
  strcat(buf, HWVER);
  strcat(buf, "-");
  strcat(buf, SWVER);
  port.println(buf);
}

void KickDog()
{
#ifdef DEBUG
  Serial.println("Kicking the dog!");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif  
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

// setup Teensy 3.1 operating params
void setup()
{
#ifdef DEBUG
  lp.CPU(TWENTYFOUR_MHZ);
#else
  lp.CPU(TWO_MHZ);
#endif
  Uart2.begin(38400); // XBee preset to 38400 baud and using transparent mode
  Uart1.begin(9600);  // Maxbotix TTL serial baud rate
  pinMode(MAXBOTIXPOWERPIN, OUTPUT);
  pinMode(VOLTAGEPOWERPIN, OUTPUT);
  pinMode(XBEERESETPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(MAXBOTIXPOWERPIN, LOW);

#ifdef DEBUG
  delay(20000);
  printResetType();
#else
  wdTimer.begin(KickDog, 500000); // kick the dog every 500msec
#endif  
  PrintAbout(Uart2);
  ResetXbee(XBEERESETPIN);
  blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
}

// main processing loop used to parse commands from host
void loop()
{
#ifdef DEBUG
  Serial.println("Sleeping...");
#endif
  lp.Sleep();
#ifdef DEBUG
  Serial.println("Awakened");
#endif
  processCommand();
}


void processCommand(void)
{
  int datum = 0;
  int range = 0;
  int depth = 0;
  int volts = 0; // battery volts * 100

  char commandByte = 0;
  
  if(Uart2.available())
  {
    commandByte = Uart2.read();
    switch(commandByte)
    {
      case CMD_GET_ABOUT: // about
      {
        PrintAbout(Uart2);
        getSensorInfo(Uart1, Uart2);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }
        
      case CMD_RESTART: // restart CPU
      {
        Uart2.clear();
        Uart2.println("CPU Resetting...");
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
        Uart2.clear();
        Uart2.println("XBee Resetting...");
        Uart2.flush();
        //ResetXbee(XBEERESETPIN); // Reset XBee
        CPU_RESTART
        break;
      }

      case CMD_SET_CALIBRATE: // calibrate mounting height and store into EEPROM, send results out serial
      {
        setDatum(Uart1);
        datum = getDatum();
        Uart2.printf(outputFormat, CMD_SET_CALIBRATE, datum);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }

      case CMD_GET_DEPTH: // read snow depth and send out serial
      {
        datum = getDatum();
        range = getRange(Uart1);
        // Ver 1.2 bug fix begin
        if(range < 0)
          depth = range;
        else
        // Ver 1.2 bug fix end
        if(range < datum)
          depth = datum - range;
        else
          depth = 0;
          
        Uart2.printf(outputFormat, CMD_GET_DEPTH, depth);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }

      case CMD_GET_CALIBRATION: // get saved datum saved in EEPROM and send out serial
      {
        datum = getDatum();
        Uart2.printf(outputFormat, CMD_GET_CALIBRATION, datum);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }

      case CMD_GET_RANGE: // read sensor range and send out serial
      {
        range = getRange(Uart1);
        Uart2.printf(outputFormat, CMD_GET_RANGE, range);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }

      case CMD_SET_MANUAL_CALIBRATE : // set manual datum and save into EEPROM, reread and send out serial
      {
        char buf[5] = {0,0,0,0,0};
        Uart2.readBytesUntil(CHAR_CR, buf, sizeof(buf));
        if
        (
          (buf[0] >= ascii_0 && buf[0] <= ascii_9) && 
          (buf[1] >= ascii_0 && buf[1] <= ascii_9) &&
          (buf[2] >= ascii_0 && buf[2] <= ascii_9) && 
          (buf[3] >= ascii_0 && buf[3] <= ascii_9)
        )
        {
          datum = (buf[0] - ascii_0) * 1000 + (buf[1] - ascii_0) * 100 + (buf[2] - ascii_0) * 10 + (buf[3] - ascii_0);
          setDatum(datum);
          datum = getDatum();
        }
        else
          datum = ERR_BAD_DATUM;

        Uart2.printf(outputFormat, CMD_SET_MANUAL_CALIBRATE, datum);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }
        
      case CMD_GET_BATT_VOLTS: // get battery voltage and send out serial
      {
#ifdef DEBUG          
        float v = 0.0;
        v = getVolts();
        Uart2.println(v, DEC);
#endif   
        volts = (int)((getVolts() * 100.0) + 0.5);
        Uart2.printf(outputFormat, CMD_GET_BATT_VOLTS, volts);
        Uart2.flush();
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }
    }
  }
}

void printResetType() 
{
  if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("[RCM_SRS1] - Stop Mode Acknowledge Error Reset");
  if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("[RCM_SRS1] - MDM-AP Reset");
  if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("[RCM_SRS1] - Software Reset");
  if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("[RCM_SRS1] - Core Lockup Event Reset");
  if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("[RCM_SRS0] - Power-on Reset");
  if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("[RCM_SRS0] - External Pin Reset");
  if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("[RCM_SRS0] - Watchdog(COP) Reset");
  if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("[RCM_SRS0] - Loss of External Clock Reset");
  if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("[RCM_SRS0] - Loss of Lock in PLL Reset");
  if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("[RCM_SRS0] - Low-voltage Detect Reset");
}

#ifdef __cplusplus
extern "C" {
#endif
  void startup_early_hook() {
    WDOG_TOVALL = 3000; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
    WDOG_TOVALH = 0;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    //WDOG_PRESC = 0; // prescaler 
  }
#ifdef __cplusplus
}
#endif
