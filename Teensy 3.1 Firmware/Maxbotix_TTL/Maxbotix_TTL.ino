/*

 Teensy 3.1 sketch that implements a snow depth gauge for use with XBee wireless communication with meteohub to log
 snow depth data using a Maxbotix ultrasonic range finder snow depth sensor.
 
 Written:  Version 1.0  10-June-2014 Trimble Software, Anchorage, AK
 Modified: Version 1.0a 19-June-2014 Changed error codes to 1000x the values to make them more obvious. Also changed how
                                       serial device readiness is detected to be able to tell if Maxbotix sensor has failed or
                                       is disconnected. Also added XBee reset functionality.
           Version 1.1  25-June-2014 Added watchdog timer per http://forum.pjrc.com/threads/25370-Teensy-3-0-Watchdog-Timer
           Version 1.2  06-Aug-2014  Fixed bug with errors not being retunred on Range command.
           Version 1.3  17-Sep-2014  Added lowpower sleep.
           Version 1.4  01-Oct-2014  Cleaned up some code having to do with XBee reset.
           Version 1.5  04-Sep-2015  Code changes for hardware ver 2B integration with Adafriut solar charger http://www.adafruit.com/products/390
                                       and xBee hardware wake/sleep on pin 17. http://www.fiz-ix.com/2012/11/low-power-xbee-sleep-mode-with-arduino-and-pin-hibernation/

 Maxbotix HRXL-Maxsonar MB7354 Teensy 3.1 TTL interface
 
 Hookup:
 
 Teensy 3.1 Pin    Maxbotix MB7354 Pin
 ---------------   ---------------
    GND             7  GND via NPN transistor to have 3.3 VDC pin switch 5.0 VDC supply to Maxbotix sensor via low side switching
    Vin             6  V+
 0  RX1             5  Serial Output TTL
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
 
                    XBee pin
                    --------
    GND             10 GND
    3.3V            1  VDD
 9  RX2             2  DOUT
 10 TX2             3  DIN
 16                 13 DIO9 To signal when XBee is awake
 17                 9  DTR  To use Pin Mode sleep/wake on XBee
 
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
XBee Pin 5 RESET +-----------------+

                   Adafruit solar charger
                   ---------------
 2                  D
 3                  C
 
*/

#include <math.h>
#include <EEPROM.h>
#include <LowPower_Teensy3.h> // duff's Teensy 3 low power library https://github.com/duff2013/LowPower_Teensy3

//#define DEBUG 1
#define SWVER "1.5"
#define HWVER "2B"

#define ABOUT "Trimble Ultrasonic Wireless Snow Depth Gauge - Ver"
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
#define CMD_GET_CHARGE_STATUS 'T' // get LiPo battery charger status
#define CMD_GET_BATT_VOLTS 'V'
#define CMD_HELP '?'

// errors
#define ERR_NONE       0
#define ERR_NO_DATA   -1000
#define ERR_NO_TARGET -2000
#define ERR_TOO_CLOSE -3000
#define ERR_BAD_DATUM -4000
#define ERR_NO_SENSOR -5000 

// Teensy I/O pins
#define MAXBOTIXPOWERPIN   23 // pin to drive Base of NPN transistor to control power to Maxbotix sensor
#define VOLTAGEPOWERPIN    22 // pin to drive Base of NPN transistor to control power to voltage divider circuit
#define XBEERESETPIN       21 // pin to drive base of NPN transistor to pull low to cause XBee module to reset
#define XBEESLEEPPIN       17 // pin to control xBee wake/sleep via pin 9 on the xBee
#define XBEEAWAKEPIN       16 // pin to detect if XBee is awake
#define DONE_CHARGING_PIN  2  // pin to test if Adafruit solar charger is done charging
#define STILL_CHARGING_PIN 3  // pin to test if Adafruit solar charger is still charging
#define ADCPIN      A0

#define LED_DIM     128
#define LED_BRIGHT  255
#define BLINK_LONG  500
#define BLINK_SHORT 1
#define ADCSAMPLES  10 // number of adc samples to take and average for reading battery voltage

// status codes for Adafruit solar charger
#define STATUS_DONE_CHARGING 2
#define STATUS_CHARGING      1
#define STATUS_NOT_CHARGING  0

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
HardwareSerial_LP Uart1 = HardwareSerial_LP(); // RX1 for Maxbotix sensor
HardwareSerial2_LP Uart2 = HardwareSerial2_LP(); // RX2/TX2 for XBee
const char ascii_0 = '0';
const char ascii_9 = '9';

const char outputFormat[] = "%c%04.4d\n";
IntervalTimer wdTimer;


void XBeeSleep(int SleepPin)
{
  // put the xBee to sleep
#ifdef DEBUG
  Serial.printf("Putting XBee to sleep...\n");
#endif
  pinMode(SleepPin, INPUT); // put pin in a high impedence state
  digitalWrite(SleepPin, HIGH);  
}

void XBeeWake(int WakePin)
{
  // wake up the XBee
#ifdef DEBUG
  Serial.printf("Waking XBee...\n");
#endif
  pinMode(WakePin, OUTPUT);
  digitalWrite(WakePin, LOW);  
}

boolean isXBeeAwake(int AwakePin)
{
  // is XBee awake?  
  pinMode(AwakePin, INPUT_PULLUP);
  int isAwake = digitalRead(AwakePin);
#ifdef DEBUG
  Serial.printf("XBee isAwake: %d\n", isAwake);
#endif
  return (isAwake > 0? true: false);
}

int XBeePrintf(HardwareSerial2_LP port, boolean flickerLED, const char *format, ...)
{
  char buf[256];
  va_list args;
  int done;
 
  if(!isXBeeAwake(XBEEAWAKEPIN)) // if XBee is asleep,
  {
    XBeeWake(XBEESLEEPPIN); // wake it up
    delay(10); // delay to allow XBee time to wakeup
  }
 
  va_start (args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  port.clear();
#ifdef DEBUG
  Serial.print(buf);
#endif
  done = port.print(buf);
  port.flush();

  if(flickerLED)
    blinkLED_BuiltIn(BLINK_SHORT, LED_DIM); // flicker the built in Teensy LED
    
  XBeeSleep(XBEESLEEPPIN); // put XBee back to sleep
  
  return done;
}

int getRange(HardwareSerial_LP port)
{
  char buf[6] = {0,0,0,0,0,0};
  #define READING_COUNT 9
  int sensorReading[READING_COUNT];
  int range = ERR_NO_DATA;
#ifdef DEBUG  
  analogWrite(LED_BUILTIN, LED_DIM);
#endif  
  digitalWrite(MAXBOTIXPOWERPIN, HIGH); // turn on/boot Maxbotix Sensor
  delay(AUTO_RANGE_DELAY); // delay to allow full auto-range filtering to take place
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
 #ifdef DEBUG 
  digitalWrite(LED_BUILTIN, LOW);
#endif

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
  Serial.printf("ranged datum: %d", datum);
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
  Serial.print("raw EEPROM bytes: ");
  Serial.print(EEPROM.read(0), HEX);
  Serial.print(" ");
  Serial.println(EEPROM.read(1), HEX);
  Serial.print("datum read from EEPROM: ");
  Serial.println(datum, DEC);
#endif
  return datum;
}

// get Maxbotix sensor boot info
void getSensorInfo (HardwareSerial_LP port1, HardwareSerial2_LP port2)
{
  char buf[64];

  digitalWrite(MAXBOTIXPOWERPIN, HIGH); // turn on/boot Maxbotix Sensor  
  delay(100);
  if(port1.available())
  {
    for(int i = 0; i < 6; i++) // 6 lines of boot data
    {
      port1.readBytesUntil(CHAR_CR, buf, sizeof(buf)); // read boot messages from Maxbotix sensor  
      XBeePrintf(port2, false, "%s\n", buf);
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

// read ADC to get battery volts via voltage divider to scale 4.2 VDC to 3.3 VDC range needed by Teensy ADC
float getVolts ()
{
  /*
  
  Voltage divider circuit for 5VDC USB battery powered Teensy 3.1
  uses high side NPN / PNP switching
  
    R1: 100k 1/4 w
    R2: 10K 1/4 w
    R3: 10K 1/4 w
    R4: 27K 1/4 w 1%
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
                                                     /  +---------------+ to Vin 4.17 VDC Adafruit solar charger B+ pin
                                                R3   \
                                                     /
    A0 +---------------------------------------------+-----+
          Vout 3.06 VDC                              /     |
                                                R4   \     _ 
                                                     /     _  C1
                                                     \     |
   AGND +--------------------------------------------+     |
                                                           +---+ Gnd
  */
  const int adcRes = 10;  // 10 bit adc
  int adcValue = 0;
  const float vIn = 4.17;
  const float vOut = 3.06;
  const float vRef = 3.3;
  const int adcSteps = pow(2, adcRes);
  
  lp.CPU(TWENTYFOUR_MHZ); // exit low power mode to be able to read ADC
  
  digitalWrite(VOLTAGEPOWERPIN, HIGH); // turn on voltage divider circuit  
  analogReference(DEFAULT);
  analogReadResolution(adcRes);
  analogReadAveraging(32);
  delay(10); // allow filter cap to charge
  
  for(int i = 0; i < ADCSAMPLES; i++)
    adcValue += analogRead(ADCPIN);
    
  adcValue = adcValue / ADCSAMPLES;
  //delay(60000); // uncomment to get suficciant time to manually measure voltage on A0 pin
  digitalWrite(VOLTAGEPOWERPIN, LOW); // turn off voltage divider circuit  
  
  lp.CPU(TWO_MHZ); // go back to low power mode after ADC read
#ifdef DEBUG
  XBeePrintf(Uart2, true, "adcValue: %d\n", adcValue);
#endif  
  return (vRef / adcSteps) * (vIn / vOut) * adcValue;
}

void ResetXBee(int resetPin)
{
  digitalWrite(resetPin, HIGH);
  
#ifdef DEBUG  
  delay(60000);
#else
  delay(250); // have to hold XBee pin 5 low for atleast 200ms
#endif
  digitalWrite(resetPin, LOW);  
}

void PrintAbout(HardwareSerial2_LP port)
{
  char buf[64] = {0};
  strcat(buf, ABOUT);
  strcat(buf," ");
  strcat(buf, HWVER);
  strcat(buf, "-");
  strcat(buf, SWVER);
  delay(50);
  XBeePrintf(port, false, "%s\n", buf);
  delay(50);
}

void PrintCommands(HardwareSerial2_LP port)
{
  delay(50);
  XBeePrintf(port, true, "%s\n", "Available Commands:");
  XBeePrintf(port, false, "  %s\n", "A - Get version information");
  XBeePrintf(port, false, "  %s\n", "B - Reboot Teensy CPU and XBee Radio");
  XBeePrintf(port, false, "  %s\n", "C - Calibrate snow depth sensor at current distance");
  XBeePrintf(port, false, "  %s\n", "D - Get calibrated snow depth (mm)");
  XBeePrintf(port, false, "  %s\n", "G - Get saved calibration value (mm)");
  XBeePrintf(port, false, "  %s\n", "R - Get snow depth sensor range (mm)");
  XBeePrintf(port, false, "  %s\n", "Sxxxx - Set manual calibration distance xxxx (mm)");
  XBeePrintf(port, false, "  %s\n", "T - Get battery charger status:");
  XBeePrintf(port, false, "    %s\n", "2: Done Charging, 1: Charging, 0: Not Charging");
  XBeePrintf(port, false, "  %s\n", "V - Get battery voltage (100x)");
  XBeePrintf(port, false, "  %s\n", "? - List available commands");
  delay(50);
}

void KickDog()
{
#ifdef DEBUG
  Serial.printf("Kicking the dog!\n");
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
  pinMode(STILL_CHARGING_PIN, INPUT_PULLUP);
  pinMode(DONE_CHARGING_PIN, INPUT_PULLUP);
  pinMode(MAXBOTIXPOWERPIN, OUTPUT);
  pinMode(VOLTAGEPOWERPIN, OUTPUT);
  pinMode(XBEERESETPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(MAXBOTIXPOWERPIN, LOW);
  digitalWrite(XBEERESETPIN, LOW);
  digitalWrite(VOLTAGEPOWERPIN, LOW);
  XBeeWake(XBEESLEEPPIN); // make sure xBee is set to be awake when using periodic polling mode on the xBee
  
#ifdef DEBUG
  delay(20000);
  printResetType();
#endif
  wdTimer.begin(KickDog, 500000); // kick the dog every 500msec

  Uart2.flush();
  Uart2.clear();
  PrintAbout(Uart2);
  blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
}

// main processing loop used to parse commands from host
void loop()
{ 
#ifdef DEBUG
  Serial.printf("Sleeping...\n");
#endif
  // put Teensy to sleep, interrupt on UART input from XBee will wake it up
  lp.Sleep();
#ifdef DEBUG
  Serial.printf("Awakened\n");
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
        blinkLED_BuiltIn(BLINK_SHORT, LED_DIM);
        break;
      }
        
      case CMD_RESTART: // restart CPU
      {
#ifdef DEBUG
        XBeePrintf(Uart2, false, "%s\n", "CPU Resetting...");
#endif         
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
        blinkLED_BuiltIn(BLINK_LONG, LED_BRIGHT);
        delay(50);
#ifdef DEBUG        
        XBeePrintf(Uart2, false, "%s\n", "XBee Resetting...");
        delay(50);
#endif        
        ResetXBee(XBEERESETPIN); // Reset XBee
        delay(50);
        CPU_RESTART;
        break;
      }

      case CMD_SET_CALIBRATE: // calibrate mounting height and store into EEPROM, send results out serial
      {
        setDatum(Uart1);
        datum = getDatum();
        XBeePrintf(Uart2, true, outputFormat, CMD_SET_CALIBRATE, datum);
        break;
      }

      case CMD_GET_DEPTH: // read snow depth and send out serial
      {
        datum = getDatum();
        range = getRange(Uart1);
        if(range < 0)
          depth = range;
        else
        if(range < datum)
          depth = datum - range;
        else
          depth = 0;
          
        XBeePrintf(Uart2, true, outputFormat, CMD_GET_DEPTH, depth);
        break;
      }

      case CMD_GET_CALIBRATION: // get saved datum saved in EEPROM and send out serial
      {
        datum = getDatum();
        XBeePrintf(Uart2, true, outputFormat, CMD_GET_CALIBRATION, datum);
        break;
      }

      case CMD_GET_RANGE: // read sensor range and send out serial
      {
        range = getRange(Uart1);
        XBeePrintf(Uart2, true, outputFormat, CMD_GET_RANGE, range);
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

        XBeePrintf(Uart2, true, outputFormat, CMD_SET_MANUAL_CALIBRATE, datum);
        break;
      }
      
      case CMD_GET_CHARGE_STATUS: // get Adafruit solar charger status
      {
        int chargeStatus = -1;
        int stillCharging = digitalRead(STILL_CHARGING_PIN);
        int doneCharging = digitalRead(DONE_CHARGING_PIN);

        switch (doneCharging << 1 | stillCharging)
        {
          case 1: // done charging
            chargeStatus = STATUS_DONE_CHARGING;
            break;
          case 2: // charging
            chargeStatus = STATUS_CHARGING;
            break;
          case 3: // not charging
            chargeStatus = STATUS_NOT_CHARGING;
            break;
          default: // shouldn't be possible
            chargeStatus = -1;
            break;
        }
        
        XBeePrintf(Uart2, true, outputFormat, CMD_GET_CHARGE_STATUS, chargeStatus);
        break;
      }
      case CMD_GET_BATT_VOLTS: // get battery voltage and send out serial
      {
#ifdef DEBUG          
        float v = 0.0;
        v = getVolts();
        XBeePrintf(Uart2, false, "%d\n", v);
#endif   
        volts = (int)((getVolts() * 100.0) + 0.5);;
        XBeePrintf(Uart2, true, outputFormat, CMD_GET_BATT_VOLTS, volts);
        break;
      }
      case CMD_HELP: // list commands
      {
        PrintCommands(Uart2);        
      }
#ifdef DEBUG
      default:
      {
        Serial.printf("\nUnknown commandByte: 0x%0x", commandByte);
      }
#endif
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
    WDOG_TOVALL = 3000; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compares itself to.
    WDOG_TOVALH = 0;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    //WDOG_PRESC = 0; // prescaler 
  }
#ifdef __cplusplus
}
#endif

