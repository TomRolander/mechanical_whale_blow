/**************************************************************************
  Mechanical Whale Blow

  Original Code:  2020-03-05

  Tom Rolander, MSEE
  Mentor, Circuit Design & Software
  Miller Library, Fabrication Lab
  Hopkins Marine Station, Stanford University,
  120 Ocean View Blvd, Pacific Grove, CA 93950
  +1 831.915.9526 | rolander@stanford.edu

 **************************************************************************/

#define VERSION "Ver 0.2"
#define MODIFIED "2020-03-06"

#include <SPI.h>
#include <SD.h>

#include <Adafruit_MAX31865.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(7, 11, 12, 13);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

#include <DHT.h>;
//Constants
#define DHT1PIN 5     // what pin we're connected to
#define DHT2PIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht1(DHT1PIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
DHT dht2(DHT2PIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

float T1 = 0.0;
float H1 = 0.0;
float T2 = 0.0;
float H2 = 0.0;
float TS = 0.0;
float PR = 0.0;

float LO_TEMP = 36.0;
float HI_TEMP = 38.0;

bool bSDLogFail = false;
int  iToggle = 0;

File fileSDCard;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int chipSelectSDCard = 10;

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"

RTC_PCF8523 rtc;
DateTime now;

#include <LiquidCrystal.h>

const int rs = 8, en = 9, d4 = A3, d5 = A2, d6 = A1, d7 = A0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();

  max.begin(MAX31865_3WIRE);

  dht1.begin();
  dht2.begin();
  
  lcd.begin(16, 2);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Mech. Whale Blow"));
  lcd.print(F(VERSION));
  lcd.setCursor(0, 1);
  lcd.print(F("TAR "));
  lcd.print(F(MODIFIED));
  delay(2000);

  // Initialize the Real Time Clock
  if (! rtc.begin())
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("*** ERROR ***   "));
    lcd.setCursor(0, 1);
    lcd.print(F("Couldnt find RTC"));
    while (1);
  }
  if (! rtc.initialized())
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("*** WARN ***    "));
    lcd.setCursor(0, 1);
    lcd.print(F("RTC isnt running"));

    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("*** DATE ***    "));
  lcd.setCursor(0, 1);
  lcd.print(now.year(), DEC);
  lcd.print(F("/"));
  LCDPrintTwoDigits(now.month());
  lcd.print(F("/"));
  LCDPrintTwoDigits(now.day());
  lcd.print(F(" "));
  LCDPrintTwoDigits(now.hour());
  lcd.print(F(":"));
  LCDPrintTwoDigits(now.minute());
  delay(2000);

//  SetupSDCardOperations();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("H1_ T1_ H2_ T2_ "));
  lcd.setCursor(0,1);
  lcd.print(F("TS_ PR_ HH MM SS"));  
  delay(4000); 
}

void loop()
{
  LCDDigitalOutputUpdate();
  delay(1000);
}

void LCDPrintTwoDigits(int iVal)
{
  if (iVal < 10)
    lcd.print(F("0"));
  lcd.print(iVal, DEC);
}

void LCDPrintThreeDigits(float fVal)
{
  if (fVal < 100.0)
    lcd.print(F("0"));
  if (fVal < 10.0)
    lcd.print(F("0"));
  int iVal = (int) fVal;
  lcd.print(iVal, DEC);
}

void SetupSDCardOperations()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("*** STATUS ***  "));
  lcd.setCursor(0, 1);
  lcd.print(F("SD Init Start   "));

  pinMode(10, OUTPUT);
  if (!SD.begin(chipSelectSDCard)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("*** ERROR ***   "));
    lcd.setCursor(0, 1);
    lcd.print(F("SD Init Failed  "));
    while (1);
  }

  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("* Test clock.csv"));
  lcd.setCursor(0, 1);
  if (SD.exists("clock.csv")) 
  {
    lcd.print(F("  Set Clock     "));
    delay(2000);

    fileSDCard = SD.open("clock.csv");
    if (fileSDCard) 
    {
      if (fileSDCard.available())
      {
        char strClockSetting[128];
        fileSDCard.read(strClockSetting, sizeof(strClockSetting));
        strClockSetting[sizeof(strClockSetting)] = '\0';
        int iDateTime[6] = {0,0,0,0,0,0};
        char *ptr1 = &strClockSetting[0];      
        for (int i=0; i<6; i++)
        {
          char *ptr2 = strchr(ptr1,',');
          if (ptr2 != 0)
          {
            *ptr2 = '\0';
            iDateTime[i] = atoi(ptr1);
            ptr1 = &ptr2[1];
          }
          else
          {
            break;
          }
        }
          
        rtc.adjust(DateTime(iDateTime[0],iDateTime[1],iDateTime[2],iDateTime[3],iDateTime[4],iDateTime[5]));
        SD.remove("clock.csv");

        lcd.setCursor(0, 1);
        lcd.print(F("* removed *     "));
        delay(2000);
      
      }
      fileSDCard.close();
    } 
    
  } else 
  {
    lcd.print(F("* does not exist"));
  }
  delay(2000);

// open the file for reading:
  fileSDCard = SD.open("LOGGING.CSV");
  if (fileSDCard) 
  {
    if (fileSDCard.available())
    {
    }
    fileSDCard.close();
  } 
  else
  {
    fileSDCard = SD.open("LOGGING.CSV", FILE_WRITE);
    if (fileSDCard) 
    {
      fileSDCard.println(F("\"Date\",\"Time\",\"OP\",\"T1\",\"H1\",\"T2\",\"H2\",\"TS\",\"P\""));
      // OP:
      // Startup
      // HeatON
      // HeatOFF
      fileSDCard.close();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("*** CREATE ***  "));
      lcd.setCursor(0, 1);
      lcd.print(F("SD LOGGING.CSV  "));
      delay(2000);
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("*** ERROR ***   "));
      lcd.setCursor(0, 1);
      lcd.print(F("SD Write Failed "));
      while (1);
    }
  }
  SD.end();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("*** STATUS ***  "));
  lcd.setCursor(0, 1);
  lcd.print(F("SD Init Finish  "));
  delay(2000);
  lcd.clear();
  
  LCDStatusUpdate_SDLogging(F("StartUp ")); 
}


void LCDDigitalOutputUpdate()
{
  H1 = dht1.readHumidity();
  T1 = dht1.readTemperature();
  H2 = dht2.readHumidity();
  T2 = dht2.readTemperature();
  
  TS = GetWaterTempSensor();

  DateTime now = rtc.now();      
  lcd.setCursor(8,1);
  LCDPrintTwoDigits(now.hour());
  lcd.print(F(":"));
  LCDPrintTwoDigits(now.minute());   
  lcd.print(F(":"));
  LCDPrintTwoDigits(now.second());   

  // "H1_ T1_ H2_ T2_ "
  // "TS_ PR_ HH MM SS"

  lcd.setCursor(0, 0);
  LCDPrintThreeDigits(H1);
  lcd.print(F(" "));  
  LCDPrintThreeDigits(T1);
  lcd.print(F(" "));  
  LCDPrintThreeDigits(H2);
  lcd.print(F(" "));  
  LCDPrintThreeDigits(T2);
  lcd.print(F(" "));  
  lcd.setCursor(0, 1);
  LCDPrintThreeDigits(TS);
  lcd.print(F(" "));  
  LCDPrintThreeDigits(PR);
  lcd.print(F(" ")); 
}

void LCDStatusUpdate_SDLogging(const __FlashStringHelper*status)
{
  lcd.setCursor(0, 1);
  lcd.print(status);

  DateTime now = rtc.now();      
  lcd.setCursor(8,1);
  LCDPrintTwoDigits(now.hour());
  lcd.print(F(" "));
  LCDPrintTwoDigits(now.minute());   
  lcd.print(F(" "));
  LCDPrintTwoDigits(now.second());   

  if (!SD.begin(chipSelectSDCard)) 
  {
    bSDLogFail = true;
    iToggle++;
    if ((iToggle & B00000001) == 0)
    {
      lcd.setCursor(0, 1);
      lcd.print(F("SD LogFail"));
    }
    return;
  }
  bSDLogFail = false;
  iToggle = 0;

//  if ((strcmp((const char*) status, "") != 0) || bForceOneMinuteLogging)
  {
    fileSDCard = SD.open("LOGGING.CSV", FILE_WRITE);
  
    // if the file opened okay, write to it:
    if (fileSDCard) 
    {
      fileSDCard.print(now.year(), DEC);
      fileSDCard.print("/");
      fileSDCard.print(now.month(), DEC);
      fileSDCard.print("/");
      fileSDCard.print(now.day(), DEC);
      fileSDCard.print(",");
      fileSDCard.print(now.hour(), DEC);
      fileSDCard.print(":");
      fileSDCard.print(now.minute(), DEC);
      fileSDCard.print(":");
      fileSDCard.print(now.second(), DEC);
      fileSDCard.print(",");
      //SDPrintBinary(digitalOutputState,5);
      fileSDCard.print(",");
      //SDPrintBinary(digitalInputState_Saved,4);
      fileSDCard.print(",");
      fileSDCard.print(status);
      fileSDCard.println("");
      fileSDCard.close();
      SD.end();
    } 
    else 
    {
      // if the file didn't open, display an error:
      lcd.setCursor(0, 0);
      lcd.print(F("*** ERROR ***   "));
      lcd.setCursor(0, 1);
      lcd.print(F("Open LOGGING.CSV"));
    }  
  }
}

float GetWaterTempSensor()
{
  float celsius = 0.0;
  // Call watertempsensor.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
//  watertempsensor.requestTemperatures(); 
//  celsius = watertempsensor.getTempCByIndex(0);

  uint16_t rtd = max.readRTD();
  double temp = max.temperature(RNOMINAL, RREF);
  uint8_t fault = false;

//  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
//  Serial.print("Ratio = "); Serial.println(ratio,8);
//  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
//  Serial.print("Temperature = "); Serial.println(max.temperature(RNOMINAL, RREF));
  

  fault = max.readFault();
  if (fault) 
  {
    Serial.print(" Fault 0x"); Serial.print(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println(" RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println(" RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println(" REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println(" REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println(" RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println(" Under/Over voltage"); 
    }
    max.clearFault();
  }

  return (temp);
}
