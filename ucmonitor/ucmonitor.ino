#include <SPI.h>

SPIClass hspi(HSPI);
static const int spiClk = 1000000; // 1 MHz

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

const int pin_DRDYBv = 25;  // data ready
const int pin_ALARMBv = 33; // alarm
const int pin_SSv = 15;    // CSB
const int pin_DRDYBh = 27;  // data ready
const int pin_ALARMBh = 26; // alarm
const int pin_SSh = 5;    // CSB
const int pin_MISO = 19;  // MISO
const int pin_MOSI = 23;  // MOSI
const int pin_SCLK = 18;  // SCLK

int32_t c1v;
int32_t c2v;
int32_t c1h;
int32_t c2h;

int32_t err1;
int32_t err2;
int32_t err3;
int32_t err4;
int32_t err5;
int32_t err6;
int32_t err7;

double accx;
double accy;
double accz;
double gyx;
double gyy;
double gyz;
double magx;
double magy;
double magz;
double temp;

bool checkVH=0; //0=v 1=h


#include <WiFi.h>
#include <FirebaseESP32.h>

#define WIFI_SSID "DESKTOP-E15OF8E 7705"
#define WIFI_PASSWORD "12345678"

#define FIREBASE_HOST "https://esp32test-3b776-default-rtdb.firebaseio.com/"

/** The database secret is obsoleted, please use other authentication methods, 
 * see examples in the Authentications folder. 
*/
#define FIREBASE_AUTH "8c9tGrSxBECAGDFgeCNIjMQyKfCl397DXuKSHSLY"
//#define NAME "/user"

//Define Firebase Data object
FirebaseData fbdo;

float BatteryVoltage;

float ReadBatteryVoltage(){
  return analogRead(35)/4096.0*7.445;
}

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount[]={};

RTC_DATA_ATTR int i=0;

//millis
unsigned long previous = 0;
unsigned long present;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

int32_t getValFromChannel(int channel)
{
  byte x1;
  byte x2;
  byte x3;

  switch (channel)
  {
  case 1:
    x1 = 0x37;
    x2 = 0x38;
    x3 = 0x39;
    break;
  case 2:
    x1 = 0x3A;
    x2 = 0x3B;
    x3 = 0x3C;
    break;
  case 3:
    x1 = 0x3D;
    x2 = 0x3E;
    x3 = 0x3F;
    break;
  }
  int32_t val;
//  Serial.print("Channel ");
//  Serial.println(channel);
//  Serial.print("X ");
//  Serial.print(x1);
//  Serial.print(" ");
//  Serial.print(x2);
//  Serial.print(" ");
//  Serial.print(x3);

  // 3 8-bit registers combination on a 24 bit number
  val = readRegister(x1);
  val = (val << 8) | readRegister(x2);
  val = (val << 8) | readRegister(x3);

  return val;
}


void setup_ECG_2_channel()
{
  // datasheet ads1293
  //Follow the next steps to configure the device for this example, starting from default registers values.
  //1. Set address 0x01 = 0x11: Connect channel 1’s INP to IN1 and INN to IN3.
  writeRegister(0x01, 0x0b);
  //2. Set address 0x02 = 0x19: Connect channel 2’s INP to IN2 and INN to IN1.
  writeRegister(0x02, 0x11);
  //3. Set address 0x0A = 0x07: Enable the common-mode detector on input pins IN1, IN2 and IN3.
  writeRegister(0x0A, 0x07);
  //4. Set address 0x0C = 0x04: Connect the output of the RLD amplifier internally to pin IN4.
  writeRegister(0x0C, 0x00);
  //5. Set address 0x12 = 0x04: Use external crystal and feed the internal oscillator's output to the digital.
  writeRegister(0x12, 0x04);
  //6. Set address 0x14 = 0x24: Shuts down unused channel 3’s signal path.
  writeRegister(0x14, 0x24);
  //7. Set address 0x21 = 0x02: Configures the R2 decimation rate as 5 for all channels.
  writeRegister(0x21, 0x02);
  //8. Set address 0x22 = 0x02: Configures the R3 decimation rate as 6 for channel 1.
  writeRegister(0x22, 0x02);
  //9. Set address 0x23 = 0x02: Configures the R3 decimation rate as 6 for channel 2.
  writeRegister(0x23, 0x02);
  //10. Set address 0x27 = 0x08: Configures the DRDYB source to channel 1 ECG (or fastest channel).
  writeRegister(0x27, 0x08);
  //11. Set address 0x2F = 0x30: Enables channel 1 ECG and channel 2 ECG for loop read-back mode.
  writeRegister(0x2F, 0x30);
  //12. Set address 0x00 = 0x01: Starts data conversion.
  writeRegister(0x00, 0x01);
}

void setup_ECG_3_channel()
{
  // datasheet ads1293
  //Follow the next steps to configure the device for this example, starting from default registers values.
  //1. Set address 0x01 = 0x11: Connect channel 1’s INP to IN1 and INN to IN3.
  writeRegister(0x01, 0x0b);
  //2. Set address 0x02 = 0x19: Connect channel 2’s INP to IN2 and INN to IN1.
  writeRegister(0x02, 0x11);

  //writeRegister(0x03, 0x2E); //diff

  //3. Set address 0x0A = 0x07: Enable the common-mode detector on input pins IN1, IN2 and IN3.
  writeRegister(0x0A, 0x07);
  //4. Set address 0x0C = 0x03: Connect the output of the RLD amplifier internally to pin IN3.
  writeRegister(0x0C, 0x03);

  writeRegister(0x0D, 0x00); //diff
  writeRegister(0x0E, 0x00); //diff
  writeRegister(0x0F, 0x00); //diff

  writeRegister(0x10, 0x00); //diff

  //5. Set address 0x12 = 0x04: Use external crystal and feed the internal oscillator's output to the digital.
  writeRegister(0x12, 0x04);
  // //6. Set address 0x14 = 0x24: Shuts down unused channel 3’s signal path.
  writeRegister(0x14, 0x24);
  //7. Set address 0x21 = 0x02: Configures the R2 decimation rate as 5 for all channels.
  writeRegister(0x21, 0x02);
  //8. Set address 0x22 = 0x02: Configures the R3 decimation rate as 6 for channel 1.
  writeRegister(0x22, 0x02);
  //9. Set address 0x23 = 0x02: Configures the R3 decimation rate as 6 for channel 2.
  writeRegister(0x23, 0x02);

  //writeRegister(0x24, 0x02); //diff

  //10. Set address 0x27 = 0x08: Configures the DRDYB source to channel 2 ECG (or fastest channel).
  writeRegister(0x27, 0x10);
  //11. Set address 0x2F = 0x30: Enables channel 1 ECG and channel 2 ECG for loop read-back mode.
  writeRegister(0x2F, 0x30); //diff
  //12. Set address 0x00 = 0x01: Starts data conversion.
  writeRegister(0x00, 0x01);
}

byte readRegister(byte reg)
{
  byte data;
  reg |= 1 << 7;
  //hspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  if (checkVH == 0) digitalWrite(pin_SSv, LOW);
  else digitalWrite(pin_SSh, LOW);
  SPI.transfer(reg);
  data = SPI.transfer(0);
  if (checkVH == 0) digitalWrite(pin_SSv, HIGH);
  else digitalWrite(pin_SSh, HIGH);
  //hspi.endTransaction();
  return data;
}

void writeRegister(byte reg, byte data)
{
  reg &= ~(1 << 7);
  //hspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_SSv, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(pin_SSv, HIGH);
  digitalWrite(pin_SSh, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(pin_SSh, HIGH);
  //hspi.endTransaction();
}
//===========SPECIALIZED SPI

void setup()
{
  pinMode(pin_DRDYBv, INPUT_PULLUP);
  pinMode(pin_ALARMBv, INPUT);
  pinMode(pin_SSv, OUTPUT);
  pinMode(pin_DRDYBh, INPUT_PULLUP);
  pinMode(pin_ALARMBh, INPUT);
  pinMode(pin_SSh, OUTPUT);
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);

  //option 1: use hspi specific spi channel
  //hspi.begin(pin_SCLK, pin_MISO, pin_MOSI, pin_SS);
  //option 2: use default spi class methods
  SPI.begin();
  SPI.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  setup_ECG_3_channel();

  ++i;
  bootCount[i]=i;
  //Serial.println("Boot number: " + String(bootCount[i]));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  //Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
//  " Seconds");

  

    //Serial.println("Going to sleep now");
  //Serial.flush(); 
  //esp_deep_sleep_start();
  //Serial.println("This will never be printed");
}

void loop()
{
  present=millis();
  if(present-previous>=900){
    previous=present;
  while(1){
  if (digitalRead(pin_ALARMBv) == false || digitalRead(pin_ALARMBh) == false)
   {
     //Serial.println("alarm raised");
      err1 = readRegister(0x18);
      err2 = readRegister(0x19);
      err3 = readRegister(0x1A);
      err4 = readRegister(0x1B);
      err5 = readRegister(0x1C);
      err6 = readRegister(0x1D);
      err7 = readRegister(0x1E);
   }
   if (digitalRead(pin_DRDYBv) == false && digitalRead(pin_DRDYBh) == false)
   {
    checkVH=0;
    // sampled data is located at 3 8-bit
      //--CHANNEL 1
      c1v = getValFromChannel(1);
      //--CHANNEL 2
      c2v = getValFromChannel(2);
  
    checkVH=1;
    // sampled data is located at 3 8-bit
      //--CHANNEL 1
      c1h = getValFromChannel(1);
      //--CHANNEL 2
      c2h = getValFromChannel(2);
      Serial.print(c2v);
      Serial.print(",");
      Serial.println(c2h);

      // read the sensor
  IMU.readSensor();
  // display the data
  accx=IMU.getAccelX_mss();
  accy=IMU.getAccelY_mss();
  accz=IMU.getAccelZ_mss();
  gyx=IMU.getGyroX_rads();
  gyy=IMU.getGyroY_rads();
  gyz=IMU.getGyroZ_rads();
  magx=IMU.getMagX_uT();
  magy=IMU.getMagY_uT();
  magz=IMU.getMagZ_uT();
  temp=IMU.getTemperature_C();
  BatteryVoltage=ReadBatteryVoltage();
    break;
   }
  }
    
    FirebaseJson json;
      //Serial.print("set json ...");
    
    json.set("ehg/ehgh", c2h);
    json.set("ehg/ehgv", c2v);
    json.set("err/1", err1);
    json.set("err/2", err2);
    json.set("err/3", err3);
    json.set("err/4", err4);
    json.set("err/5", err5);
    json.set("err/6", err6);
    json.set("err/7", err7);
    json.set("imu/acc/x", accx);
    json.set("imu/acc/y", accy);
    json.set("imu/acc/z", accz);
    json.set("imu/gy/r", gyx);
    json.set("imu/gy/p", gyy);
    json.set("imu/gy/y", gyz);
    json.set("imu/mag/x", magx);
    json.set("imu/mag/y", magy);
    json.set("imu/mag/z", magz);
    json.set("imu/bar", 0);
    json.set("imu/temp", temp);
    json.set("bat", BatteryVoltage);
      //Serial.println(" end");
    
    if (Firebase.pushJSON(fbdo, "/user2", json)) {
      //Serial.println("push json");
      Firebase.setTimestamp(fbdo, "/user2/" + fbdo.pushName() + "/timestamp/");
    
      //Serial.println("set timestamp");
    
    } else {
      Serial.println(fbdo.errorReason());
    }
      //Serial.println(millis());
}
}
