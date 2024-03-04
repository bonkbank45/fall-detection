#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include "SoftwareSerial.h"
#include "TinyGPS++.h"

SoftwareSerial SerialGPS(2, 0); // RX TX
TinyGPSPlus gps;

float latitude, longitude, speed;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;     // stores if a fall has occurred
boolean trigger1 = false; // stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; // stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; // stores if third trigger (orientation change) has occurred
byte trigger1count = 0;   // stores the counts past since trigger 1 was set true
byte trigger2count = 0;   // stores the counts past since trigger 2 was set true
byte trigger3count = 0;   // stores the counts past since trigger 3 was set true
int angleChange = 0;

int state;
int amp;
const int filter = 0;
const int TRIGGER1 = 1;
const int TRIGGER2 = 2;
const int TRIGGER3 = 3;
const int FALL_ACTIVATION = 4;

#define SSID "YOUR WIFI NAME"
#define PASSWORD "YOUR WIFI PASSWORD"

/*
  https://notify-bot.line.me/doc/en/
*/
#define LINE_TOKEN "YOUR TOKEN"
#define LINE_API "https://notify-api.line.me/api/notify"

int getAccAmp();
int getAngleAmp();
bool displayInfo();
void lineNotify();
void serialFlush();
void mpu_read();

void setup()
{
  Serial.begin(9600);
  SerialGPS.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Connecting to WiFi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  state = filter;
}
void loop()
{
  if (state == filter)
  {
    amp = getAccAmp();
    // Serial.println(amp);,

    if (amp <= 2 && trigger2 == false)
    { // if AM breaks lower threshold (0.4g)
      Serial.println("TRIGGER 1 ACTIVATED");
      state = TRIGGER1;
    }
    delay(100);
  }
  else if (state == TRIGGER1)
  {
    amp = getAccAmp();
    trigger1count++;
    if (amp >= 12)
    { // if AM breaks upper threshold (3g)
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1count = 0;
      state = TRIGGER2;
    }
    if (trigger1count >= 6)
    { // allow 0.5s for AM to break upper threshold
      trigger1count = 0;
      Serial.println("TRIGGER 1 DECACTIVATED");
      state = filter;
    }
    delay(100);
  }
  else if (state == TRIGGER2)
  {
    trigger2count++;
    angleChange = getAngleAmp();
    Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400)
    { // if orientation changes by between 80-100 degrees
      trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
      state = TRIGGER3;
    }
    if (trigger2count >= 6)
    { // allow 0.5s for orientation change
      trigger2count = 0;
      Serial.println("TRIGGER 2 DECACTIVATED");
      state = filter;
    }
    delay(100);
  }
  else if (state == TRIGGER3)
  {
    trigger3count++;
    if (trigger3count >= 10)
    {
      angleChange = getAngleAmp();
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10))
      { // if orientation changes remains between 0-10 degrees
        trigger3count = 0;
        Serial.println(angleChange);
        state = FALL_ACTIVATION;
      }
      else
      { // user regained normal orientation
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
        state = filter;
      }
    }
    delay(100);
  }
  else if (state == FALL_ACTIVATION)
  { // in event of a fall detection
    Serial.println("FALL DETECTED");
    if (displayInfo())
    {
      Serial.print(latitude, 6);
      Serial.print(",");
      Serial.print(longitude, 6);
      Serial.print(" | Speed (Km) : ");
      Serial.print(speed, 2);
      Serial.println();
      delay(2000);
    }
    else
    {
      Serial.println("Can't find Location");
    }
    // lineNotify(latitude,longitude); waiting for develop this function to send data to api line
    // send to database
    state = filter;
  }
}

void mpu_read()
{
  /*
    Wire.read() << 8: ในบรรทัดนี้, Wire.read() จะอ่านข้อมูลจาก MPU ที่ถูกส่งมาผ่าน I2C และคืนค่า
    ในรูปแบบของไบต์ (8 บิต). << 8 คือการทำ bitwise shift ซึ่งหมายถึงการเลื่อนค่าไบต์ทั้งหมดไปทางซ้ายขึ้น 8 ที
    ทำให้ได้ค่าที่เหมาะสมสำหรับบิตที่สูงขึ้นของค่าที่จะสร้างขึ้น.
    | Wire.read();: ในขั้นตอนนี้, Wire.read() จะอ่านข้อมูลไบต์ที่ต่อมาที่จะถูก OR กับค่าที่ได้จากขั้นตอนก่อนหน้า. 
    การ OR ทำให้บิตที่ต่ำกว่าของค่าที่ได้จาก Wire.read() << 8 ถูกเติมเต็มไปด้วยข้อมูลใหม่ที่ได้จาก Wire.read().
    รวมกัน, ข้อมูลจาก ACCEL_XOUT_H และ ACCEL_XOUT_L ถูกอ่านจาก MPU และถูกประมวลผลในบรรทัดนี้เพื่อสร้างค่าเต็มที่
    แทนค่าความเร่งในแกน X ของ MPU. การ shift และ OR ทำให้ได้ค่าที่ถูกต้องตามรูปแบบที่ใช้ในการเก็บค่าแบบ signed integer (16-bit).
  */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

int getAccAmp()
{
  mpu_read();
  // calibration
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  // amp acc
  float Mag_Acc = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Acc = Mag_Acc * 10; // Mulitiplied by 10 bcz values are between 0 to 1
  return Acc;
}

int getAngleAmp()
{
  mpu_read();
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  float Mag_Angle = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
  return Mag_Angle;
}

void lineNotify()
{
  HTTPClient https;
  /*
    Using BearSSL because We can't send REQ to Line Api with HTTP
    so We will use BearSSL to secure connection (ESP8266 have BearSSL)
  */
  std::unique_ptr<BearSSL::WiFiClientSecure> client(new BearSSL::WiFiClientSecure);
  client->setInsecure();
  https.begin(*client, LINE_API);
  https.addHeader("Authorization", "Bearer " + String(LINE_TOKEN));
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String httpMsg = "message=Fall Detection!";
  Serial.println(httpMsg);
  int httpsResponseCode = https.POST(httpMsg);
  Serial.println("Line Notify Respond Status: " + String(httpsResponseCode));
}

bool displayInfo()
{
  /*
    Extraction NMEA and Explain about NMEA. via https://www.artronshop.co.th/article/44/%E0%B8%81%E0%B8%B2%E0%B8%A3%E0%B9%83%E0%B8%8A%E0%B9%89%E0%B8%87%E0%B8%B2%E0%B8%99%E0%B9%82%E0%B8%A1%E0%B8%94%E0%B8%B9%E0%B8%A5-gps-ublox-neo-6m
  */
  if (SerialGPS.available())
  {
    String line;
    while (SerialGPS.available() > 0)
    {
      char c = SerialGPS.read();
      if (c == '\r')
      {
        Serial.println(line);
        return true;
      }
      else if (c == '\n')
      {
        // pass
      }
      else
      {
        line += c;
      }
      delay(1);
    }
  }
  return false;
}

// bool displayInfo()
// {
//   if (SerialGPS.available())
//   {
//     String line = "";
//     while (SerialGPS.available())
//     {
//       char c = SerialGPS.read();
//       if (c == '\r')
//       {
//         Serial.println(line);
//         if (line.indexOf("$GPRMC") >= 0)
//         {
//           // Serial.println(line);
//           String dataCut[13];
//           int index = 0;
//           for (int dataStart = 0; dataStart < line.length();)
//           {
//             dataCut[index] = line.substring(dataStart + 1, line.indexOf(',', dataStart + 1));
//             // Serial.println(dataCut[index]);
//             dataStart = line.indexOf(',', dataStart + 1);
//             index++;
//           }
//           if (dataCut[2] == "A")
//           {
//             int dotPos = 0;
//             dotPos = dataCut[3].indexOf('.');
//             String latDeg = dataCut[3].substring(0, dotPos - 2);
//             String latMin = dataCut[3].substring(dotPos - 2, dotPos + 10);
//             dotPos = dataCut[5].indexOf('.');
//             String lngDeg = dataCut[5].substring(0, dotPos - 2);
//             String lngMin = dataCut[5].substring(dotPos - 2, dotPos + 10);
//             latitude = (latDeg.toFloat() + (latMin.toFloat() / 60.0)) * (dataCut[4] == "N" ? 1 : -1);
//             longitude = (lngDeg.toFloat() + (lngMin.toFloat() / 60.0)) * (dataCut[6] == "E" ? 1 : -1);
//             speed = dataCut[7].toFloat() * 1.652;

//             return true;
//           }
//           else
//           {
//             Serial.println("Error : No fix now.");
//           }
//           serialFlush();
//         }
//         line = "";
//       }
//       else if (c == '\n')
//       {
//         // pass
//       }
//       else
//       {
//         line += c;
//       }
//       delay(1);
//     }
//   }
//   return false;
// }

// void serialFlush()
// {
//   while (Serial.available())
//     Serial.read();
// }