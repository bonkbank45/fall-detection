#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>
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
const int reconnectWifi = 5;

#define LINE_TOKEN "YOUR LINE TOKEN" // TOKEN
#define LINE_API "https://notify-api.line.me/api/notify"         // LINE API will deprecate soon!

#define LED_WIFI D5
#define LED_GPS D7
#define LED_POWER D8

bool ledState = LOW;
bool startCountGpsBuffer = false;
unsigned int countGpsBuffer = 0;

const String DEVICE_TOKEN = "YOUR DEVICE TOKEN";

char thingsboardServer[30] = "YOUR THINGSBOARD IP";

char thingsboardServerPort[5] = "9090";

WiFiManager wifiManager;

int getAccAmp();
int getAngleAmp();
float *getGpsInfo();
void getGpsInfoLedStatus();
void sendTelemetry(char *urlServer, char *port, String token, float latiude, float longitude);
void appendFloatToString(String &str, float value);
void lineNotify(float lanitude, float longitde);
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

  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_GPS, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_GPS, LOW);
  digitalWrite(LED_POWER, HIGH);

  WiFiManagerParameter customThingsboardServer("server", "ThingBoard Server Ip Address", thingsboardServer, 40);
  WiFiManagerParameter customThingsboardPort("port", "ThingBoard Server Port", thingsboardServerPort, 6);

  wifiManager.addParameter(&customThingsboardServer);
  wifiManager.addParameter(&customThingsboardPort);

  wifiManager.autoConnect("AutoConnectAP_FallDetection", "your password");

  strcpy(thingsboardServer, customThingsboardServer.getValue());
  strcpy(thingsboardServerPort, customThingsboardPort.getValue());

  if (WiFi.isConnected())
  {
    digitalWrite(LED_WIFI, HIGH);
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

    float *coordinate = getGpsInfo();
    // LED_GPS Status
    if (coordinate[0] == 0.0 && coordinate[1] == 0.0)
    {
      if (startCountGpsBuffer)
      {
        countGpsBuffer++;
        if (countGpsBuffer == 4)
        {
          countGpsBuffer = 0;
          startCountGpsBuffer = false;
        }
      }
      else
      {
        if (ledState == LOW)
          ledState = HIGH;
        else
          ledState = LOW;
        digitalWrite(LED_GPS, ledState);
      }
    }
    else
    {
      digitalWrite(LED_GPS, HIGH);
      startCountGpsBuffer = true;
    }

    if (amp <= 2 && trigger2 == false)
    { // if AM breaks lower threshold (0.4g)
      Serial.println("TRIGGER 1 ACTIVATED");
      state = TRIGGER1;
    }

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
    }

    delay(100);
  }

  else if (state == reconnectWifi)
  {
    digitalWrite(LED_WIFI, LOW);

    wifiManager.autoConnect("AutoConnectAP_FallDetection", "lalafell");

    if (WiFi.isConnected())
    {
      state = filter;
    }
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

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
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

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
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

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
    }

    delay(100);
  }
  else if (state == FALL_ACTIVATION)
  { // in event of a fall detection

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
    }

    Serial.println("FALL DETECTED");
    float *coordinate = getGpsInfo();

    sendTelemetry(thingsboardServer, thingsboardServerPort, DEVICE_TOKEN, coordinate[0], coordinate[1]);
    lineNotify(coordinate[0], coordinate[1]);

    if (!WiFi.isConnected())
    {
      state = reconnectWifi;
    }

    state = filter;
  }
}

void mpu_read()
{
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

void lineNotify(float lanitude, float longitde)
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

  if (lanitude == 0.0 && longitde == 0.0)
  {
    String httpMsg = "message=Fall Detection!\nError: Can't detect location.";
    int httpsResponseCode = https.POST(httpMsg);
    Serial.println("Line Notify Respond Status: " + String(httpsResponseCode));
  }
  else
  {
    String mapLink = "http://maps.google.com/maps?q=";
    appendFloatToString(mapLink, lanitude);
    mapLink += ",";
    appendFloatToString(mapLink, longitde);

    String httpMsg = "message=Fall Detection!\nGoogle Map Link: " + mapLink;
    Serial.println(httpMsg);

    int httpsResponseCode = https.POST(httpMsg);
    Serial.println("Line Notify Respond Status: " + String(httpsResponseCode));
  }
}

void sendTelemetry(char *urlServer, char *port, String token, float latiude, float longitude)
{
  WiFiClient wificlient;
  HTTPClient http;
  char httpProtocol[8] = "http://";
  strcat(httpProtocol, urlServer);
  strcat(httpProtocol, ":");
  strcat(httpProtocol, port);
  String postUrl = httpProtocol;
  postUrl += "/api/v1/" + token + "/telemetry";
  Serial.println(postUrl);
  http.begin(wificlient, postUrl);
  http.addHeader("Content-Type", "application/json");

  String httpMessage = "{latitude:";
  appendFloatToString(httpMessage, latiude);
  httpMessage += ",longitude:";
  appendFloatToString(httpMessage, longitude);
  httpMessage += "}";

  Serial.println(httpMessage);

  int httpStatusCode = http.POST(httpMessage);
  Serial.println(httpStatusCode);
}

void appendFloatToString(String &str, float value)
{
  char buffer[10];
  dtostrf(value, 9, 6, buffer);
  str += buffer;
}

float *getGpsInfo()
{
  /*
    Extraction NMEA and Explain about NMEA. via https://www.artronshop.co.th/article/44/%E0%B8%81%E0%B8%B2%E0%B8%A3%E0%B9%83%E0%B8%8A%E0%B9%89%E0%B8%87%E0%B8%B2%E0%B8%99%E0%B9%82%E0%B8%A1%E0%B8%94%E0%B8%B9%E0%B8%A5-gps-ublox-neo-6m
  */
  static float coordinate[2];
  coordinate[0] = 00.000000;
  coordinate[1] = 00.000000;
  if (SerialGPS.available())
  {
    String line = "";
    while (SerialGPS.available() > 0)
    {
      char c = SerialGPS.read();
      if (c == '\r')
      {
        // Serial.println(line);
        if (line.indexOf("$GPRMC" >= 0))
        {
          String dataCut[13];
          int index = 0;
          for (unsigned int dataStart = 0; dataStart < line.length();)
          {
            dataCut[index] = line.substring(dataStart + 1, line.indexOf(",", dataStart + 1));
            dataStart = line.indexOf(",", dataStart + 1);
            index++;
          }
          if (dataCut[2] == "A")
          {
            int dotPos = 0;
            dotPos = dataCut[3].indexOf("."); // 4916.45 - แยกออกมาได้เป็น 49 องศา 16.45 นาที
            String latDeg = dataCut[3].substring(0, dotPos - 2);
            String latMin = dataCut[3].substring(dotPos - 2, dataCut[3].length());
            dotPos = dataCut[5].indexOf('.');
            String lngDeg = dataCut[5].substring(0, dotPos - 2);
            String lngMin = dataCut[5].substring(dotPos - 2, dataCut[5].length());
            latitude = (latDeg.toFloat() + (latMin.toFloat() / 60.0)) * (dataCut[4] == "N" ? 1 : -1);
            longitude = (lngDeg.toFloat() + (lngMin.toFloat() / 60.0)) * (dataCut[6] == "E" ? 1 : -1);

            coordinate[0] = latitude;
            coordinate[1] = longitude;
          }
          else
          {
            // Serial.println("Error: No Signal from Satalite");
          }
          serialFlush();
        }
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
    serialFlush();
  }
  // Serial.println("No Serial from GPS NEO6m (maybe wire RX, TX connect wrong?)");
  Serial.print("Latitude= ");
  Serial.println(coordinate[0], 6);
  Serial.print("Longtitude= ");
  Serial.println(coordinate[1], 6);
  return coordinate;
}

void serialFlush()
{
  while (Serial.available())
    Serial.read();
}