#include "NewPing.h"
#include <Wire.h>
#include <SPI.h>
#include "RH_NRF24.h"
RH_NRF24 nrf24;
//#include "SparkFunTMP102.h"
#define PING_MAX_DISTANCE 200
#define PING_PIN 7
int motor_left[] = {2, 3};
int motor_right[] = {6, 5};
NewPing sonar(PING_PIN, PING_PIN, PING_MAX_DISTANCE);
int sensorAddress = 0x91 >> 1;
String abc;

byte msb;
byte lsb;
int temperature;
//const int ALERT_PIN = A3;
//TMP102 sensor0(0x48);
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; i++) {
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
    //pinMode(ALERT_PIN, INPUT);
    //sensor0.begin();
    //sensor0.setFault(0);
    //sensor0.setAlertPolarity(1);
    //sensor0.setAlertMode(0);
    //sensor0.setConversionRate(2);
    //sensor0.setExtendedMode(0);
    //sensor0.setHighTempC(29.4);
    //sensor0.setLowTempC(26.67);
  }
  Wire.begin();
  initRadio();
}
void loop() {
  delay(50);
  int distanceInCms = sonar.ping() / US_ROUNDTRIP_CM;
  //request reading from temp sensor
  
  Serial.println("Distance: " + String(distanceInCms) + "cm");
  //void temp();
  if ( distanceInCms < 25 && distanceInCms != 0)
  {
    turn_around();
    Serial.println("turn around");
  }
  else
  {
    drive_forward();
    Serial.println("drive forward");
  }
  temp();
  sendData();
}
void drive_forward() {
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}
void turn_around()
{
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

void temp() {
  Wire.requestFrom(sensorAddress, 2);
  if (2 <= Wire.available()) {
    msb = Wire.read();
    lsb = Wire.read();
    temperature = ((msb) << 4);
    temperature |= (lsb >> 4);
    //temperature = int(temperature*0.0625);
    abc = "Temperature: " + String(temperature*0.0625);
    Serial.print("Temperature: ");
    Serial.println(temperature*0.0625);
    Serial.println(abc);
  }
  delay(500);
}

void initRadio()
{
  //while (!Serial)
  // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init()) {
    //Serial.println("init failed");
    // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  }
  if (!nrf24.setChannel(1)) {
    // Serial.println("setChannel failed");}
    if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
      //Serial.println("setRF failed");
    }
  }
}

void sendData()
{
  //Serial.println("Sending to nrf24_server");
  // Send a message to nrf24_server
  //convert string into char arry
  char stringarray[16];
  abc.toCharArray(stringarray, 16);
  //uint8_t data[] = "HELLO";
  //unsigned double data=abc;
  nrf24.send(stringarray, sizeof(stringarray));
  //radio.write(data, sizeof(unsigned double);

  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (nrf24.waitAvailableTimeout(500))
  {
    // Should be a reply message for us now
    if (nrf24.recv(buf, &len))
    {
      //Serial.print("got reply: ");
      //Serial.println((char*)buf);
    }
    else
    {
      //Serial.println("recv failed");
    }
  }
  else
  {
    //Serial.println("No reply, is nrf24_server running?");
  }
  
}
//void temp() {
////float temperature;
//boolean alertPinState, alterRegisterState;
//sensor0.wakeup();
//temperature = sensor0.readTempC();
//alertPinState = digitalRead(ALERT_PIN);
//sensor0.sleep();
//Serial.print("Temperature: ");
//Serial.print(temperature);
//delay(1000);
//}
