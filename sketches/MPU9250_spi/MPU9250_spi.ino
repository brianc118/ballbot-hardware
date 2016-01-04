#include <SPI.h>
#include <MPU9250.h>

uint8_t MAG_ADR = 0x0c;

#define SPI_CLOCK 8000000

#define MOSI_PIN 11
#define SCK_PIN  14
#define SS_PIN   10  //cs
#define MISO_PIN 12

#define AK8963

uint8_t buffer[14];  
int16_t ax,ay,az;
int16_t p,q,r;
int16_t temp;
int16_t mx,my,mz;



void setup() {
  
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.begin();
  
  
  pinMode(SS_PIN, OUTPUT);
  
  uint8_t who_am_i;
  while(!Serial.available()){};
  while(Serial.available()){
  	Serial.read();
  };
  //  I2Cdev::readByte(ADR, 0x75, &who_am_i);
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));  // gain control of SPI bus
  digitalWrite(SS_PIN,LOW);
  uint8_t dump;
  dump = SPI.transfer(0xF5); // READ(MSB=1) 0x80 or 0x75 -> 0xF5
  who_am_i = SPI.transfer(0);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  
  if(who_am_i == 0x71){
      Serial.println("Successfully connected to MPU9250");

      // I2Cdev::writeByte(ADR, 0x1b, 0x00);
      // GYRO_CONFIG(0x1B) -> 0x00
      // Full Scale Accelerometer Range  = 250 deg/s
      SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));  // gain control of SPI bus
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x1b); //  Write(MSB=0)
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      SPI.endTransaction();
      delay(1);
      
      // I2Cdev::writeByte(ADR, 0x1c, 0x00);
      // ACCEL_CONFIG(0x1C) -> 0x00
      SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));  // gain control of SPI bus
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x1c);
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      SPI.endTransaction();
      delay(1);
      
      // I2Cdev::writeByte(ADR, 0x6b, 0x00);
      // PWR_MGMT_1(0x6B) -> 0x00
      SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));  // gain control of SPI bus
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x6b);
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      SPI.endTransaction();
      delay(1);

 }
  else{
  	  Serial.println(who_am_i);
      Serial.println("Failed to Connect to MPU9250");
  }

  // Connect to COMPASS(AK8963) via AUX_i2c 


}

void loop() {

  uint8_t dump;
  uint8_t who_am_i;
  int16_t i;
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));  // gain control of SPI bus
  digitalWrite(SS_PIN,LOW);
//  dump = SPI.transfer(0xF5);
//  who_am_i = SPI.transfer(0);
  SPI.transfer(0xbb);
  for(i=0;i<14;i++){
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();

  ax = (((int16_t)buffer[0]) << 8) | buffer[1];
  ay = (((int16_t)buffer[2]) << 8) | buffer[3];
  az = (((int16_t)buffer[4]) << 8) | buffer[5];
  temp = (((int16_t)buffer[6]) << 8) | buffer[7];
  p = (((int16_t)buffer[8]) << 8) | buffer[9];
  q = (((int16_t)buffer[10]) << 8) | buffer[11];
  r = (((int16_t)buffer[12]) << 8) | buffer[13];
  
  uint8_t mag_status2;

    Serial.print(ax);    Serial.print(",");
    Serial.print(ay);    Serial.print(",");
    Serial.print(az);    Serial.print(",");
    Serial.print(p);    Serial.print(",");
    Serial.print(q);    Serial.print(",");
    Serial.print(r);    Serial.print(",");
    Serial.print(mx);    Serial.print(",");
    Serial.print(my);    Serial.print(",");
    Serial.print(mz);    Serial.print(",");
    Serial.println(temp);
  
  delay(100);
  
  
}
