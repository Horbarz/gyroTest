#include <Wire.h>
#include <ESP8266WiFi.h>
#include <DFRobot_sim808.h>
#include <FirebaseArduino.h>
#include <string.h>
#include <SoftwareSerial.h>

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

//Firebase parameters
#define FIREBASE_HOST "caraccidentalert-84950.firebaseio.com" 
#define FIREBASE_AUTH "IDQ18vGIPylAAb74Eq751lbWtTiPZilrut305aL7" 
#define WIFI_SSID "horbarz" 
#define WIFI_PASSWORD "frodopam" 

//Vibration sensor instantiation
#define vibrationSensor A0
#define alarmPin 14

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

//GPS parameters
#define MESSAGE_LENGTH 160
#define PIN_TX    5
#define PIN_RX    4
#define MAX_WORLD_COUNT 300
#define MIN_WORLD_COUNT 2

char *Words[MAX_WORLD_COUNT];
char *StringToParse;
int incomingByte = 0;
char myData[200];
char lon[12];
char lat[12];
char message[MESSAGE_LENGTH];
int messageIndex = 0;
char MESSAGE[300] = "Hello TOMISIN there is an accident at this location, Longitude & Latitude: ";
char wspeed[12];
char phone[16] = "+2348175885645";
char datetime[10] = "8/09/2019";


SoftwareSerial mySerial(PIN_TX,PIN_RX);
DFRobot_SIM808 sim808(&mySerial);//Connect RX,TX,PWR,

void setup() {
  Serial.begin(9600);
  Wire.begin(sda, scl);
  MPU6050_Init();
  pinMode(alarmPin,OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); 
  Serial.print("connecting"); 
  while (WiFi.status() != WL_CONNECTED) { 
    Serial.print("."); 
    delay(500); 
  } 
  Serial.println(); 
  Serial.print("connected: "); 
  Serial.println(WiFi.localIP()); 
   
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

  if ((Gx>0 || Gx<-5.00) && (Gy>0 || Gy<-5.00) && (Ax>-1.23 || Ay<-0.06)){
      
      //sendMessage();
      digitalWrite(alarmPin,HIGH);
      
    }else{
      digitalWrite(alarmPin,LOW);
      }

  delay(100);
  Firebase.setFloat("Data/Accelerometer/Ax", Ax); 
  Firebase.setFloat("Data/Accelerometer/Ay", Ay); 
  Firebase.setFloat("Data/Accelerometer/Az", Az); 
  Firebase.setFloat("Data/Temperature/Temp", T); 
  Firebase.setFloat("Data/Gyroscope/Gx", Gx); 
  Firebase.setFloat("Data/Gyroscope/Gy", Gy); 
  Firebase.setFloat("Data/Gyroscope/Gy", Gz);
  // handle error 
  if (Firebase.failed()) { 
      Serial.print("setting /number failed:"); 
      Serial.println(Firebase.error());   
      return; 
  } 
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

void sendGPSData(){
    mySerial.println("AT+CGNSINF");
    delay(500);
    if(mySerial.find("+CGNSINF: ")){
        mySerial.readBytesUntil(' ',myData,50);
         byte word_count;
        //comma delimited
        word_count = split_message(myData);
          if (word_count >= MIN_WORLD_COUNT) {
                strcat(lat,getLatitude(word_count));
                strcat(lon,getLongitude(word_count));
                //String data = "Hello, the longitude is: ";
                strcat(MESSAGE,lon);
                strcat(MESSAGE," and ");
                strcat(MESSAGE,lat);
//                Serial.println(MESSAGE);
//                sim808.init();
//                sim808.sendSMS(phone,MESSAGE);

                Firebase.setString("Data/Longitude", lon);
                Firebase.setString("Data/Latitude", lat); 
                Serial.print("Longitude: ");
                Serial.print(lon);
                Serial.print("Latitude: ");
                Serial.print(lat);
                // handle error 
                if (Firebase.failed()) { 
                   Serial.print("setting /message failed:"); 
                   Serial.println(Firebase.error());   
                   return; 
                } 
                delay(1000);
               
          }
        
      }
}
byte split_message(char* str) {
  byte word_count = 0; //number of words
  char * item = strtok (str, " ,"); //getting first word (using comma as delimeter)

  while (item != NULL) {
    if (word_count >= MAX_WORLD_COUNT) {
      break;
    }
    Words[word_count] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    word_count++;
  }
  return  word_count;
}

char * getLatitude(byte word_count) {
  for (byte sms_block = 0; sms_block < word_count; sms_block++) {}
  //Serial.print("Latitude: ");
  char * lat = Words[3];
  return lat;
}

char * getLongitude(byte word_count) {
  for (byte sms_block = 0; sms_block < word_count; sms_block++) {}
  //Serial.print("Longitude: ");
  char * lng = Words[4];
  return lng;
}

void sendVibData(){
  // set value 
  float vibrationInput = analogRead(vibrationSensor);
  Firebase.setFloat("Data/Vibration", vibrationInput); 
  // handle error 
  if (Firebase.failed()) { 
      Serial.print("setting /number failed:"); 
      Serial.println(Firebase.error());   
      return; 
  } 
 }  
 void sendMessage(){
    mySerial.println("AT+CGNSINF");
    delay(500);
    if(mySerial.find("+CGNSINF: ")){
        mySerial.readBytesUntil(' ',myData,50);
        byte word_count;
        //comma delimited
        word_count = split_message(myData);
        if (word_count >= MIN_WORLD_COUNT) {
        strcat(lat,getLatitude(word_count));
        strcat(lon,getLongitude(word_count));
                
        strcat(MESSAGE,lon);
        strcat(MESSAGE," and ");
        strcat(MESSAGE,lat);
        Serial.println(MESSAGE);
        sim808.init();
        sim808.sendSMS(phone,MESSAGE);
              
     }
  }
}  
