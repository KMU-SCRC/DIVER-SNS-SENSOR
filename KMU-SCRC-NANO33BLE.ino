#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_APDS9960.h>
//#include <SoftwareSerial.h>
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float Pressure;
float Temperature, Humidity;
int Proximity;
int gesture;

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
/*  //in-built LED
  pinMode(LED_BUILTIN, OUTPUT);
  //Red LED
  pinMode(LEDR, OUTPUT);
  //Green LED
  pinMode(LEDG, OUTPUT);
  //Blue LED
  pinMode(LEDB, OUTPUT);
*/
  if (!IMU.begin()) { //LSM9DSI센서 시작
    Serial.println("Error initializing LSM9DSI");
    while (1);
  }
  if (!BARO.begin()) { //LPS22HB센서 시작
    Serial.println("Error initializing LPS22HB");
    while (1);
  }
  if (!HTS.begin()) { //HTS221센서 시작
    Serial.println("Error initializing HTS221");
    while (1);
  }
  if (!APDS.begin()) { //APDS9960센서 시작
    Serial.println("Error initializing APDS9960");
    while (1);
  }
  /*
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  */
}

void loop() {

  //가속도센서
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    Serial.print("acceleration="); Serial.print(accel_x); Serial.print(","); Serial.print(accel_y); Serial.print(","); Serial.println(accel_z);
  }
  //자이로센서
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    Serial.print("angularRate="); Serial.print(gyro_x); Serial.print(","); Serial.print(gyro_y); Serial.print(","); Serial.println(gyro_z);
  }
  //지자계센서
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mag_x, mag_y, mag_z);
    Serial.print("gaussMagnetic="); Serial.print(mag_x); Serial.print(","); Serial.print(mag_y); Serial.print(","); Serial.println(mag_z);
  }
  //압력센서
  Pressure = BARO.readPressure();
  Serial.print("dryBarometricPressure="); Serial.println(Pressure);
  //온도센서
  Temperature = HTS.readTemperature();
  Serial.print("dryTemperature="); Serial.println(Temperature);
  //습도센서
  Humidity = HTS.readHumidity();
  Serial.print("dryHumidity="); Serial.println(Humidity);
  //근접센서
  while (!APDS.proximityAvailable()) {};
  Proximity = APDS.readProximity();
  Serial.print("proximity="); Serial.println(Proximity);
  //Serial.println("----------------------------------------------------");
  
  //주변색인식센서
  while (! APDS.colorAvailable()) {
    delay(5);
  }
  int r, g, b;

  // read the color
  APDS.readColor(r, g, b);

/*
  if (r > g & r > b)
  {
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
  else if (g > r & g > b)
  {
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, HIGH);
  }
  else if (b > g & b > r)
  {
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
  }
  else
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
*/

//  delay(1000);

  Serial.print("dryAmbientRGB="); Serial.print("R."); Serial.print(r); Serial.print(","); Serial.print("G."); Serial.print(g); Serial.print(","); Serial.print("B."); Serial.println(b);

  delay(5000);

   //GNSS센서
   /*
  if (Serial1.available()) { 
     Serial.write(Serial1.read());
      if (receivedData == '1') {
      digitalWrite(LED_BUILTIN, HIGH); // switch LED On
    }
    else if (receivedData == '0') {
      digitalWrite(LED_BUILTIN, LOW);  // switch LED Off
    }

  }
    */
/*
  //제스처인식센서#1
  if (APDS.gestureAvailable()) {
    delay(5);
  }
  gesture = APDS.readGesture();
  Serial.print("gesture="); Serial.println(gesture);
*/
  //제스처인식센서#2
  /*
  if (APDS.gestureAvailable()) {
  // a gesture was detected, read and print to serial monitor
  int gesture = APDS.readGesture();
    switch (gesture) {
      case GESTURE_UP:
        Serial.println("Detected UP gesture");
        digitalWrite(LEDR, LOW);
        delay(1000);
        digitalWrite(LEDR, HIGH);
        break;
      case GESTURE_DOWN:
        Serial.println("Detected DOWN gesture");
        digitalWrite(LEDG, LOW);
        delay(1000);
        digitalWrite(LEDG, HIGH);
        break;
      case GESTURE_LEFT:
        Serial.println("Detected LEFT gesture");
        digitalWrite(LEDB, LOW);
        delay(1000);
        digitalWrite(LEDB, HIGH);
        break;
      case GESTURE_RIGHT:
        Serial.println("Detected RIGHT gesture");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        break;
      default:
        break;
    }
  }
*/

}