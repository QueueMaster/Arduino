#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_L3GD20_U.h> //Using unified library for L3GD20
#include <Adafruit_ADXL343.h> //Using specific library for ADXL343
#include <Wire.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3D //Adjustable, OLED may have different I2C address
#define MASS 1500.0f //kg, define mass (using float for calculations)
#define NUM_CALIBRATION_SAMPLES 200 //Number of samples to average for bias
#define RAD_TO_DEG 57.2958f
#define ALPHA 0.98 //Adjustable, complementary filler parameter

float forceX=0;
float forceY=0;
float forceZ=0;

unsigned long previousTime=0;
float deltaTime=0;

float angleX_rad=0; //Roll angle
float angleY_rad=0; //Pitch angle
float angleZ_rad=0; //Yaw angle

float angleX_deg=0;
float angleY_deg=0;
float angleZ_deg=0;

float gyroBiasX=0;
float gyroBiasY=0;
float gyroBiasZ=0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_L3GD20_Unified gyro=Adafruit_L3GD20_Unified(20); //Instantiate with sensor ID
Adafruit_ADXL343 accel=Adafruit_ADXL343(&Wire); //Pass wire object for I2C

void setup() {
  Serial.begin(115200);
  Serial.println("L3GD20H and ADXL343 Display with Complementary Filler");

  //Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed."));
    while (1);
  }
  Serial.println("OLED initialized.");
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();

  //Initialize gyroscope
  if (!gyro.begin()) {
    Serial.println("Failed to find L3GD20 chip");
    while (1);
  }
  Serial.println("L3GD20 found.");
  gyro.enableAutoRange(true); //Enable auto ranging

  //Calibrate gyroscopic bias
  Serial.println("Calibrating gyro...");
  float sumX=0, sumY=0, sumZ=0;
  for (int i=0; i<NUM_CALIBRATION_SAMPLES; i++) {
    sensors_event_t event;
    gyro.getEvent(&event);
    sumX += event.gyro.x;
    sumY += event.gyro.y;
    sumZ += event.gyro.z;
    delay(10);
  }
  gyroBiasX=sumX / NUM_CALIBRATION_SAMPLES;
  gyroBiasY=sumY / NUM_CALIBRATION_SAMPLES;
  gyroBiasZ=sumZ / NUM_CALIBRATION_SAMPLES;
  Serial.print("Gyro Bias X="); Serial.print(gyroBiasX,4);
  Serial.print(",Y="); Serial.print(gyroBiasY,4);
  Serial.print(", Z="); Serial.print(gyroBiasZ,4);
  Serial.println("Calibration complete.");

  //Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("ADXL343 not found.");
    while (1);
  } else {
    Serial.println("ADXL343 found.");
  }

  accel.setDataRate(ADXL343_DATARATE_100_HZ); //Example data rate seting
  accel.setRange(ADXL343_Range_2_G); //Accelerometer range

  previousTime=millis();
}

void loop() {

  unsigned long currentTime=millis();
  deltaTime=(currentTime-previousTime) / 1000.0f;

  //Get gyroscope event
  sensors_event_t gyroEvent;
  gyro.getEvent(&gyroEvent);

  //Get accelerometer event
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);

  //Subtract bias from angular velocities
  float correctedGyroX=gyroEvent.gyro.x-gyroBiasX;
  float correctedGyroY=gyroEvent.gyro.y-gyroBiasY;
  float correctedGyroZ=gyroEvent.gyro.z-gyroBiasZ;

  //Calculate roll and pitch from accelerometer
  float accRoll=atan2(accelEvent.acceleration.y, accelEvent.acceleration.z);
  float accPitch=atan2(-accelEvent.acceleration.x, sqrt(accelEvent.acceleration.y*accelEvent.acceleration.y+accelEvent.acceleration.z*accelEvent.acceleration.z));

  //Complementary filter
  angleX_rad=ALPHA*(angleX_rad+correctedGyroX*deltaTime)+(1-ALPHA)*accRoll;
  angleY_rad=ALPHA*(angleY_rad+correctedGyroY*deltaTime)+(1-ALPHA)*accPitch;
  angleZ_rad+=correctedGyroZ*deltaTime; //Integrate yaw from gyro only

  angleX_deg=angleX_rad*RAD_TO_DEG;
  angleY_deg=angleY_rad*RAD_TO_DEG;
  angleZ_deg=angleZ_rad*RAD_TO_DEG;

  //Calculate forces with Newton's 2nd Law of Motion
  forceX=MASS*accelEvent.acceleration.x;
  forceY=MASS*accelEvent.acceleration.y;
  forceZ=MASS*accelEvent.acceleration.z;

  //Display data on OLED
  display.clearDisplay();

  //Display angles (deg)
  display.setCursor(0,0);
  display.print("Angle (deg)");

  display.setCursor(0,8);
  display.print("R (X):");
  display.print(angleX_deg,1);

  display.setCursor(64,8); //Move to right side for pitch
  display.print("P (Y):");
  display.print(angleY_deg,1);

  display.setCursor(0,16); //New line for yaw
  display.print("Y (Z):");
  display.print(angleZ_deg,1);

  //Display acceleration data
  display.setCursor(0,24);
  display.print("Accel (m/s^2)");

  display.setCursor(0,32);
  display.print("X:");
  display.print(accelEvent.acceleration.x,1);

  display.setCursor(40,32);
  display.print("Y:");
  display.print(accelEvent.acceleration.y,1);

  display.setCursor(80,32);
  display.print("Z:");
  display.print(accelEvent.acceleration.z,1);

  //Display force data
  display.setCursor(0,40);
  display.print("Force (N)");

  display.setCursor(0,48);
  display.print("X:");
  display.print(forceX,1);

  display.setCursor(40,48);
  display.print("Y:");
  display.print(forceY,1);

  display.setCursor(0,56);
  display.print("Z:");
  display.print(forceZ,1);

  display.display();

  previousTime=currentTime;
  delay(100); //Adjustable
}