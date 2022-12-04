

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float turn = 35;
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;

const int RELAY_ENABLE_Pump = 4;
const int RELAY_ENABLE_Sol13 = 2;
const int RELAY_ENABLE_Sol24 = 3;

void setup() 
{
  pinMode(RELAY_ENABLE_Sol13, OUTPUT);
  pinMode(RELAY_ENABLE_Sol24, OUTPUT);
  pinMode(RELAY_ENABLE_Pump, OUTPUT);

  Serial.begin(115200);


  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // // Calibrate gyroscope. The calibration must be at rest.
  // // If you don't want calibrate, comment this line.
  // mpu.calibrateGyro();

  // // // Set threshold sensivty. Default 3.
  // // // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);
}

void loop()
{
  timer = millis();

  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
  //pitch = pitch + (pitchread);
  //roll = roll + (rollread);

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.println(roll);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
  digitalWrite(RELAY_ENABLE_Sol13, HIGH);
  digitalWrite(RELAY_ENABLE_Sol24, HIGH);
  digitalWrite(RELAY_ENABLE_Pump, HIGH);

  while(roll < 30)
  {
    timer = millis();
    // Read normalized values
    Vector normAccel = mpu.readNormalizeAccel();

    pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
    // pitch = pitch + (pitchread) * timeStep;
    // roll = roll + (rollread) * timeStep;

    // Output raw
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.println(roll);

    // Wait to full timeStep period
    delay((timeStep*1000) - (millis() - timer));

    digitalWrite(RELAY_ENABLE_Sol13, LOW);
    digitalWrite(RELAY_ENABLE_Pump, LOW);
  }
  digitalWrite(RELAY_ENABLE_Sol13, HIGH);
  digitalWrite(RELAY_ENABLE_Pump, HIGH);
  while(roll > 50)  
  { 
    timer = millis();

    Vector normAccel = mpu.readNormalizeAccel();

    // Calculate Pitch & Roll
    pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    //float pitch2 = pitch + norm.YAxis * timeStep
    roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    //float roll2 = pitch + norm.YAxis * timeStep
    // pitch = pitch + (pitchread) * timeStep;
    // roll = roll + (rollread) * timeStep;

    // Output raw
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.println(roll);

    // Wait to full timeStep period
    delay((timeStep*1000) - (millis() - timer));
    digitalWrite(RELAY_ENABLE_Sol24, LOW);
    digitalWrite(RELAY_ENABLE_Pump, LOW);
  }
  digitalWrite(RELAY_ENABLE_Sol24, HIGH);
  digitalWrite(RELAY_ENABLE_Pump, HIGH);
}
