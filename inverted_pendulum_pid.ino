#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

unsigned long prevTime = 0;  
float angle = 0;
float alpha = 0.65;  // Complementary filter constant
float angleOffset = -12;
float kp=41;
float kd=0;
float prev_error=0;

unsigned long prev_time=0;
#define i1 2
#define i2 4
#define n1 5
void setup() {
  pinMode(i1,OUTPUT);
  pinMode(i2,OUTPUT);
  pinMode(n1,OUTPUT);
 

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Faster I2C

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) delay(1000);
  }

  Serial.println("MPU6050 ready");
  delay(500); // Let sensor settle

  // Initialize angle using accelerometer to avoid jump
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  angle = atan2(accelX, accelY) * 180 / PI - angleOffset;

  prevTime = millis();
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();

    if (input == 'a') {
      kp -= 10;
      Serial.print("kp decreased to: ");
      Serial.println(kp);
    }
    else if (input == 'd') {
      kp += 10;
      Serial.print("kp increased to: ");
      Serial.println(kp);
    }
  }
  float error=angle;
  float now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float gyroRate = gz / 131.0;

  float accelAngle = atan2(accelX, accelY) * 180 / PI - angleOffset;

  // Complementary filter (smooth + responsive)
  float gyroAngle = angle+gyroRate*dt;
  angle = alpha * (gyroAngle) + (1 - alpha) * accelAngle;

  // Output for plotting
  // creating control  
  // float derivative=0;

  float output = kp * angle;
  // derivative = (error - prev_error) / dt;
  prev_error=error;
  int pwmval = (int)constrain(abs(output), 0, 255);

  if(output> 0){
    analogWrite(n1,pwmval);
    digitalWrite(i1,HIGH);
    digitalWrite(i2,LOW);



  }
  else {
    analogWrite(n1,pwmval);
    digitalWrite(i1,LOW);
    digitalWrite(i2,HIGH);

  }
  // Serial.print(accelAngle); 
  // Serial.print(", ");
  // Serial.print(gyroAngle);
  // Serial.print(", ") ;
  // Serial.println(angle);

  delayMicroseconds(1); // ~100Hz loop
}
