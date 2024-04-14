#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ultrasonic.h>

#define BT_RX_PIN 12
#define BT_TX_PIN 13

#define SERVO_MOTOR_PIN 10
#define SERVO_MOTOR_DEFAULT_ANGLE 115
#define SERVO_MOTOR_MAX_ANGLE 70
#define MIN_SAFE_DISTANCE 15

#define LED_GREEN_PIN 11
#define LED_RED_PIN A0 //A0

#define LED_IR_LEFT_PIN A4
#define LED_IR_RIGHT_PIN A5

#define BUZZER_PIN A2

#define WHEEL_R1_PIN 5
#define WHEEL_R2_PIN 3

#define WHEEL_L1_PIN A3
#define WHEEL_L2_PIN 6

#define ULTRASONIC_TRIG_PIN 7
#define ULTRASONIC_ECHO_PIN 8

#define OFF 0x1
#define ON 0x0

// Create software serial object to communicate with BC417
Servo myServo; 
SoftwareSerial BTserial(BT_RX_PIN, BT_TX_PIN); // RX, TX
Ultrasonic ultrasonic(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
String btCommandBuffer = ""; 
String lcCommandBuffer = ""; 

void setup() {
  // Setup buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  
  // 设置所有电机引脚为输出
  pinMode(WHEEL_R1_PIN, OUTPUT);
  pinMode(WHEEL_R2_PIN, OUTPUT);
  pinMode(WHEEL_L1_PIN, OUTPUT);
  pinMode(WHEEL_L2_PIN, OUTPUT);

  // IR Obstacle sensor
  pinMode(LED_IR_LEFT_PIN, INPUT);
  pinMode(LED_IR_RIGHT_PIN, INPUT);

  // ULTRASONIC sensor
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  
  // Servo Motor
  pinMode(SERVO_MOTOR_PIN, OUTPUT);
  myServo.attach(SERVO_MOTOR_PIN);
  // LEDs
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  // // Open serial communications:
  Serial.begin(9600);        
  BTserial.begin(9600);  
  init_state();

  Serial.println("Robot is Ready");  
  BTserial.println("Remote Robot is Ready");  
}

void init_state() {
  digitalWrite(LED_RED_PIN, OFF);
  digitalWrite(LED_GREEN_PIN, OFF);
  myServo.write(SERVO_MOTOR_DEFAULT_ANGLE);
  stopMotors();
}

// 停止所有电机
void stopMotors() {
  digitalWrite(WHEEL_R1_PIN, LOW);
  digitalWrite(WHEEL_R2_PIN, LOW);
  digitalWrite(WHEEL_L1_PIN, LOW);
  digitalWrite(WHEEL_L2_PIN, LOW);
}

// 驱动左右电机向前
void driveForward(float speed) {  
  if (ensureSafeDistance()) {
    digitalWrite(WHEEL_R2_PIN, LOW);  
    digitalWrite(WHEEL_L2_PIN, LOW);        
    analogWrite(WHEEL_R1_PIN, abs(speed) * 255);  // 发送PWM信号
    analogWrite(WHEEL_L1_PIN, abs(speed) * 255);  // 发送PWM信号
  }
}

// 驱动左右电机向后
void driveBackward(float speed) {
  digitalWrite(WHEEL_R1_PIN, LOW);  
  digitalWrite(WHEEL_L1_PIN, LOW);      
  analogWrite(WHEEL_L2_PIN, speed * 255);  // 发送PWM信号
  analogWrite(WHEEL_R2_PIN, speed * 255);  // 发送PWM信号
}

void turnLeft() {
  int speed = 100;
  digitalWrite(WHEEL_L1_PIN, LOW);    
  analogWrite(WHEEL_L2_PIN, speed);   
  digitalWrite(WHEEL_R1_PIN, speed);      
  analogWrite(WHEEL_R2_PIN, LOW); 
}
void turnRight() {
  int speed = 100;
  digitalWrite(WHEEL_L1_PIN, speed);    
  analogWrite(WHEEL_L2_PIN, LOW);   
  digitalWrite(WHEEL_R1_PIN, LOW);      
  analogWrite(WHEEL_R2_PIN, speed); 
}
void runMotors(float speed) {  
  if (speed < 0) {
    driveForward(speed);
  } else if (speed > 0) {
    driveBackward(speed);
  } else {
    stopMotors();
  }
}

void beep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
}

bool ensureSafeDistance() {
  bool wheel_l = digitalRead(WHEEL_L1_PIN);
  bool wheel_r = digitalRead(WHEEL_R1_PIN);
  if (wheel_l && wheel_r) {
    myServo.write(SERVO_MOTOR_DEFAULT_ANGLE);
    float distance = ultrasonic.read();
    if (distance <= MIN_SAFE_DISTANCE) {
      stopMotors();
      beep();    
      return false;  
    } 
  }
  return true;
}

void loop() {
  ensureSafeDistance();
  if (Serial.available()) {
    char received = Serial.read();
    if (received == '\n') {  // 假设命令以换行符结束
      execCmd(lcCommandBuffer);
      lcCommandBuffer = "";  // 清空缓冲区以准备下一个命令
    } else {
      lcCommandBuffer += received;  // 添加字符到缓冲区
    }
  }

  if (BTserial.available()) {
    char received = BTserial.read();
    if (received == '\n') {  // 假设命令以换行符结束
      execCmd(btCommandBuffer);
      btCommandBuffer = "";  // 清空缓冲区以准备下一个命令
    } else {
      btCommandBuffer += received;  // 添加字符到缓冲区
    }
  }
}


bool hasObstacleLeft() {
  int sensorValue = digitalRead(LED_IR_LEFT_PIN);
  return sensorValue == LOW;
}

bool hasObstacleRight() {
  int sensorValue = digitalRead(LED_IR_RIGHT_PIN);
  return sensorValue == LOW;
}

void execCmd(String cmd){
  cmd.toUpperCase();
  Serial.println("CMD: " + cmd);  
  if (cmd == "LED_R" || cmd == "LR") {
    bool state = digitalRead(LED_RED_PIN);
    digitalWrite(LED_RED_PIN, !state);
  } 
  else if (cmd == "LED_G" || cmd == "LG") {
    bool state = digitalRead(LED_GREEN_PIN); 
    digitalWrite(LED_GREEN_PIN, !state);
  }
  else if (cmd == "INIT") {
    init_state();
  }
  else if (cmd.startsWith("SERVO") || cmd.startsWith("S ")) {
    int spaceIndex = cmd.indexOf(' ');  // Find the space character
    if (spaceIndex != -1) {
      String angleStr = cmd.substring(spaceIndex + 1);  // Get the substring after "servo "
      int angle = angleStr.toInt();  // Convert the angle part to an integer      
      if (angle >= SERVO_MOTOR_MAX_ANGLE) {angle = SERVO_MOTOR_MAX_ANGLE;}
      else if (angle <= -SERVO_MOTOR_MAX_ANGLE) {angle = -SERVO_MOTOR_MAX_ANGLE;}
      int new_angle = SERVO_MOTOR_DEFAULT_ANGLE + (0-angle);
      int current_angle = myServo.read();
      if (current_angle !=  new_angle) {
        myServo.write(new_angle);  // Command the servo to move to the angle
        Serial.println("Servo moved to angle: " + String(new_angle));    
      }
    }
  }
  else if (cmd.startsWith("RUN") || cmd.startsWith("R ")) {
    int spaceIndex = cmd.indexOf(' ');  // Find the space character
    if (spaceIndex != -1) {
      String angleStr = cmd.substring(spaceIndex + 1);  // Get the substring after "servo "
      float speed = angleStr.toFloat();  // Convert the angle part to an integer
      if (speed >= 1) {speed = 1;}
      else if (speed <= -1) {speed = -1;}      
      runMotors(speed);      
    }
  }
  else if (cmd == "READ_DISTANCE"  || cmd == "RD") {
    delay(500);
    float distance = distance = ultrasonic.read();
    // Print the distance in centimeters:    
    String result = "Distance: " + String(distance) + " cm";
    Serial.println(result);      
    BTserial.println(result);
  }
  else if (cmd == "BEEP"  || cmd == "B") {
    beep();
  }
  else if (cmd == "STOP"  || cmd == "STOP") {
    stopMotors();
  }
  else if (cmd == "TURN_LEFT"  || cmd == "tl") {
    turnLeft();
  }
  else if (cmd == "TURN_RIGHT"  || cmd == "tr") {
    turnRight();
  }
  else if (cmd == "DETECT_OBSTACLES" || cmd == "DO") {
    bool left = hasObstacleLeft();    
    bool right = hasObstacleRight();
    if (left || right) {
      if (left) {
        String result = "Left side detected obstacles";
        Serial.println(result);      
        BTserial.println(result);
      }
      if (right) {
        String result = "Right side detected obstacles";
        Serial.println(result);      
        BTserial.println(result);
      }      
    }
    if (!left && !right) {
      String result = "No obstacles detected";
      Serial.println(result);      
      BTserial.println(result);
    }
  }
}