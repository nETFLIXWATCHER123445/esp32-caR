#include "BluetoothSerial.h"
#include <driver/ledc.h>  // Include the LEDC driver

// Motor control pins
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

// Enable pins
#define ENA 32
#define ENB 33

// PWM settings
#define PWM_FREQ 1000    // 1 KHz
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define ENA_CHANNEL LEDC_CHANNEL_0
#define ENB_CHANNEL LEDC_CHANNEL_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0

// Bluetooth
BluetoothSerial SerialBT;
String device_name = "ESP32_Car";

void setupPWM() {
  // Configure PWM timer
  ledc_timer_config_t timer_conf = {
    .speed_mode = PWM_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT, // Same as PWM_RESOLUTION
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // Configure PWM channels
  ledc_channel_config_t channel_conf = {
    .gpio_num = ENA,
    .speed_mode = PWM_MODE,
    .channel = ENA_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);
  
  channel_conf.gpio_num = ENB;
  channel_conf.channel = ENB_CHANNEL;
  ledc_channel_config(&channel_conf);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);
  Serial.println("Bluetooth device is ready to pair");
  Serial.println("Device name: " + device_name);

  // Setup motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup PWM
  setupPWM();
  
  // Stop motors initially
  stopMotors();
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.println("Received: " + String(command));
    
    switch(command) {
      case 'F': // Forward
        moveForward(255);
        break;
      case 'B': // Backward
        moveBackward(255);
        break;
      case 'L': // Left
        turnLeft(255);
        break;
      case 'R': // Right
        turnRight(255);
        break;
      case 'S': // Stop
        stopMotors();
        break;
      case '1': // Speed 25%
        setSpeed(64);
        break;
      case '2': // Speed 50%
        setSpeed(128);
        break;
      case '3': // Speed 75%
        setSpeed(192);
        break;
      case '4': // Speed 100%
        setSpeed(255);
        break;
      default:
        break;
    }
  }
  delay(20);
}

// Motor control functions
void moveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, speed);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, speed);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}

void moveBackward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, speed);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, speed);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}

void turnLeft(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, speed);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, speed);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, speed);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, speed);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, 0);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, 0);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}

void setSpeed(int speed) {
  ledc_set_duty(PWM_MODE, ENA_CHANNEL, speed);
  ledc_set_duty(PWM_MODE, ENB_CHANNEL, speed);
  ledc_update_duty(PWM_MODE, ENA_CHANNEL);
  ledc_update_duty(PWM_MODE, ENB_CHANNEL);
}
