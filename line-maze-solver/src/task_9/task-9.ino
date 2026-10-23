#include <Arduino.h>

// sensor
const int SENSOR_PIN[5]={34,35,32,33,25}; // L->R
const float WEIGHTS[5]={-2.0,-1.0,0.0,1.0,2.0};

//l298N
const int PIN_ENA = 14; // Left motor PWM speed pin (0 to 255 via analogWrite)
const int PIN_IN1 = 27; // Left motor direction pin A
const int PIN_IN2 = 26; // Left motor direction pin B
const int PIN_ENB = 13; // Right motor PWM speed pin (0 to 255 via analogWrite)
const int PIN_IN3 = 12; // Right motor direction pin A
const int PIN_IN4 = 11; // Right motor direction pin B

// will be declared later
float KP;
float KD;
int SPEED = 150;

float lastError = 0.0;

void setMotor(int pinA, int pinB, int pinPwm, int speed) {
    digitalWrite(pinA, (speed >= 0) ? HIGH : LOW);
    digitalWrite(pinB, (speed >= 0) ? LOW : HIGH);
    analogWrite(pinPwm, abs(speed));
}

void setup() {

    Serial.begin(115200);

    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PIN[i], INPUT);
    }
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
}

void loop() {
    int sensor[5];
    float weightedSum = 0.0;
    float totalSum = 0.0;

    for (int i = 0; i < 5; i++) {
        sensor[i] = digitalRead(SENSOR_PIN[i]);
        weightedSum += sensor[i] * WEIGHTS[i];
        totalSum    += sensor[i];
    }

    // line lost: stop
    if (totalSum == 0) {
        setMotor(PIN_IN1, PIN_IN2, PIN_ENA, 0);
        setMotor(PIN_IN3, PIN_IN4, PIN_ENB, 0);
        return;
    }

    float error = weightedSum / totalSum;
    float correction = KP * error + KD * (error - lastError)/0.01;
    lastError = error;

    int leftSpeed  = constrain(SPEED + (int)correction, -255, 255);
    int rightSpeed = constrain(SPEED - (int)correction, -255, 255);

    setMotor(PIN_IN1, PIN_IN2, PIN_ENA, leftSpeed);
    setMotor(PIN_IN3, PIN_IN4, PIN_ENB, rightSpeed);

    Serial.printf("s=%d%d%d%d%d err=%.2f L=%d R=%d\n",sensor[0], sensor[1], sensor[2], sensor[3], sensor[4],error, leftSpeed, rightSpeed);

    delay(10);
}