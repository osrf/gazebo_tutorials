// Arduino sketch for tactor glove
// Author: Morgan Quigley
// pinout: https://www.pjrc.com/teensy/card2b.pdf

#define NUM_MOTORS 5
#define MOTOR_RUN_TIME 100
#define MOTOR_PWM 255
int led = 11;
int motors[NUM_MOTORS] = {4, 9, 10, 12, 14};
unsigned long t_motor_start[NUM_MOTORS] = { 0, 0, 0, 0, 0 };

void setup()
{
  pinMode(led, OUTPUT);
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(motors[i], OUTPUT);
    analogWrite(motors[i], 0);
  }
  Serial.begin(9600); // baud rate is ignored on teensy... it's usb,
baudrate is ignored...
}

void loop()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    if (!t_motor_start[i])
      continue; // motor isn't running
    if (millis() > t_motor_start[i] + MOTOR_RUN_TIME)
    {
      analogWrite(motors[i], 0);
      t_motor_start[i] = 0;
    }
  }

  if (Serial.available())
  {
    char b = Serial.read();
    if (b >= '1' && b <= '5')
    {
      int motor = b - '1';
      t_motor_start[motor] = millis();
      analogWrite(motors[motor], MOTOR_PWM);
    }
  }
}
