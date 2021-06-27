#include <ESP32Servo.h>

int pos = 0;
int servo_pin = 18;

N_SERVO = 1;
Servo *surfo;

enum Action {
  IDLE, SET_SERVO
};
int servo_target = 0; // 0 is none, t
Action action = Action.IDLE;

// update frequency - 20Hz - guess I need to adjust the attach() parameters

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  surfo.setPeriodHertz(50);
  surfo.attach(servo_pin, 500, 2400); // pin number, min, max microsecond settings
}

void loop() {
  listen_for_message();
  
  switch (action) {
    case Action.SET_SERVO:
      surfo.write(pos);
    
    case Action.IDLE:
    default:
      return;
  };
  if (action == Action.IDLE) {
    return;
  }
  for (pos = 0; pos <= 180; pos++) {
    //delay(5);
  }
  for (pos = 180; pos >= 0; pos--) {
    surfo.write(pos);
    //delay(5);
  }
}
