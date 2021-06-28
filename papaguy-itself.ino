#include <ESP32Servo.h>

#define SERIAL_PORT 115200

#define N_SERVO 1
Servo surfo[N_SERVO];
int surfo_pin[N_SERVO] = {
    18
};

char message_head;
short message_body;

enum Action {
  IDLE, SET_SERVO
};
Action action = Action::IDLE;

// update frequency - 20Hz - guess I need to adjust the attach() parameters

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int s=0; s < N_SERVO; s++) {
    surfo[s].setPeriodHertz(50);
    surfo[s].attach(surfo_pin[s], 500, 2400); // pin number, min, max microsecond settings for PWM
  }

  Serial.begin(SERIAL_PORT);
  Serial.println("PapaGuy is alive.");
}

void loop() {
  for(int s=0; s < N_SERVO; s++) {
    listen_for_message(s);
    execute(s);
  }
}

// phi: azimuth in horizontaler ebene (90° mittig); theta: vertikaler winkel, (0° nach unten, 180° nach oben)
enum Message {
    HEAD_PHI = 1,
    WING_LEFT_THETA = 2,
    WING_RIGHT_THETA = 3,
    EYE_LEFT = 20,
    EYE_RIGHT = 21,
    FOG = 22,
    // ...
};

void listen_for_message(int index) {
    if (!surfo[index].attached()) {
      action = Action::IDLE;
      return;
    }
    int result = Serial.readBytes(&message_head, 1);
    Serial.print("MESSAGE_HEAD ");
    Serial.println(result);
}

// need to adjust this if you use a different Servo library!
int servo_state_from(int message_body) {
    return (int)(((float)message_body / 1024.) * 180.);
}

void execute(int index) {
  int current_state, new_state;
  switch (action) {
    case Action::SET_SERVO:
      current_state = surfo[index].read();
      new_state = servo_state_from(message_body);
      if (current_state != new_state) {
        Serial.print("POS ");
        Serial.print(current_state);
        Serial.print(" -> ");
        Serial.println(new_state);
        surfo.write(new_state);
      }
      return;

    case Action::IDLE:
    default:
      return;
  };
}
