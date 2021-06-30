#include <ESP32Servo.h>

#define SERIAL_BAUD 115200

#define N_SERVO 1
Servo surfo[N_SERVO];
int SURFO_PIN[N_SERVO] = {
  18
};

#define N_RADAR 1
int RADAR_PIN[N_RADAR] = {
  34
};

int HEAD_DIRECTION[N_RADAR] = {
  90,
};

char message_target;
byte message_body;

enum Action {
  IDLE, SET_SERVO
};
Action action = Action::IDLE;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int s=0; s < N_SERVO; s++) {
    surfo[s].setPeriodHertz(50);
    surfo[s].attach(SURFO_PIN[s], 500, 2400); // pin number, min, max microsecond settings for PWM
  }

  for(int r=0; r < N_RADAR; r++) {
    pinMode(RADAR_PIN[r], INPUT);
  }
  
  Serial.begin(SERIAL_BAUD);
  ensure_serial_connection();
}

void ensure_serial_connection() {
  if (Serial) {
    return;
  }
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  Serial.println("PapaGuy is alive.");  
}

int current_direction = -1;

void loop() {
  ensure_serial_connection();
  
  if (radar_detection(&current_direction)) {
    Serial.print("Detected something... ");
    Serial.println(current_direction);
  }
  
  for(int s=0; s < N_SERVO; s++) {
    listen_for_message(s);
    execute(s);
  }

  delay(20);
}

#define HEAD_SERVO 1
// phi: azimuth in horizontaler ebene (90° mittig); theta: vertikaler winkel, (0° nach unten, 180° nach oben)
enum Message {
  HEAD_PHI = HEAD_SERVO,
  WING_LEFT_THETA = 2,
  WING_RIGHT_THETA = 3,
  BEAK = 4,
  ENVELOPE = 17
  // EYES / FOG / ..?
  // KILL = 124 ?
  // PANIC = 125 ? 
};
// for testing, just control servo 1 witih the ENVELOPE information. will be an array later.
#define ENVELOPE 1

void listen_for_message(int index) {
    if (!surfo[index].attached() || Serial.available() == 0) {
      action = Action::IDLE;
      return;
    }
    int target = Serial.readBytes(&message_target, 1);
    int body = Serial.readBytes(&message_body, 2);
    Serial.print("MESSAGE: ");
    Serial.print(target);
    Serial.println(body);
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
        surfo[index].write(new_state);
      }
      return;

    case Action::IDLE:
    default:
      return;
  };
}

bool radar_detection(int *direction) {
  /*
  for(int r=0; r < N_RADAR; r++) {
    int sensor_value = digitalRead(RADAR_PIN[r]);
    Serial.print("RADAR ");
    Serial.print(r);
    Serial.print(": ");
    Serial.println(sensor_value);
  }
  */
  
  int fake_value = random(1000);
  if(fake_value < 5) {
    *direction = fake_value;
    return true;
  }
  return false;
};
