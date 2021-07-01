#include <ESP32Servo.h>

#define SERIAL_BAUD 115200

#define N_SERVO 1
Servo surfo[N_SERVO];
int SURFO_PIN[N_SERVO] = { 18 };

#define N_RADAR 5
#define NO_PIN 0
#define RADAR_EMULATE true // if that is true, the NO_PINs will get random values
int RADAR_PIN[N_RADAR] = { 34, NO_PIN, NO_PIN, NO_PIN, NO_PIN };
int metric_points[N_RADAR] = {0};

char message_target;
byte message_body;

enum Action { IDLE, SET_SERVO };
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

  for (int r=0; r < N_RADAR; r++) {
    if (RADAR_PIN[r] != NO_PIN) {
      pinMode(RADAR_PIN[r], INPUT);      
    }
  }
  
  Serial.begin(SERIAL_BAUD);
  Serial.println("PapaGuy is listening.");  
}

int current_direction = -1; // assign radar_detection in 0 .. 180 after recognition

#define CHECK_RADAR_EVERY 100
int step = 0;
void loop() {

  if (step % CHECK_RADAR_EVERY == 0) {
    measure_direction_metrics(); // probably don't skip steps, but accumulate

    if (calculate_metric_points()) {
      Serial.print("RADAR!");
      for (int r=0; r < N_RADAR; r++) {
        Serial.print(metric_points[r]);
        Serial.print(";");
      }
      Serial.println("");
      reset_direction_metrics();
    }
  }
  
  for(int s=0; s < N_SERVO; s++) {
    listen_for_message(s);
    execute(s);
  }

  step++;
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

int last_value[N_RADAR] = {0};
int METRIC_integrated_absolute_gradient[N_RADAR] = {0};
int THRESHOLD_integrated_absolute_gradient = 1000;
int METRIC_absolute_over_threshold[N_RADAR] = {0};
int THRESHOLD_absolute_over_threshold = 1000;
int THRESHOLD_absolute_over_threshold_times = 10;

#define NOTHING_DETECTED -1
int strongest_radar;
int second_radar;

void measure_direction_metrics() {
  for (int r=0; r < N_RADAR; r++) {
    int pin = RADAR_PIN[r];
    if (pin == NO_PIN) {
      continue;
    }
    int sensor_value = analogRead(pin);
    int gradient = sensor_value - last_value[r];
    unsigned int abs_gradient = abs(gradient);

    METRIC_integrated_absolute_gradient[r] += abs_gradient;
    if (sensor_value > THRESHOLD_absolute_over_threshold) {
      METRIC_absolute_over_threshold[r]++;
    }
        
    last_value[r] = sensor_value;
  }
}

bool calculate_metric_points() {
  bool any_point_found = false;
  for (int r=0; r < N_RADAR; r++) {
    if (RADAR_PIN[r] == NO_PIN && RADAR_EMULATE) {
      any_point_found = put_emulation_garbage_into_metric_point(r);
      continue;
    }
    
    // in case of these being shitty, try different one(s)
    int metric = METRIC_integrated_absolute_gradient[r];
    int threshold = THRESHOLD_integrated_absolute_gradient;

    if (metric > threshold) {
      metric_points[r]++;
      any_point_found = true;
    }
  }
  return any_point_found;
}

void reset_direction_metrics() {
  for (int r=0; r < N_RADAR; r++) {
    METRIC_integrated_absolute_gradient[r] = 0;
    METRIC_absolute_over_threshold[r] = 0;
    metric_points[r] = 0;
  }
}

bool put_emulation_garbage_into_metric_point(int r) {
  bool any_point_found = false;
  int rnd_int = random(1000);
  if (rnd_int == r) {
    metric_points[r] += random(1, 3);
    if (r > 1) {
      metric_points[r - 1] += random(0, 3);
    }
    if (r < N_RADAR - 1) {
      metric_points[r + 1] += random(0, 3);
    }
    any_point_found = true;
  }
  return any_point_found;
}
