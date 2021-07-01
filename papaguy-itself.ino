#include <ESP32Servo.h>

#define SERIAL_BAUD 115200

#define N_SERVO 1
Servo surfo[N_SERVO];
int SURFO_PIN[N_SERVO] = { 18 };

#define N_RADAR 5
int RADAR_PIN[N_RADAR] = { 34, 0, 0, 0, 0 };
int metric_points[N_RADAR] = {0};
int HEAD_DIRECTION[N_RADAR] = { 20, 55, 90, 125, 160 };

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

  for(int r=0; r < N_RADAR; r++) {
    pinMode(RADAR_PIN[r], INPUT);
  }
  
  Serial.begin(SERIAL_BAUD);
  Serial.println("PapaGuy is listening.");  
}

int current_direction = -1; // assign radar_detection in 0 .. 180 after recognition

#define CHECK_RADAR_EVERY 100
int step = 0;
void loop() {

  if (step % CHECK_RADAR_EVERY == 0) {
    if (radar_detection(&current_direction)) {
      Serial.print("RADAR_");
      for (int r=0; r < N_RADAR; r++) {
        Serial.print(metric_points[r]);
      }
      Serial.println(";");
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

bool radar_detection(int *direction) {
  measure_direction_metrics();
  
  int strongest_radar = NOTHING_DETECTED;
  int second_strongest_radar = NOTHING_DETECTED;
  find_strongest_radars();
  
  if (strongest_radar != NOTHING_DETECTED) {
    //*direction = interpolate_direction_from_radars();
    return true;    
  }
  return false;
};

void measure_direction_metrics() {
  for(int r=0; r < N_RADAR; r++) {
    int sensor_value = analogRead(RADAR_PIN[r]);
    int gradient = sensor_value - last_value[r];
    unsigned int abs_gradient = abs(gradient);

    METRIC_integrated_absolute_gradient[r] += abs_gradient;
    if (sensor_value > THRESHOLD_absolute_over_threshold) {
      METRIC_absolute_over_threshold[r]++;
    }
        
    last_value[r] = sensor_value;
  }
}

void find_strongest_radars() {
  for (int r=0; r < N_RADAR; r++) {
    // in case of these being shitty, try different one(s)
    int metric = METRIC_integrated_absolute_gradient[r];
    int threshold = THRESHOLD_integrated_absolute_gradient;

    if (metric > threshold) {
      metric_points[r]++;
    }
  }

  for (int r=0; r < N_RADAR; r++) {
    if (metric_points[r] > metric_points[strongest_radar]) {
      second_radar = strongest_radar;
      strongest_radar = r;
    }
  }
}
// radar data interpretation not done here, done by server
/*
int interpolate_direction_from_radars() {
  int strongest_direction = HEAD_DIRECTION[strongest_radar];
  if (second_radar == NOTHING_DETECTED) {
    return strongest_direction;
  }
  int second_direction = HEAD_DIRECTION[second_radar];
  int topmost_points = metric_points[strongest_radar];
  int secondmost_points = metric_points[second_radar];
  float weight = (float)(topmost_points) / (float)(topmost_points + secondmost_points);

  return (int)(weight * strongest_direction + (1.0 - weight) * second_direction);
}
*/
void reset_direction_metrics() {
  for (int r=0; r < N_RADAR; r++) {
    METRIC_integrated_absolute_gradient[r] = 0;
    METRIC_absolute_over_threshold[r] = 0;
    metric_points[r] = 0;
  }
}
