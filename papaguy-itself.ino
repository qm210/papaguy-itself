#include <ESP32Servo.h>

#define SERIAL_BAUD 115200

// SET THESE: /////////////////////////////

#define N_SERVO 1
Servo surfo[N_SERVO];
int SURFO_PIN[N_SERVO] = { 18 };

#define N_RADAR 5
#define NO_PIN 0
#define ENABLE_RADAR_EMULATE true // if that is true, the NO_PINs will get random values
int RADAR_PIN[N_RADAR] = { NO_PIN, NO_PIN, NO_PIN, NO_PIN, NO_PIN };
int metric_points[N_RADAR] = {0};
bool lets_emulate = false;

// TODO: define a map for the EYES / FOG pins, or see whether they can be derived from the Message enum

///////////////////////////////////////////

// head: azimuth in horizontaler ebene (90° mittig); wings: vertikaler winkel, (0° nach unten, 90° horizontal)
enum Message {
  IDLE = 0,
  HEAD = 1,
  WING_LEFT = 2,
  WING_RIGHT = 3,
  BEAK = 4,
  ENVELOPE = 17,
  EYES = 20, // two eyes?
  FOG = 23,
  IS_ALIVE = 63,
  EMULATE_RADARS = 101,
  DEACTIVATE = 125,
  REACTIVATE = 126,
  RESET = 127,
};

unsigned short message_action;
int message_body;
char message[3]; // first byte is Message (see enum above), last two bytes are the value information (payload)
bool deactivated = false;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int s=0; s < N_SERVO; s++) {
    surfo[s].setPeriodHertz(50);
    surfo[s].attach(SURFO_PIN[s], 544, 2400); // pin number, min, max microsecond settings for PWM
  }

  for (int r=0; r < N_RADAR; r++) {
    if (RADAR_PIN[r] != NO_PIN) {
      pinMode(RADAR_PIN[r], INPUT);
    }
  }

  Serial.begin(SERIAL_BAUD);
  Serial.println("PapaGuy is listening.");

  reset_direction_metrics();
}

#define CHECK_RADAR_EVERY 5 // only to reduce load a little
int step = 0;

void loop() {

  if (step % CHECK_RADAR_EVERY == 0) {
    measure_direction_metrics();

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

  if (listen_for_message()) {
    execute();
  }

  step++;
  if (step > 10000) {
    step -= 10000;
  }
}

void(* reboot) (void) = 0;

bool listen_for_message() {
  if (Serial.available() == 0) {
    message_action = Message::IDLE;
    return false;
  }
  Serial.readBytes(message, 3);
  message_action = message[0];
  message_body = message[2] | message[1] << 8;
  Serial.print("MESSAGE: ");
  Serial.print(message_action);
  Serial.print(";");
  Serial.println(message_body);
  return message_action != Message::IDLE;
}

// need to adjust this if you use a different Servo library!
int servo_state_from(int message_body) {
    return (int)(((float)message_body / 1024.) * 180.);
}

#define ENVELOPE_LIGHT_THRESHOLD (0.6 * 1023)

void execute() {
  if (message_action == Message::REACTIVATE) {
    deactivated = false;
  } else if (message_action == Message::RESET) {
    reboot();
  }

  if (deactivated) {
    return;
  }

  switch (message_action) {
    case Message::HEAD:
    case Message::WING_LEFT:
    case Message::WING_RIGHT:
    case Message::BEAK:
      execute_set_servo(message_action, message_body);
      return;

    case Message::EYES:
    case Message::FOG:
      execute_set_switch(message_action, message_body > 0);

    case Message::ENVELOPE:
      // FOR DEBUG (e.g. if you have only one servo...)
      // execute_set_servo(1, message_body);

      execute_set_servo(Message::BEAK, message_body);
      execute_set_servo(Message::WING_LEFT, message_body);
      execute_set_servo(Message::WING_RIGHT, message_body);

      execute_set_switch(Message::EYES, message_body > ENVELOPE_LIGHT_THRESHOLD);
      return;

    case Message::EMULATE_RADARS:
      lets_emulate = ENABLE_RADAR_EMULATE;
      return;

    case Message::DEACTIVATE:
      deactivated = true;
      return;

    case Message::IS_ALIVE:
      Serial.println("PapaGuy is listening, yes.");
      return;

    case Message::IDLE:
      return;

    default:
      Serial.print("UNKNOWN MESSAGE: ");
      Serial.println(message_action);
      return;
  };
}

void execute_set_servo(int target, int payload) {
  int index = target - 1;
  int value = servo_state_from(payload);
  if (index >= N_SERVO || !surfo[index].attached()) {
    return;
  }
  int old_value = surfo[index].read();
  if (old_value != value) {
    surfo[index].write(value);
  }
}

void execute_set_switch(int target, bool payload) {
  // TODO: SET PIN ACCORDINGLY (might need map)
}

int last_value[N_RADAR] = {0};
int METRIC_integrated_absolute_gradient[N_RADAR] = {0};
int THRESHOLD_integrated_absolute_gradient = 2000;
int METRIC_absolute_over_threshold[N_RADAR] = {0};
int THRESHOLD_absolute_over_threshold = 1000;
int THRESHOLD_absolute_over_threshold_times = 10;

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
  if (emulation_was_triggered()) {
    return true;
  }

  bool any_point_found = false;
  for (int r=0; r < N_RADAR; r++) {
    if (RADAR_PIN[r] == NO_PIN) {
      continue;
    }
    // in case of these being shitty, try different one(s)
    int metric = METRIC_integrated_absolute_gradient[r];
    int threshold = THRESHOLD_integrated_absolute_gradient;

    Serial.print("METRIC ");
    Serial.print(metric);
    Serial.print("; THRESHOLD ");
    Serial.println(threshold);

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
    lets_emulate = false;
  }
}

bool emulation_was_triggered() {
  if (!lets_emulate) {
    return false;
  }

  bool any_point_found = false;
  int r = random(100);
  if (r < N_RADAR) {
    metric_points[r] += random(1, 10);
    if (r > 1) {
      metric_points[r - 1] += random(0, 4);
    }
    if (r < N_RADAR - 1) {
      metric_points[r + 1] += random(0, 4);
    }
    any_point_found = true;
  }
  return any_point_found;
}
