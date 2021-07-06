// comment out for arduino
// #define ESP32

// comment out to reduce Serial communication
#define DEBUG_SERVO
// #define DEBUG_RADAR

#ifdef ESP32
  #include <ESP32Servo.h>
#else
  #include <Arduino.h>
  #include <Servo.h>
#endif

#define SERIAL_BAUD 115200

// AFTER WIRING, SET THESE: /////////////////////////////

#define N_SERVO 5
Servo surfo[N_SERVO];
int SURFO_PIN[N_SERVO] = { 2, 3, 4, 5, 6 };

#define N_RADAR 5
#define NO_PIN 0
#define ENABLE_RADAR_EMULATE true // this enables the EMULATE_RADAR message
int RADAR_PIN[N_RADAR] = { NO_PIN, NO_PIN, A0, NO_PIN, NO_PIN };
int metric_points[N_RADAR] = { 0 };
bool lets_emulate = false;

#define RADAR_HISTORY_N 20

// TODO: define a map for the EYES / FOG pins, or see whether they can be derived from the Message enum

///////////////////////////////////////////

// head: azimuth in horizontaler ebene (90° mittig); wings: vertikaler winkel, (0° nach unten, 90° horizontal)
enum Message {
  IDLE = 0,
  BODY_TILT = 1,
  WINGS = 2,
  HEAD_TILT = 3,
  HEAD_ROTATE = 4,
  BEAK = 5,
  ENVELOPE = 17,
  EYES = 20, // two eyes?
  FOG = 23,
  IS_ALIVE = 63,
  EMULATE_RADARS = 101,
  DEACTIVATE = 125,
  REACTIVATE = 126,
  RESET = 127,
};

int MESSAGE_FOR_SERVO[N_SERVO] = { Message::BODY_TILT, Message::WINGS, Message::HEAD_TILT, Message::HEAD_ROTATE, Message::BEAK };

unsigned short message_action;
int message_body;
char message[3]; // first byte is Message (see enum above), last two bytes are the value information (payload)
bool deactivated = false;

bool listen_for_message();
int translate_to_servo_position(unsigned short action, int message_body);
void execute();
void execute_set_servo(int target, int payload);
void execute_set_switch(int target, bool payload);
void measure_direction_metrics();
bool calculate_metric_points();
void reset_direction_metrics();
bool emulation_was_triggered();

void setup() {
  #ifdef ESP32
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
  #endif

  for (int s=0; s < N_SERVO; s++) {
    #ifdef ESP32
        surfo[s].setPeriodHertz(50);
    #endif
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

  pinMode(LED_BUILTIN, OUTPUT);
}

int step = -1;

void loop() {
    step++;
    if (step > 10000) {
        step -= 10000;
    }

    measure_direction_metrics();

    if (step % RADAR_HISTORY_N == 0 && calculate_metric_points()) {
        Serial.print("RADAR!");
        for (int r=0; r < N_RADAR; r++) {
            Serial.print(metric_points[r]);
            Serial.print(";");
        }
        Serial.println("");
        reset_direction_metrics();
    }

    if (listen_for_message()) {
        execute();
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

inline float constrained_map(int x, int old_lower, int old_upper, int new_lower, int new_upper) {
    const int result = map(message_body, old_lower, old_upper, new_lower, new_upper);
    return (float)constrain(result, min(new_lower, new_upper), max(new_lower, new_upper));
}
#define ACTUAL_HEAD_TILT_CENTER 450
// exceeding the Topy limits will probably kill the PapaGuy!!
int translate_to_servo_position(unsigned short action, int message_body) {
    float result;
    switch (action) {
        case Message::BODY_TILT:
            result = constrained_map(message_body, 0, 1023, 500, 280);
            break;
        case Message::WINGS:
            result = constrained_map(message_body, 0, 1023, 500, 1100);
            break;
        case Message::HEAD_TILT:
            if (message_body < 512) {
                result = constrained_map(message_body, 0, 511, 850, ACTUAL_HEAD_TILT_CENTER);
            } else {
                result = constrained_map(message_body, 512, 1023, ACTUAL_HEAD_TILT_CENTER, 100);
            }
            break;
        case Message::BEAK:
            result = constrained_map(message_body, 0, 1023, 180, 800);
            break;
        default:
            result = (float)constrain(message_body, 0, 1023);
    }

    // now only convert to the units of our Servo library
    return (int)((result / 1023.) * 180.);
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
    case Message::BODY_TILT:
    case Message::WINGS:
    case Message::HEAD_TILT:
    case Message::HEAD_ROTATE:
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
      execute_set_servo(Message::WINGS, message_body);

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

void execute_set_servo(int message, int payload) {
  int index;
  for(;index < N_SERVO && MESSAGE_FOR_SERVO[index] == message; index++);

  if (index >= N_SERVO || !surfo[index].attached()) {
    return;
  }
  int value = translate_to_servo_position(message, payload);
  int old_value = surfo[index].read();
  if (old_value != value) {
    surfo[index].write(value);
  }

  #ifdef DEBUG_SERVO
    for (int s = 0; s < N_SERVO - 1; s++) {
        Serial.print(surfo[s].read());
        Serial.print(", ");
    }
    Serial.println(surfo[N_SERVO - 1].read());
  #endif
}

void execute_set_switch(int target, bool payload) {
  // TODO: SET PIN ACCORDINGLY (might need map)
}

int _radar_history[N_RADAR * RADAR_HISTORY_N] = {0};
#define radar_history(radar, step) _radar_history[radar * RADAR_HISTORY_N + step]
#define oldest_radar_value(radar) _radar_history[radar * RADAR_HISTORY_N + RADAR_HISTORY_N - 1]
float radar_average[N_RADAR] = {0};
int radar_average_n[N_RADAR] = {0};
long duration_of_signal[N_RADAR] = {0};
bool currently_registering_something[N_RADAR] = {false};
#define IGNORE_DEVIATION_FROM_AVERAGE 20
#define MAX_GRADIENT_AT_POSSIBLE_END_OF_SIGNAL 2

void measure_direction_metrics() {
  for (int r = 0; r < N_RADAR; r++) {
    int pin = RADAR_PIN[r];
    if (pin == NO_PIN) {
        continue;
    }

    for (int h = 1; h < RADAR_HISTORY_N; h++) {
        radar_history(r, h) = radar_history(r, h - 1);
    }
    int new_value = analogRead(pin);
    radar_history(r, 0) = new_value;

    // somehow deal with overflowing of the index, I assume one could do this
    if (radar_average_n[r] == 30000) {
        radar_average_n[r] = 1000;
    } else {
        radar_average_n[r]++;
    }
    radar_average[r] = (radar_average_n[r] * radar_average[r] + new_value) / (radar_average_n[r] + 1);

    int gradient = new_value - oldest_radar_value(r);
    unsigned int abs_gradient = abs(gradient);

    float deviation = abs(new_value - radar_average[r]);
    if (deviation > IGNORE_DEVIATION_FROM_AVERAGE) {
        if (currently_registering_something[r]) {
            duration_of_signal[r]++;
        } else {
            currently_registering_something[r] = true;
            duration_of_signal[r] = 1;
        }
    } else if (currently_registering_something[r]) {
        if (abs_gradient < MAX_GRADIENT_AT_POSSIBLE_END_OF_SIGNAL) {
            currently_registering_something[r] = false;
        }
    }

    // below here, there's only debug output for the first radar
    if (r != 0) {
        continue;
    }

    digitalWrite(LED_BUILTIN, deviation > IGNORE_DEVIATION_FROM_AVERAGE ? HIGH : LOW);

    #ifdef DEBUG_RADAR
        Serial.print(new_value);
        Serial.print(", ");
        Serial.print(gradient);
        Serial.print(", ");
        Serial.print(deviation);
        Serial.print(", ");
        Serial.print(currently_registering_something[r]);
        Serial.print(", ");
        Serial.print(duration_of_signal[r]);
        Serial.print(", ");
        Serial.println(metric_points[r]);
    #endif
  }
}

bool calculate_metric_points() {
    if (emulation_was_triggered()) {
        return true;
    }

    bool any_point_found = false;
    for (int r = 0; r < N_RADAR; r++) {
        if (RADAR_PIN[r] == NO_PIN) {
            continue;
        }
        if (!currently_registering_something[r] && duration_of_signal[r] > 0) {
            metric_points[r] += duration_of_signal[r];
            duration_of_signal[r] = 0;
            any_point_found = true;
        }
    }
    return any_point_found;
}

void reset_direction_metrics() {
  for (int r = 0; r < N_RADAR; r++) {
    duration_of_signal[r] = 0;
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
