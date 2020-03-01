// Motor driver pins
#define motor_1_A 5
#define motor_1_B 6
#define motor_2_A 7
#define motor_2_B 8

// Input pins (from RPi)
#define DIG_IN_0 0
#define DIG_IN_1 1
#define PWM_IN A2

// Input from voltage divider
#define battery_pin A4

float bat_voltage;

void setup() {
  // set motor pins as output
  pinMode(motor_1_A, OUTPUT);
  pinMode(motor_1_B, OUTPUT);
  pinMode(motor_2_A, OUTPUT);
  pinMode(motor_2_B, OUTPUT);
} // end of method setup

void loop() {
  // this is the formula to read the battery pack voltage
  bat_voltage = analogRead(battery_pin) * 3.3 / 1024 * 2;
  if (bat_voltage < 4) {
    set_motor(0, 0);
    set_motor(1, 0);
  }
  else {
    // car driving
    if (DIG_IN_0) {
      set_motor(0, PWM_IN);
    }
    else {
      set_motor(0, PWM_IN*-1);
    }
    if (DIG_IN_1) {
      set_motor(1, PWM_IN);
    }
    else {
      set_motor(1, PWM_IN*-1);
    }
  }
} // end of method loop

void set_motor(bool motor_name, int motor_speed) {
  motor_speed = constrain(motor_speed, -255, 255);
  if (motor_name) {
    // left motors
    if (motor_speed > 0) { // forward
      digitalWrite(motor_1_A, LOW);
      analogWrite(motor_1_B, abs(motor_speed));
    }
    else if (motor_speed < 0) { // reverse
      digitalWrite(motor_1_B, LOW);
      analogWrite(motor_1_A, abs(motor_speed));
    }
    else { // zero
      digitalWrite(motor_1_A, LOW);
      digitalWrite(motor_1_B, LOW);
    }
  }
  else {
    // right motors
    if (motor_speed > 0) { // forward
      digitalWrite(motor_2_A, LOW);
      analogWrite(motor_2_B, abs(motor_speed));
    }
    else if (motor_speed < 0) { // reverse
      digitalWrite(motor_2_B, LOW);
      analogWrite(motor_2_A, abs(motor_speed));
    }
    else { // zero
      digitalWrite(motor_2_A, LOW);
      digitalWrite(motor_2_B, LOW);
    }
  }
} // end of method set_motor
