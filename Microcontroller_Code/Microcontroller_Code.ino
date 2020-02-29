// Motor driver pins
#define motor_1_A 5
#define motor_1_B 6
#define motor_2_A 7
#define motor_2_B 8

void setup() {
  // set motor pins as output
  pinMode(motor_1_A, OUTPUT);
  pinMode(motor_1_B, OUTPUT);
  pinMode(motor_2_A, OUTPUT);
  pinMode(motor_2_B, OUTPUT);
} // end of setup method

void loop() {

} // end of loop method

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
} // end of set_motor method
