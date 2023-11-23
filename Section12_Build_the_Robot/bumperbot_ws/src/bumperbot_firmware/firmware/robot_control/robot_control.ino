#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11  // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;             // 0-255
double left_wheel_cmd = 0.0;              // 0-255
// Tuning
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Mo tor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/385.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/385.0)) * 0.10472;
    
    rightMotor.Compute();
    leftMotor.Compute();

    // Ignore commands smaller than inertia
    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "p";
  }
  else
  {
    right_wheel_sign = "n";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "n";
  }
  else
  {
    left_wheel_sign = "p";
  }
  left_encoder_counter++;
}
