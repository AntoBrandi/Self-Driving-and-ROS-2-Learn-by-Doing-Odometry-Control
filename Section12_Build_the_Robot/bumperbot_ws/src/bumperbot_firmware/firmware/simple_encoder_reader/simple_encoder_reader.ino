// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;    // rad/s

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
}

void loop() {
  right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/385.0)) * 0.10472;
  String encoder_read = "r" + right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_read);
  right_encoder_counter = 0;
  analogWrite(L298N_enA, 100);
  delay(100);
}

void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";
  }
  else
  {
    right_encoder_sign = "n";
  }
  right_encoder_counter++;
}
