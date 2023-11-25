// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

#define encoder_phaseA 3  // Interrupt 
#define encoder_phaseB 5  

unsigned int encoder_counter = 0;
String encoder_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;
double wheel_meas_vel = 0.0;    // rad/s

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);

  pinMode(encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_phaseA), encoderCallback, RISING);
}

void loop() {
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    wheel_meas_vel = (10 * encoder_counter * (60.0/385.0)) * 0.10472;

    String encoder_read = "r" + encoder_sign + String(wheel_meas_vel);
    Serial.println(encoder_read);
    last_millis = current_millis;
    encoder_counter = 0;

    analogWrite(L298N_enA, 100);
  }
}

void encoderCallback()
{
  if(digitalRead(encoder_phaseB) == HIGH)
  {
    encoder_sign = "p";
  }
  else
  {
    encoder_sign = "n";
  }
  encoder_counter++;
}