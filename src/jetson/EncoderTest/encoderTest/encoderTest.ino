/*
 * Encoder example sketch
 * by Andrew Kramer
 * 1/1/2016
 *
 * Records encoder ticks for each wheel
 * and prints the number of ticks for
 * each encoder every 500ms
 *
 */

// pins for the encoder inputs
#define RH_ENCODER_A 2 
#define RH_ENCODER_B 3

// variables to store the number of encoder pulses
// for each motor
volatile signed long rightCount = 0;

void setup() {
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  
  // initialize hardware interrupts
  attachInterrupt(0, rightEncoderEvent, CHANGE);  
  attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  delay(50);
}


// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}
