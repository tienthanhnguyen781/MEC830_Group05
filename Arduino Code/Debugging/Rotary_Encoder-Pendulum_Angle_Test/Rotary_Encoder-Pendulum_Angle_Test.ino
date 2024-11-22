#define encoderPinA 18  // Interrupt pin
#define encoderPinB 19  // Interrupt pin

volatile long encoderPosition = 0;
volatile byte lastEncoded = 0;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  Serial.begin(9600);
}

void loop() {
  Serial.println(encoderPosition);
  delay(100);
}

void updateEncoder() {
  bool MSB = digitalRead(encoderPinA);
  bool LSB = digitalRead(encoderPinB);

  byte encoded = (MSB << 1) | LSB;
  byte sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPosition--;

  lastEncoded = encoded;
}
