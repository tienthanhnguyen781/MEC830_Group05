//HMI Var
// Define pin numbers for buttons and LEDs
const int ESTOP_BUTTON_PIN = 31;
const int RESET_BUTTON_PIN = 33;
const int PID_BUTTON_PIN = 35;
const int POLEPLACEMENT_BUTTON_PIN = 29;

const int ESTOP_LED_PIN = 36;
const int RESET_LED_PIN = 38;
const int PID_LED_PIN = 42;
const int POLEPLACEMENT_LED_PIN = 40;

// Variable to store button states
bool eStopActivated = false;
bool resetActivated = false;
bool pidActivated = false;
bool polePlacementActivated = false;

// Variables to track if LEDs should stay on
bool eStopLedOn = false;
bool resetLedOn = false;
bool pidLedOn = false;
bool polePlacementLedOn = false;


//Hardware Stuff
/**
 * == Inverted Pendulum Stabilization using PID Control ==
 * 
 * == Hardware Specification ==
 * - Motor Driver: BTS7960 to drive XD775 DC motor
 * - Cart Configuration: Cart on linear rails with encoder mounted on idler side using a pulley
 * - Encoders: Two rotary encoders with 600 PPR
 *   - Cart Encoder: Mounted on idler pulley (measures cart position)
 *   - Pendulum Encoder: Mounted on pendulum pivot (measures pendulum angle)
 * - PID Button: Pushbutton connected to pin 35 (used for zero position calibration)
 * - PolePlacement Button: Pushbutton connected to pin 29 (used for zero position calibration for pole placement controller)
 * - Emergency Stop Button: Pushbutton connected to pin 39 (used to immediately cut off power)
 * - Reset Button: Pushbutton connected to pin 33 (used to reset the system after emergency stop)
 * - LED Indicator: LED connected to pin 36 (lights up when emergency stop is activated)
 */

#include <Arduino.h>

// === Pin Definitions ==========================================================================

// Cart encoder pins (adjust if necessary)
const uint8_t CART_ENCODER_A = 2;  // Cart encoder output A
const uint8_t CART_ENCODER_B = 3;  // Cart encoder output B

// Pendulum encoder pins (adjust if necessary)
const uint8_t PENDULUM_ENCODER_A = 18;  // Pendulum encoder output A
const uint8_t PENDULUM_ENCODER_B = 19;  // Pendulum encoder output B


// BTS7960 motor driver pins (adjust if necessary)
const uint8_t RPWM_PIN = 9;   // Right PWM input of BTS7960
const uint8_t LPWM_PIN = 10;  // Left PWM input of BTS7960
const uint8_t R_EN_PIN = 7;   // Right enable pin of BTS7960
const uint8_t L_EN_PIN = 8;   // Left enable pin of BTS7960

// === System Constants ==========================================================================

// Encoder pulses per revolution (PPR)
const int ENCODER_PPR = 600;  // Encoder PPR as per datasheet

// If using quadrature encoding, counts per revolution (CPR) is 4 times PPR
const int COUNTS_PER_REVOLUTION = ENCODER_PPR * 4;

// Physical constants
const float PULLEY_RADIUS = 0.01f;                             // Pulley radius in meters (adjust to match your pulley)
const float POSITION_LIMIT = 0.35f;                            // Maximum cart position in meters (adjust based on your rail length)
const float PULLEY_CIRCUMFERENCE = 2.0f * PI * PULLEY_RADIUS;  // Precomputed circumference

// Control thresholds
const float THETA_THRESHOLD = PI / 6.0f;  // Maximum allowable pendulum angle (30 degrees in radians)
const float PI2 = 2.0f * PI;              // Constant for 2*pi

// === Encoder Variables ==========================================================================

volatile long cartEncoderValue = 0L;   // Cart encoder count
volatile uint8_t lastCartEncoded = 0;  // Last encoded value for cart encoder

volatile long pendulumEncoderValue = 0L;   // Pendulum encoder count
volatile uint8_t lastPendulumEncoded = 0;  // Last encoded value for pendulum encoder

long pendulumZeroCount = 0L;  // Encoder count when pendulum is upright (zero position)

// === Timing Variables ==========================================================================

unsigned long lastTimeMicros = 0L;  // Last time stamp in microseconds
float dt = 0.0f;                    // Delta time in seconds

// Control loop timing
unsigned long lastLoopTime = 0L;                  // Last control loop time
const unsigned long LOOP_INTERVAL_MICROS = 5000;  // Control loop interval in microseconds (5 ms)

// Logging timing
unsigned long lastLogTime = 0L;             // Last logging time
const unsigned long LOG_INTERVAL_MS = 100;  // Logging interval in milliseconds

// === State Variables ==========================================================================

// Cart state
float x = 0.0f;       // Cart position in meters
float last_x = 0.0f;  // Previous cart position
float v = 0.0f;       // Cart velocity in meters per second

// Pendulum state
float theta = 0.0f;       // Pendulum angle in radians
float last_theta = 0.0f;  // Previous pendulum angle
float w = 0.0f;           // Pendulum angular velocity in radians per second

// === PID Control Variables ==========================================================================

// PID gains (to be tuned)
float Kp_theta = 18.0f;  // Proportional gain for pendulum angle
float Ki_theta = 0.5f;   // Integral gain for pendulum angle
float Kd_theta = 0.2f;   // Derivative gain for pendulum angle

float Kp_x = 0.1f;  // Proportional gain for cart position
float Ki_x = 0.0f;  // Integral gain for cart position
float Kd_x = 0.1f;  // Derivative gain for cart position

// PID terms
float error_theta = 0.005f;     // Error between desired and actual angle
float last_error_theta = 0.0f;  // Previous error for pendulum angle
float integral_theta = 0.0f;    // Integral of error for pendulum angle
float derivative_theta = 0.0f;  // Derivative of error for pendulum angle

float error_x = 25.0f;       // Error between desired and actual cart position
float last_error_x = 0.0f;  // Previous error for cart position
float integral_x = 0.0f;    // Integral of error for cart position
float derivative_x = 0.0f;  // Derivative of error for cart position

// Control variables
float control_theta = 0.0f;  // Control signal for pendulum angle
float control_x = 0.0f;      // Control signal for cart position
float control = 0.0f;        // Combined control signal
float u = 0.0f;              // Control signal mapped to PWM value

// Pole Placement gains (to be tuned)
float K1 = -0.5f;  // Gain for cart position
float K2 = -1.0f;  // Gain for cart velocity
float K3 = -2.0f;  // Gain for pendulum angle
float K4 = -2.5f;   // Gain for pendulum angular velocity

// === Function Prototypes ==========================================================================

void cartEncoderHandler();                // Interrupt handler for cart encoder
void pendulumEncoderHandler();            // Interrupt handler for pendulum encoder
float saturate(float v, float maxValue);  // Function to limit a value to a maximum
float getAngle(long pulses);              // Function to calculate pendulum angle from encoder pulses
float getCartPosition(long pulses);       // Function to calculate cart position from encoder pulses
void driveMotor(float u);                 // Function to drive motor based on control signal
boolean isControllable(float theta);      // Function to check if pendulum is within controllable angle
void log_state(float control, float u);   // Function to log system state for debugging
void emergencyStop();                     // Function to handle emergency stop
void resetSystem();                       // Function to reset the system after emergency stop
void polePlacementControl();              // Function to handle pole placement control

// === Encoder Lookup Table for Efficient Quadrature Decoding ==========================================================================

const int8_t encoderLookupTable[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

// === Setup Function ===================================================================================================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure cart encoder pins as inputs with pull-up resistors
  pinMode(CART_ENCODER_A, INPUT_PULLUP);
  pinMode(CART_ENCODER_B, INPUT_PULLUP);

  // Configure pendulum encoder pins as inputs with pull-up resistors
  pinMode(PENDULUM_ENCODER_A, INPUT_PULLUP);
  pinMode(PENDULUM_ENCODER_B, INPUT_PULLUP);

  // Configure PID button pin as input with pull-up resistor
  pinMode(PID_BUTTON_PIN, INPUT_PULLUP);  // Using internal pull-up resistor

  // Configure PolePlacement button pin as input with pull-up resistor
  pinMode(POLEPLACEMENT_BUTTON_PIN, INPUT_PULLUP);  // Using internal pull-up resistor

  // Configure emergency stop button pin as input with pull-up resistor
  pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);  // Using internal pull-up resistor

  // Configure reset button pin as input with pull-up resistor
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);  // Using internal pull-up resistor

  // Configure emergency stop LED pin as output
  pinMode(ESTOP_LED_PIN, OUTPUT);
  digitalWrite(ESTOP_LED_PIN, LOW);  // Initially turn off the LED

  // Attach interrupts for cart encoder (quadrature decoding)
  attachInterrupt(digitalPinToInterrupt(CART_ENCODER_A), cartEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CART_ENCODER_B), cartEncoderHandler, CHANGE);

  // Attach interrupts for pendulum encoder (quadrature decoding)
  attachInterrupt(digitalPinToInterrupt(PENDULUM_ENCODER_A), pendulumEncoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PENDULUM_ENCODER_B), pendulumEncoderHandler, CHANGE);

  // Configure motor driver pins as outputs
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  // Enable the motor driver by setting enable pins HIGH
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  // Initialize motor outputs to zero (motor stopped)
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  // Initialize timing
  lastTimeMicros = micros();
  lastLoopTime = micros();
  lastLogTime = millis();

  // Initialize PID variables
  integral_theta = 0.0f;
  last_error_theta = 0.0f;
  integral_x = 0.0f;
  last_error_x = 0.0f;

  // Wait for pendulum to be placed upright for zero calibration using either PID or PolePlacement button
  Serial.println("Place the pendulum in the upright position and press the calibration button to calibrate zero position.");

  // Set button pins as input with internal pullup resistors
  pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PID_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POLEPLACEMENT_BUTTON_PIN, INPUT_PULLUP);

  // Set LED pins as output
  pinMode(ESTOP_LED_PIN, OUTPUT);
  pinMode(RESET_LED_PIN, OUTPUT);
  pinMode(PID_LED_PIN, OUTPUT);
  pinMode(POLEPLACEMENT_LED_PIN, OUTPUT);

  // Start with all LEDs off
  digitalWrite(ESTOP_LED_PIN, LOW);
  digitalWrite(RESET_LED_PIN, LOW);
  digitalWrite(PID_LED_PIN, LOW);
  digitalWrite(POLEPLACEMENT_LED_PIN, LOW);
}

void loop() {

  // Read button states (active low)
  eStopActivated = !digitalRead(ESTOP_BUTTON_PIN);
  resetActivated = !digitalRead(RESET_BUTTON_PIN);
  pidActivated = !digitalRead(PID_BUTTON_PIN);
  polePlacementActivated = !digitalRead(POLEPLACEMENT_BUTTON_PIN);

  // E-Stop functionality: Cutoff power, turn on the E-Stop LED, and turn off all other LEDs
  if (eStopActivated) {
    eStopLedOn = true;
    resetLedOn = false;
    pidLedOn = false;
    polePlacementLedOn = false;
    digitalWrite(PID_LED_PIN, LOW);
    digitalWrite(POLEPLACEMENT_LED_PIN, LOW);
    // Stop the motor immediately
    driveMotor(0);

    // Disable the motor driver by setting enable pins LOW
    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);

    // Print emergency stop message
    Serial.println("EMERGENCY STOP ACTIVATED! System halted.");
  }

  // Reset functionality: Turns off the E-Stop LED and restores power if pressed
  if (resetActivated) {
    eStopLedOn = false;
    resetLedOn = true;
    digitalWrite(RESET_LED_PIN, HIGH);
    // Restore power to the system logic here
    // Re-enable motor driver
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);
    // Reset variables
    eStopActivated = false;
    integral_theta = 0.0f;
    last_error_theta = 0.0f;
    integral_x = 0.0f;
    last_error_x = 0.0f;
    last_x = x;
    last_theta = theta;

    // Optionally, re-calibrate zero position if needed
    // Uncomment the following lines if you want to re-calibrate
    Serial.println("Re-calibrating zero position. Place the pendulum upright and press the calibration button.");
    while (digitalRead(RESET_BUTTON_PIN) == HIGH) {
      // Wait for calibration button press
    }
    delay(50);
    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
      // Wait for calibration button release
    }
    pendulumZeroCount = pendulumEncoderValue;
    cartEncoderValue = 0L;

    Serial.println("Pendulum zero position re-calibrated.");

    Serial.println("System reset. Resuming operation.");
  }

  // Check for E-Stop activation at any point
  if (eStopActivated) {
    eStopLedOn = true;
    resetLedOn = false;
    pidLedOn = false;
    polePlacementLedOn = false;
    digitalWrite(PID_LED_PIN, LOW);
    digitalWrite(POLEPLACEMENT_LED_PIN, LOW);
    digitalWrite(RESET_LED_PIN, LOW);
    // Stop the motor immediately
    driveMotor(0);

    // Disable the motor driver by setting enable pins LOW
    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);

    // Print emergency stop message
    Serial.println("EMERGENCY STOP ACTIVATED! System halted.");
  }

  // PID functionality: Turns on the PID LED if pressed and turns off Pole Placement LED
  if (pidActivated && !eStopLedOn) {
    pidLedOn = true;
    polePlacementLedOn = false;
  }

  // Pole Placement functionality: Turns on the Pole Placement LED if pressed and turns off PID LED
  if (polePlacementActivated && !eStopLedOn) {
    runPolePlacementFunction();
    polePlacementLedOn = true;
    pidLedOn = false;
  }

  // Run PID loop continuously if PID LED is on
  if (pidLedOn) {
    runPIDFunction();
  }

  // Update E-Stop, Reset, PID, and Pole Placement LED states
  digitalWrite(ESTOP_LED_PIN, eStopLedOn ? HIGH : LOW);
  digitalWrite(RESET_LED_PIN, resetLedOn ? HIGH : LOW);
  digitalWrite(PID_LED_PIN, pidLedOn ? HIGH : LOW);
  digitalWrite(POLEPLACEMENT_LED_PIN, polePlacementLedOn ? HIGH : LOW);
}
// === Function Definitions ===

void runPIDFunction() {
  // Control loop timing
  unsigned long now = micros();
  if (now - lastLoopTime >= LOOP_INTERVAL_MICROS) {
    dt = (now - lastTimeMicros) / 1000000.0f;  // Convert microseconds to seconds
    lastTimeMicros = now;
    lastLoopTime = now;

    // --- Update Cart State ---

    // Calculate cart position (x) in meters
    x = getCartPosition(cartEncoderValue);  // Small correction to counter left drift

    // Calculate cart velocity (v) in meters per second
    v = (x - last_x) / dt;

    // --- Update Pendulum State ---

    // Calculate pendulum angle (theta) in radians
    theta = getAngle(pendulumEncoderValue);

    // Calculate pendulum angular velocity (w) in radians per second
    w = (theta - last_theta) / dt;

    // --- PID Control Logic for Pendulum Angle ---

    // Calculate error between desired angle (0.0 for upright) and current angle
    error_theta = theta;  // Desired angle is 0

    // Update integral term with current error
    integral_theta = saturate(integral_theta + error_theta * dt, 5.0f);

    // Calculate derivative term (rate of change of error)
    derivative_theta = (error_theta - last_error_theta) / dt;

    // Compute control signal for pendulum angle using PID formula
    control_theta = (Kp_theta * error_theta) + (Ki_theta * integral_theta) + (Kd_theta * derivative_theta);

    // --- PID Control Logic for Cart Position ---

    // Calculate error between desired cart position (0.0 for center) and current position
    error_x = -x;  // Desired position is 0 (center)

    // Update integral term with current error
    integral_x = saturate(integral_x + error_x * dt, 2.0f);

    // Calculate derivative term (rate of change of error)
    derivative_x = 0.8f * derivative_x + 0.2f * ((error_x - last_error_x) / dt);

    // Compute control signal for cart position using PID formula
    control_x = (Kp_x * error_x) + (Ki_x * integral_x) + (Kd_x * derivative_x);

    // --- Combine Control Signals ---

    // Combine cart position and pendulum angle control signals
    control = control_theta + control_x;

    // Limit control signal to maximum allowable voltage (e.g., ±12V)
    control = saturate(control, 12.0f);  // Adjust the limit based on your motor's voltage

    // Map control signal to PWM value (-254 to 254)
    u = (254.0f * control) / 12.0f;  // Add damping factor  // Scale control signal to PWM range

    // Invert the control signal if motor moves in the wrong direction
    // Uncomment the following line if needed
    // u = -u;

    // --- Motor Control ---

    // Apply control signal if system is within controllable range
    if (isControllable(theta) && fabs(x) < POSITION_LIMIT) {
      driveMotor(u);  // Drive motor with control signal
    } else {
      driveMotor(0);          // Stop motor if out of control range
      integral_theta = 0.0f;  // Reset integral term to prevent windup
      integral_x = 0.0f;      // Reset integral term to prevent windup
      eStopActivated = true;
      eStopLedOn = true;
      resetLedOn = false;
      pidLedOn = false;
      polePlacementLedOn = false;
      digitalWrite(PID_LED_PIN, LOW);
      digitalWrite(POLEPLACEMENT_LED_PIN, LOW);
      // Stop the motor immediately
      driveMotor(0);

      // Disable the motor driver by setting enable pins LOW
      digitalWrite(R_EN_PIN, LOW);
      digitalWrite(L_EN_PIN, LOW);

      // Print emergency stop message
      Serial.println("EMERGENCY STOP ACTIVATED! System halted.");
    }

    // --- Update Previous State Variables ---

    last_x = x;                      // Update last cart position
    last_theta = theta;              // Update last pendulum angle
    last_error_theta = error_theta;  // Update last error for pendulum angle
    last_error_x = error_x;          // Update last error for cart position

    // --- Logging ---

    log_state(control, u);  // Log state variables for debugging
  }
}

/**
 * Interrupt handler for cart encoder
 * Reads the quadrature encoder signals and updates the encoder count
 */
void cartEncoderHandler() {
  // Read the current state of the encoder pins
  uint8_t MSB = digitalRead(CART_ENCODER_A);  // Most significant bit
  uint8_t LSB = digitalRead(CART_ENCODER_B);  // Least significant bit

  // Combine the two bits into a single value
  uint8_t encoded = (MSB << 1) | LSB;

  // Calculate the state transition
  uint8_t sum = (lastCartEncoded << 2) | encoded;

  // Update encoder count based on the lookup table
  cartEncoderValue += encoderLookupTable[sum & 0x0F];

  // Store current encoded value for the next interrupt
  lastCartEncoded = encoded;
}

/**
 * Interrupt handler for pendulum encoder
 * Reads the quadrature encoder signals and updates the encoder count
 */
void pendulumEncoderHandler() {
  // Read the current state of the encoder pins
  uint8_t MSB = digitalRead(PENDULUM_ENCODER_A);  // Most significant bit
  uint8_t LSB = digitalRead(PENDULUM_ENCODER_B);  // Least significant bit

  // Combine the two bits into a single value
  uint8_t encoded = (MSB << 1) | LSB;

  // Calculate the state transition
  uint8_t sum = (lastPendulumEncoded << 2) | encoded;

  // Update encoder count based on the lookup table
  pendulumEncoderValue += encoderLookupTable[sum & 0x0F];

  // Store current encoded value for the next interrupt
  lastPendulumEncoded = encoded;
}

/**
 * Saturate a value to a specified maximum magnitude
 * @param v The input value
 * @param maxValue The maximum allowable magnitude
 * @return The saturated value
 */
float saturate(float v, float maxValue) {
  if (v > maxValue) return maxValue;
  if (v < -maxValue) return -maxValue;
  return v;
}

/**
 * Calculate pendulum angle in radians from encoder pulses
 * @param pulses The encoder pulse count
 * @return The calculated angle in radians
 */
float getAngle(long pulses) {
  // Adjust pulses by subtracting the zero offset
  long adjustedPulses = pulses - pendulumZeroCount;

  // Calculate the angle in radians
  float angle = (PI2 * adjustedPulses) / COUNTS_PER_REVOLUTION;

  // Normalize angle to the range [-PI, PI]
  angle = fmod(angle + PI, PI2) - PI;

  return angle;
}

/**
 * Calculate cart position in meters from encoder pulses
 * @param pulses The encoder pulse count
 * @return The calculated position in meters
 */
float getCartPosition(long pulses) {
  // Calculate linear distance moved based on pulley circumference and encoder pulses
  // Distance = (number of rotations) * (circumference of pulley)
  float distance = (pulses / (float)COUNTS_PER_REVOLUTION) * PULLEY_CIRCUMFERENCE;
  return distance;
}

/**
 * Drive the motor based on the control signal
 * @param u Control signal mapped to PWM value (-254 to 254)
 */
void driveMotor(float u) {
  // Limit PWM value to allowable range
  u = saturate(u, 254.0f);  // 254 is maximum for analogWrite

  int pwm_value = (int)fabs(u);

  if (u > 0.0f) {
    // Forward direction
    analogWrite(RPWM_PIN, pwm_value);  // Set right PWM
    analogWrite(LPWM_PIN, 0);          // Left PWM off
  } else if (u < 0.0f) {
    // Reverse direction
    analogWrite(RPWM_PIN, 0);          // Right PWM off
    analogWrite(LPWM_PIN, pwm_value);  // Set left PWM
  } else {
    // Stop motor
    analogWrite(RPWM_PIN, 0);  // Right PWM off
    analogWrite(LPWM_PIN, 0);  // Left PWM off
  }
}

/**
 * Check if the pendulum is within the controllable angle range
 * @param theta The current pendulum angle in radians
 * @return True if within range, False otherwise
 */
boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;  // Returns true if angle is less than threshold
}

/**
 * Log the system state variables for debugging
 * @param control The control signal before scaling
 * @param u The control signal mapped to PWM value
 */
void log_state(float control, float u) {
  // Control logging frequency
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentTime;

    // Output data with labels for Serial Monitor
    Serial.print("Theta (deg): ");
    Serial.print(theta * 180.0f / PI, 2);  // Pendulum angle in degrees
    Serial.print("\t");

    Serial.print("Angular Velocity (deg/s): ");
    Serial.print(w * 180.0f / PI, 2);  // Angular velocity in degrees/sec
    Serial.print("\t");

    Serial.print("Cart Position (m): ");
    Serial.print(x, 4);  // Cart position in meters
    Serial.print("\t");

    Serial.print("Cart Velocity (m/s): ");
    Serial.print(v, 4);  // Cart velocity in m/s
    Serial.print("\t");

    Serial.print("Control (V): ");
    Serial.print(control, 4);  // Control signal in volts
    Serial.print("\t");

    Serial.print("PWM Output: ");
    Serial.println(u, 4);  // PWM Output
  }
}

void runPolePlacementFunction() {
  // Control loop timing
  unsigned long now = micros();
  if (now - lastLoopTime >= LOOP_INTERVAL_MICROS) {
    dt = (now - lastTimeMicros) / 1000000.0f;  // Convert microseconds to seconds
    lastTimeMicros = now;
    lastLoopTime = now;

    // --- Update Cart State ---

    // Calculate cart position (x) in meters
    x = getCartPosition(cartEncoderValue);

    // Calculate cart velocity (v) in meters per second
    v = (x - last_x) / dt;

    // --- Update Pendulum State ---

    // Calculate pendulum angle (theta) in radians
    theta = getAngle(pendulumEncoderValue);

    // Calculate pendulum angular velocity (w) in radians per second
    w = (theta - last_theta) / dt;

    // --- Pole Placement Control Logic ---

    // Calculate control input using state feedback
    control = K1 * x + K2 * v + K3 * theta + K4 * w;

    // Limit control signal to maximum allowable voltage (e.g., ±12V)
    control = saturate(control, 12.0f);

    // Map control signal to PWM value (-254 to 254)
    u = (254.0f * control) / 12.0f;

    // --- Motor Control ---

    // Apply control signal if system is within controllable range
    if (isControllable(theta) && fabs(x) < POSITION_LIMIT) {
      driveMotor(u);  // Drive motor with control signal
    } else {
      driveMotor(0);          // Stop motor if out of control range
      eStopActivated = true;
      eStopLedOn = true;
      resetLedOn = false;
      pidLedOn = false;
      polePlacementLedOn = false;
      digitalWrite(PID_LED_PIN, LOW);
      digitalWrite(POLEPLACEMENT_LED_PIN, LOW);

      // Stop the motor immediately
      driveMotor(0);

      // Disable the motor driver by setting enable pins LOW
      digitalWrite(R_EN_PIN, LOW);
      digitalWrite(L_EN_PIN, LOW);

      // Print emergency stop message
      Serial.println("EMERGENCY STOP ACTIVATED! System halted.");
    }

    // --- Update Previous State Variables ---

    last_x = x;         // Update last cart position
    last_theta = theta; // Update last pendulum angle

    // --- Logging ---

    log_state(control, u);  // Log state variables for debugging
  }
}
