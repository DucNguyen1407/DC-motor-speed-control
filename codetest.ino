#include <LiquidCrystal.h>

// ==========================================
// HARDWARE CONFIGURATION
// ==========================================
#define ENCODER_PIN_A 2 
#define ENCODER_PIN_B 4 

const int PULSES_PER_REVOLUTION = 360;

// L298 DRIVER
const int IN1 = A1;
const int IN2 = A0;
const int ENA = 3;

// ==========================================
// PID & TRAJECTORY GENERATION
// ==========================================
double targetSetpoint = 0.0;
double currentSetpoint = 0.0;
double maxRampStep = 2.0; // 2.0 RPM/20ms = 100 RPM/s

double PV = 0.0;
double filteredPV = 0.0;
double last_PV = 0.0; // Used to calculate derivative on measurement (PV)

// PID PARAMETERS
double Kp = 1.521;
double Ki = 10.045;
double Kd = 0.0165;

double error = 0;
double integral = 0;
double derivative = 0;

// ==========================================
// TIMER + ENCODER
// ==========================================
volatile long pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long T = 20; // Sampling period: 20 ms

// ==========================================
// INTERRUPT SERVICE ROUTINE (ISR)
// ==========================================
void countPulse()
{
    // Inverted logic to synchronize phase with L298, preventing instability at negative setpoints
    if (digitalRead(ENCODER_PIN_B) == HIGH) {
        pulseCount--; 
    } else {
        pulseCount++; 
    }
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
    Serial.begin(9600);

    // MOTOR PINS
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    // ENCODER PINS
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    // Initial state: Motor stopped
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);

    // INTERRUPT SETUP
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), countPulse, RISING);

    lastTime = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop()
{
    // ======================================
    // 1. RECEIVE DATA FROM C#
    // ======================================
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');
        int firstSep = data.indexOf(';');
        int secondSep = data.indexOf(';', firstSep + 1);
        int thirdSep = data.indexOf(';', secondSep + 1);

        if (firstSep > 0 && secondSep > 0 && thirdSep > 0)
        {
            targetSetpoint = data.substring(0, firstSep).toDouble();
            Kp = data.substring(firstSep + 1, secondSep).toDouble();
            Ki = data.substring(secondSep + 1, thirdSep).toDouble();
            Kd = data.substring(thirdSep + 1).toDouble();
        }
    }

    // ======================================
    // 2. PID CONTROL LOOP (Every 20ms)
    // ======================================
    unsigned long now = millis();

    if (now - lastTime >= T)
    {
        double dt = (now - lastTime) / 1000.0;

        // ----------------------------------
        // A. SOFT SETPOINT (RAMP / TRAJECTORY)
        // ----------------------------------
        if (currentSetpoint < targetSetpoint) {
            currentSetpoint += maxRampStep;
            if (currentSetpoint > targetSetpoint) currentSetpoint = targetSetpoint;
        } 
        else if (currentSetpoint > targetSetpoint) {
            currentSetpoint -= maxRampStep;
            if (currentSetpoint < targetSetpoint) currentSetpoint = targetSetpoint;
        }

        // ----------------------------------
        // B. READ ENCODER & COMPUTE RPM
        // ----------------------------------
        noInterrupts();
        long pulses = pulseCount;
        pulseCount = 0;
        interrupts();

        double rawPV = (pulses * 60.0) / (PULSES_PER_REVOLUTION * dt);
        
        // Low-pass filter
        filteredPV = 0.85 * filteredPV + 0.15 * rawPV;
        PV = filteredPV;

        // ----------------------------------
        // C. PID COMPUTATION
        // ----------------------------------
        error = currentSetpoint - PV;
        
        // Proportional term
        double P_term = Kp * error;
        
        // Derivative term (Derivative on Measurement to prevent derivative kick)
        derivative = -(PV - last_PV) / dt;
        double D_term = Kd * derivative;

        // Integral term with Clamping Anti-Windup
        double test_output = P_term + (Ki * integral) + D_term;
        bool is_saturated_high = (test_output >= 255) && (error > 0);
        bool is_saturated_low = (test_output <= -255) && (error < 0);

        if (!is_saturated_high && !is_saturated_low) 
        {
            integral += error * dt;
        }
        
        double I_term = Ki * integral;
        
        // Compute total output
        double output = P_term + I_term + D_term;
        output = constrain(output, -255.0, 255.0);
        
        int PWM_out = (int)output;

        // ----------------------------------
        // D. DRIVE MOTOR (L298 OUTPUT)
        // ----------------------------------
        if (PWM_out > 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, PWM_out);
        } 
        else if (PWM_out < 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, abs(PWM_out)); 
        } 
        else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, 0);
        }

        // ----------------------------------
        // E. UPDATE STATE & SERIAL TELEMETRY
        // ----------------------------------
        last_PV = PV; // Store current PV for the next derivative cycle
        lastTime = now;

        // Output for plotting / telemetry
        Serial.print(currentSetpoint);
        Serial.print(",");
        Serial.println(PV);
    }
}