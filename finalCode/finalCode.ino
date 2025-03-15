#include <math.h>

#define VOLTAGE_SENSOR_PIN A0   // AC voltage input pin
#define CURRENT_SENSOR_PIN A1   // ACS712 sensor input pin
#define XOR_OUTPUT_PIN 8        // XOR gate output pin

const float VREF = 5.0;             // Arduino reference voltage
const float scaleFactor = 11.0;      // Voltage divider scaling factor
const float transformerRatio = 0.05; // Transformer turn ratio
const int numSamples = 200;          // Number of samples for RMS calculation

const float ACS712_SENSITIVITY = 0.066; // 66mV/A for ACS712 30A
const float ACS712_OFFSET = 2.5;        // Midpoint offset (no-load)

float frequency = 50.0; // Set to 50Hz

void setup() {
    Serial.begin(9600);
    pinMode(XOR_OUTPUT_PIN, INPUT);  // XOR Gate output pin
}

// Function to calculate RMS Voltage
float calculateRMSVoltage() {
    float sumSquares = 0;
    float offsetVoltage = 2.5;  // DC offset in circuit

    for (int i = 0; i < numSamples; i++) {
        int rawValue = analogRead(VOLTAGE_SENSOR_PIN);
        float voltage = (rawValue * VREF) / 1023.0;  // Convert ADC to voltage
        voltage -= offsetVoltage;  // Remove DC offset (center at 0V)
        sumSquares += voltage * voltage;  // Sum of squares
        delay(2);  // Small delay for stable sampling
    }

    float Vrms_arduino = sqrt(sumSquares / numSamples) * scaleFactor;
    float Vrms_actual = Vrms_arduino / transformerRatio;
    return Vrms_actual;
}

// Function to calculate RMS Current
float calculateRMSCurrent() {
    float sumSquares = 0;

    for (int i = 0; i < numSamples; i++) {
        float rawVoltage = analogRead(CURRENT_SENSOR_PIN) * (VREF / 1023.0);
        float current = (rawVoltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
        sumSquares += current * current;
        delay(2);
    }

    float Irms = sqrt(sumSquares / numSamples);
    return Irms;
}

// Function to calculate Power Factor using XOR Gate method (Corrected)
float calculatePowerFactor_XOR() {
    float pulsewidth = 0;
    int validReadings = 0;

    for (int i = 0; i < 5; i++) { // Take multiple readings to reduce noise
        float pw = pulseIn(XOR_OUTPUT_PIN, HIGH, 20000);  // 20ms timeout
        if (pw > 0) {  // Ignore invalid readings
            pulsewidth += pw;
            validReadings++;
        }
        delay(10); // Small delay to stabilize readings
    }

    if (validReadings == 0) return 0.0;  // Prevent division by zero

    pulsewidth /= validReadings; // Take average pulse width

    float phase = (pulsewidth / 20000.0) * 180;  // Corrected phase calculation
    float powerfactor = cos(phase * PI / 180);   // Convert to PF

    // Ensure PF is in valid range (0 to 1)
    if (powerfactor < 0) powerfactor = 0;
    if (powerfactor > 1) powerfactor = 1;

    return powerfactor;
}

void loop() {
    float Vrms = calculateRMSVoltage();
    float Irms = calculateRMSCurrent();
    float powerFactor = calculatePowerFactor_XOR();
    float realPower = Vrms * Irms * powerFactor;

    Serial.print("Voltage (Vrms): "); Serial.print(Vrms); Serial.println(" V");
    Serial.print("Current (Irms): "); Serial.print(Irms); Serial.println(" A");
    Serial.print("Real Power (P): "); Serial.print(realPower); Serial.println(" W");
    Serial.print("Power Factor (PF): "); Serial.print(powerFactor); Serial.println();

    Serial.println("-----------------------------------");
    delay(1000);
}
