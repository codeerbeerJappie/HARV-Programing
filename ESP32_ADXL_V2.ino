/* ESP32 MASTER: ADXL345 -> pitch (float) + fast flag (byte) -> two Feelix motors
 *
 * ADXL345 on plain Wire (GPIO33 + GPIO25, order auto-detected).
 * Link.h claims Wire1 for the motors on GPIO21/22.
 *
 * Sent to each motor:
 *   float  pitch in degrees   -> callback index 0
 *   byte   1 = moving fast    -> callback index 2   (held for FAST_HOLD_MS once detected)
 */
#include "src/Link.h"
#include <Wire.h>

#define DEBUG           false

#define ESP32_SCL       22
#define ESP32_SDA       21
#define SLAVE_0_ADDRESS 0x70
#define SLAVE_1_ADDRESS 0x71
#define CB_PITCH        0      // slave receiveFloat
#define CB_FAST         2      // slave receiveByte

#define ADXL_PIN_A      25
#define ADXL_PIN_B      26
#define ADXL_ADDR       0x53

/* ---- tuning ---- */
const float    TILT_SIGN      = 1.5;    // -1.0 flips the pitch direction
const float    FAST_THRESHOLD = 0.9;    // motion above this = moving fast
const uint16_t FAST_HOLD_MS   = 4000;   // fast flag stays on this long
const float    PITCH_STEP     = 2.0;    // only send pitch when it changed by this many degrees

uint8_t slaves[MAX_NR_OF_SLAVES] = { SLAVE_0_ADDRESS, SLAVE_1_ADDRESS };
MasterDevice I2C_Master(slaves);

/* ---------- ADXL345 ---------- */
void adxlWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool adxlInit() {
    Wire.beginTransmission(ADXL_ADDR);
    if (Wire.endTransmission() != 0) return false;
    adxlWrite(0x2D, 0x08);   // measurement mode
    adxlWrite(0x31, 0x08);   // full resolution, +-2 g
    adxlWrite(0x2C, 0x0A);   // 100 Hz
    return true;
}

bool adxlConnect() {         // tries both pin orders
    for (int attempt = 0; attempt < 2; attempt++) {
        int sda = (attempt == 0) ? ADXL_PIN_A : ADXL_PIN_B;
        int scl = (attempt == 0) ? ADXL_PIN_B : ADXL_PIN_A;
        Wire.end();
        Wire.begin(sda, scl);
        Wire.setClock(100000);
        if (adxlInit()) {
            Serial.printf("ADXL345 ready (SDA=GPIO%d, SCL=GPIO%d)\n", sda, scl);
            return true;
        }
    }
    return false;
}

bool adxlRead(float &x, float &y, float &z) {
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(0x32);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)6) != 6) return false;
    int16_t rx = Wire.read() | (Wire.read() << 8);
    int16_t ry = Wire.read() | (Wire.read() << 8);
    int16_t rz = Wire.read() | (Wire.read() << 8);
    x = rx * 0.0039; y = ry * 0.0039; z = rz * 0.0039;
    return true;
}

/* ---------- sensor values ---------- */
bool  adxlReady = false;
float fx = 0, fy = 0, fz = 1, px = 0, py = 0, pz = 1;
float motion = 0.0, pitch = 0.0;
unsigned long fastUntil = 0;

void updateSensor() {
    float x, y, z;
    if (!adxlRead(x, y, z)) return;

    motion = 0.4 * motion + 0.6 * (fabs(x - px) + fabs(y - py) + fabs(z - pz));
    px = x; py = y; pz = z;

    fx = 0.6 * fx + 0.4 * x;
    fy = 0.6 * fy + 0.4 * y;
    fz = 0.6 * fz + 0.4 * z;
    pitch = TILT_SIGN * atan2(fx, sqrt(fy * fy + fz * fz)) * 180.0 / PI;

    unsigned long now = millis();
    if (now >= fastUntil && motion > FAST_THRESHOLD && pitch < 30) fastUntil = now + FAST_HOLD_MS;
}

/* ---------- main ---------- */
unsigned long prevSensor = 0, prevSend = 0, prevPrint = 0, prevRetry = 0;



void setup() {
    delay(1000);
    Serial.begin(115200);

    adxlReady = adxlConnect();
    if (!adxlReady) Serial.println("ADXL345 NOT found - check wiring");

    I2C_Master.init(ESP32_SDA, ESP32_SCL, CLOCK_SPEED::SLOW_MODE, DEBUG);
    
}

void loop() {
    unsigned long now = millis();

    if (adxlReady) {
        if (now - prevSensor >= 20) { prevSensor = now; updateSensor(); }
    } else if (now - prevRetry >= 1000) {
        prevRetry = now;
        adxlReady = adxlConnect();
    }

    /* send only on a change (plus a 1 s refresh), so the motors are interrupted as little as possible */
    if (now - prevSend >= 100) {
        prevSend = now;

        static float lastPitch = 999.0;
        static int8_t lastFast = -1;
        static unsigned long lastPitchTime = 0, lastFastTime = 0;

        if (fabs(pitch - lastPitch) >= PITCH_STEP || now - lastPitchTime >= 1000) {
            lastPitch = pitch;
            lastPitchTime = now;
            I2C_Master.sendFloat(pitch, SLAVE_0_ADDRESS, CB_PITCH);
            delay(5);
            I2C_Master.sendFloat(pitch, SLAVE_1_ADDRESS, CB_PITCH);
            delay(5);
        }

        int8_t fast = (now < fastUntil) ? 1 : 0;
        if (fast != lastFast || now - lastFastTime >= 4000) {
            lastFast = fast;
            lastFastTime = now;
            I2C_Master.sendByte(fast, SLAVE_0_ADDRESS, CB_FAST);
            delay(5);
            I2C_Master.sendByte(fast, SLAVE_1_ADDRESS, CB_FAST);
        }
    }

    if (now - prevPrint >= 200) {
        prevPrint = now;
        Serial.printf("pitch: %6.1f  motion: %.3f  fast: %d\n", pitch, motion, now < fastUntil);
    }
}