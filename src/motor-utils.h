#include <ESP32Servo.h>
#include <AccelStepper.h>


bool lidOpen = false;
bool lid2Open = false;
bool lidOverride = false;
int openAngle = 0;
int closedAngle = 180;
unsigned long lastOpen = 0;
unsigned long lastClosed = 0;
unsigned long lastLid2Open = 0;
unsigned long lastLid2Closed = 0;
long lastFeedAmount = 0;

// defined in main.cpp (included later in the same translation unit)
void logEvent(const char* msg);

// Servo
Servo lidServo;
Servo lid2Servo;
constexpr int servoPIN = 15;
constexpr int lid2ServoPIN = 4;
constexpr int SERVO_DURATION_TOLERANCE_MS = 4000;

// Per-lid target angles (degrees)
constexpr int LID1_OPEN_ANGLE = 0;
constexpr int LID1_CLOSED_ANGLE = 100;
constexpr int LID2_OPEN_ANGLE = 180;
constexpr int LID2_CLOSED_ANGLE = 55;
constexpr int SERVO_MOVE_STEPS = 50;
constexpr int SERVO_MOVE_DURATION_MS = 1000;

// Position feedback: each servo's internal potentiometer wiper is wired
// through a voltage divider (max < 3.3V) to an ADC pin, so the arm position
// can be read even while the servo PWM output is detached.
constexpr int LID1_POT_PIN = 32;
constexpr int LID2_POT_PIN = 33;
constexpr int POT_SAMPLES = 5;
constexpr int POT_SAMPLE_DELAY_MS = 2;
constexpr int POT_MIN_SPAN = 20; // min ADC difference between the two extremes for a valid calibration
constexpr int LID_POSITION_TOLERANCE_DEG = 15; // deadband before a position mismatch is corrected
constexpr int LID_POSITION_SETTLE_MS = 1000; // wait after the last command before the watchdog checks
constexpr int SERVO_CALIBRATION_SETTLE_MS = 1500; // wait after driving to an extreme before sampling the pot

struct LidPotCalibration
{
    int potOpen = -1; // ADC reading when the lid is fully open
    int potClosed = -1; // ADC reading when the lid is fully closed
    bool valid = false;
};

LidPotCalibration lid1Pot;
LidPotCalibration lid2Pot;

int readPot(const int pin)
{
    long sum = 0;
    for (int i = 0; i < POT_SAMPLES; i++)
    {
        sum += analogRead(pin);
        delay(POT_SAMPLE_DELAY_MS);
    }
    return (int)(sum / POT_SAMPLES);
}

void calibrateLid(Servo& servo, const int servoPin, const int potPin,
                  const int openAngleTarget, const int closedAngleTarget,
                  LidPotCalibration& cal, const char* label)
{
    servo.attach(servoPin);
    servo.write(openAngleTarget);
    delay(SERVO_CALIBRATION_SETTLE_MS);
    cal.potOpen = readPot(potPin);
    servo.write(closedAngleTarget);
    delay(SERVO_CALIBRATION_SETTLE_MS);
    cal.potClosed = readPot(potPin);
    cal.valid = abs(cal.potOpen - cal.potClosed) >= POT_MIN_SPAN;
    if (cal.valid)
    {
        Serial.printf("%s pot calibrated: open=%d closed=%d\n", label,
                      cal.potOpen, cal.potClosed);
    }
    else
    {
        Serial.printf("%s pot calibration FAILED: open=%d closed=%d - position feedback disabled\n",
                      label, cal.potOpen, cal.potClosed);
    }
}

void calibrateLids()
{
    Serial.println("Calibrating lid potentiometers...");
    analogReadResolution(12);
    analogSetPinAttenuation(LID1_POT_PIN, ADC_11db);
    analogSetPinAttenuation(LID2_POT_PIN, ADC_11db);
    // drive each lid to both extremes; both end up fully closed
    calibrateLid(lidServo, servoPIN, LID1_POT_PIN, LID1_OPEN_ANGLE,
                 LID1_CLOSED_ANGLE, lid1Pot, "Lid 1");
    calibrateLid(lid2Servo, lid2ServoPIN, LID2_POT_PIN, LID2_OPEN_ANGLE,
                 LID2_CLOSED_ANGLE, lid2Pot, "Lid 2");
    lidOpen = false;
    lid2Open = false;
    lastOpen = 0;
    lastClosed = millis();
    lastLid2Open = 0;
    lastLid2Closed = millis();
    Serial.println("Lid pot calibration done, both lids closed");
}

// Estimates the current arm angle in degrees from the pot reading.
// Returns -1 when the pot is not calibrated (position unknown).
int estimateLidAngle(const LidPotCalibration& cal, const int potPin,
                     const int openAngleTarget, const int closedAngleTarget)
{
    if (!cal.valid || cal.potOpen == cal.potClosed)
    {
        return -1;
    }
    const int adc = readPot(potPin);
    long angle = (long)(adc - cal.potOpen) *
        (closedAngleTarget - openAngleTarget) / (cal.potClosed - cal.potOpen) +
        openAngleTarget;
    const int lo = min(openAngleTarget, closedAngleTarget);
    const int hi = max(openAngleTarget, closedAngleTarget);
    if (angle < lo) angle = lo;
    if (angle > hi) angle = hi;
    return (int)angle;
}

int estimateLid1Angle()
{
    return estimateLidAngle(lid1Pot, LID1_POT_PIN, LID1_OPEN_ANGLE,
                            LID1_CLOSED_ANGLE);
}

int estimateLid2Angle()
{
    return estimateLidAngle(lid2Pot, LID2_POT_PIN, LID2_OPEN_ANGLE,
                            LID2_CLOSED_ANGLE);
}

struct SmoothServo
{
    Servo* servo;
    int pin;
    int from;
    int to;
    int steps;
    int duration;
    unsigned long startTime;
    bool active;
    bool attached = false;
} lidMotion, lid2Motion;

void updateSmoothMove(SmoothServo& motion)
{
    if (!motion.active) return;

    const unsigned long now = millis();
    const unsigned long elapsed = now - motion.startTime;

    if (elapsed >=
        (static_cast<unsigned long>(motion.duration) + SERVO_DURATION_TOLERANCE_MS))
    {
        motion.active = false;
        return;
    }

    if (elapsed >= static_cast<unsigned long>(motion.duration))
    {
        motion.servo->write(motion.to); // final angle
        return;
    }

    const float progress = static_cast<float>(elapsed) / motion.duration; // 0..1
    // cosine ease-in/out
    const float factor = (1 - cos(progress * PI)) / 2;
    const int angle = motion.from + (motion.to - motion.from) * factor;

    motion.servo->write(angle);
}

void startSmoothMove(SmoothServo& motion, const int pin, Servo& servo,
                     const int from, const int to, const int steps,
                     const int duration)
{
    motion.servo = &servo;
    motion.pin = pin;
    motion.from = from;
    motion.to = to;
    motion.steps = steps;
    motion.duration = duration;
    motion.startTime = millis();
    motion.active = true;
    if (!motion.attached) {
        motion.servo->attach(motion.pin);
        motion.attached = true;
    }
}

void updateSmoothMoves()
{
    updateSmoothMove(lidMotion);
    updateSmoothMove(lid2Motion);
}

enum Lid
{
    LID_1 = 1,
    LID_2 = 2,
    LID_BOTH = 3
};

void openLid(const int which)
{
    if (which & LID_1)
    {
        if (!lidOpen)
        {
            // start from the real arm position when the pot says where it is
            int from = LID1_CLOSED_ANGLE;
            const int actual = estimateLid1Angle();
            if (actual >= 0)
            {
                from = actual;
            }
            if (abs(from - LID1_OPEN_ANGLE) > LID_POSITION_TOLERANCE_DEG)
            {
                startSmoothMove(lidMotion, servoPIN, lidServo, from, LID1_OPEN_ANGLE,
                                SERVO_MOVE_STEPS, SERVO_MOVE_DURATION_MS);
            }
        }
        lidOpen = true;
        lastOpen = millis();
        Serial.println("Open lid!");
    }
    if (which & LID_2)
    {
        if (!lid2Open)
        {
            int from = LID2_CLOSED_ANGLE;
            const int actual = estimateLid2Angle();
            if (actual >= 0)
            {
                from = actual;
            }
            if (abs(from - LID2_OPEN_ANGLE) > LID_POSITION_TOLERANCE_DEG)
            {
                startSmoothMove(lid2Motion, lid2ServoPIN, lid2Servo, from, LID2_OPEN_ANGLE,
                                SERVO_MOVE_STEPS, SERVO_MOVE_DURATION_MS);
            }
        }
        lid2Open = true;
        lastLid2Open = millis();
        Serial.println("Open lid2!");
    }
}

void keepClosed()
{
    lidMotion.servo->attach(servoPIN);
    lidMotion.servo->write(lidOpen ? openAngle : closedAngle);
    lidMotion.attached = true;
}

void closeLid(const int which)
{
    if (which & LID_1)
    {
        if (lidOpen)
        {
            int from = LID1_OPEN_ANGLE;
            const int actual = estimateLid1Angle();
            if (actual >= 0)
            {
                from = actual;
            }
            if (abs(from - LID1_CLOSED_ANGLE) > LID_POSITION_TOLERANCE_DEG)
            {
                startSmoothMove(lidMotion, servoPIN, lidServo, from, LID1_CLOSED_ANGLE,
                                SERVO_MOVE_STEPS, SERVO_MOVE_DURATION_MS);
            }
        }
        lidOpen = false;
        lastClosed = millis();
        Serial.println("Close lid!");
    }
    if (which & LID_2)
    {
        if (lid2Open)
        {
            int from = LID2_OPEN_ANGLE;
            const int actual = estimateLid2Angle();
            if (actual >= 0)
            {
                from = actual;
            }
            if (abs(from - LID2_CLOSED_ANGLE) > LID_POSITION_TOLERANCE_DEG)
            {
                startSmoothMove(lid2Motion, lid2ServoPIN, lid2Servo, from, LID2_CLOSED_ANGLE,
                                SERVO_MOVE_STEPS, SERVO_MOVE_DURATION_MS);
            }
        }
        lid2Open = false;
        lastLid2Closed = millis();
        Serial.println("Close lid2!");
    }
}

// Watchdog: if the pot says the arm is not where the commanded state says it
// should be, move it back (from the real position). Runs in auto and manual
// (override) mode alike.
void checkLidPositionOne(Servo& servo, SmoothServo& motion, const int pin,
                         bool& isOpen, unsigned long& lastOpenedAt,
                         unsigned long& lastClosedAt,
                         const int openAngleTarget, const int closedAngleTarget,
                         const LidPotCalibration& cal, const int potPin,
                         const char* label)
{
    if (!cal.valid) return;
    if (motion.active) return;
    const unsigned long now = millis();
    const unsigned long lastCommanded =
        lastOpenedAt > lastClosedAt ? lastOpenedAt : lastClosedAt;
    if (now - lastCommanded < LID_POSITION_SETTLE_MS) return;

    const int actual = estimateLidAngle(cal, potPin, openAngleTarget,
                                        closedAngleTarget);
    if (actual < 0) return;
    const int expected = isOpen ? openAngleTarget : closedAngleTarget;
    if (abs(actual - expected) <= LID_POSITION_TOLERANCE_DEG) return;

    char msg[128];
    snprintf(msg, sizeof(msg),
             "%s position mismatch: expected %d, actual %d - correcting",
             label, expected, actual);
    Serial.println(msg);
    logEvent(msg);
    startSmoothMove(motion, pin, servo, actual, expected, SERVO_MOVE_STEPS,
                    SERVO_MOVE_DURATION_MS);
    if (expected == openAngleTarget)
    {
        lastOpenedAt = now;
    }
    else
    {
        lastClosedAt = now;
    }
}

void checkLidPosition()
{
    checkLidPositionOne(lidServo, lidMotion, servoPIN, lidOpen, lastOpen,
                        lastClosed, LID1_OPEN_ANGLE, LID1_CLOSED_ANGLE, lid1Pot,
                        LID1_POT_PIN, "Lid 1");
    checkLidPositionOne(lid2Servo, lid2Motion, lid2ServoPIN, lid2Open,
                        lastLid2Open, lastLid2Closed, LID2_OPEN_ANGLE,
                        LID2_CLOSED_ANGLE, lid2Pot, LID2_POT_PIN, "Lid 2");
}

// Stepper
constexpr int stepPIN1 = 13;
constexpr int stepPIN2 = 12;
constexpr int stepPIN3 = 14;
constexpr int stepPIN4 = 27;
AccelStepper stepper(AccelStepper::FULL4WIRE, stepPIN1, stepPIN3, stepPIN2, stepPIN4);

constexpr int MAX_MOVEMENTS = 5;
long feedMovements[MAX_MOVEMENTS] = {-250, 0, -250, 0, 0};
int feedMovementIndex = 0;

void setupMotors()
{
    // Init servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    lidServo.setPeriodHertz(50); // standard 50 Hz servo
    lid2Servo.setPeriodHertz(50); // standard 50 Hz servo

    // Init stepper
    stepper.setMaxSpeed(400);
    stepper.setAcceleration(800);
}

void feedAmount(const int amount)
{
    feedMovements[MAX_MOVEMENTS - 1] = amount;
    feedMovementIndex = 0;
    stepper.setCurrentPosition(0);
    stepper.moveTo(feedMovements[0]);
    stepper.enableOutputs();
    lastFeedAmount = amount;
}

void updateStepper()
{
    stepper.run();
    if (stepper.distanceToGo() == 0)
    {
        if (feedMovementIndex >= MAX_MOVEMENTS - 1)
        {
            stepper.disableOutputs();
        }
        else
        {
            feedMovementIndex++;
            stepper.moveTo(feedMovements[feedMovementIndex]);
        }
    }
}
