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

// Servo
Servo lidServo;
Servo lid2Servo;
constexpr int servoPIN = 15;
constexpr int lid2ServoPIN = 4;
constexpr int SERVO_DURATION_TOLERANCE_MS = 4000;

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
        motion.servo->attach(motion.pin, 500, 2400);
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
            startSmoothMove(lidMotion, servoPIN, lidServo, 180, 0, 50, 1000);
        }
        lidOpen = true;
        lastOpen = millis();
        Serial.println("Open lid!");
    }
    if (which & LID_2)
    {
        if (!lid2Open)
        {
            startSmoothMove(lid2Motion, lid2ServoPIN, lid2Servo, 55, 180, 50, 1000);
        }
        lid2Open = true;
        lastLid2Open = millis();
        Serial.println("Open lid2!");
    }
}

void keepClosed()
{
    lidMotion.servo->attach(servoPIN, 500, 2400);
    lidMotion.servo->write(lidOpen ? openAngle : closedAngle);
    lidMotion.attached = true;
}

void closeLid(const int which)
{
    if (which & LID_1)
    {
        if (lidOpen)
        {
            startSmoothMove(lidMotion, servoPIN, lidServo, 0, 180, 50, 1000);
        }
        lidOpen = false;
        lastClosed = millis();
        Serial.println("Close lid!");
    }
    if (which & LID_2)
    {
        if (lid2Open)
        {
            startSmoothMove(lid2Motion, lid2ServoPIN, lid2Servo, 180, 55, 50, 1000);
        }
        lid2Open = false;
        lastLid2Closed = millis();
        Serial.println("Close lid2!");
    }
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
