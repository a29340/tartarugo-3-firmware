#include <ESP32Servo.h>
#include <AccelStepper.h>


bool lidOpen = false;
bool lidOverride = false;
int openAngle = 185;
int closedAngle = 78;
unsigned long lastOpen = 0;
long lastFeedAmount = 0;

// Servo
Servo lidServo;
constexpr int servoPIN = 12;
constexpr int SERVO_DURATION_TOLERANCE_MS = 4000;

struct SmoothServo
{
    Servo* servo;
    int from;
    int to;
    int steps;
    int duration;
    unsigned long startTime;
    bool active;
} lidMotion;

void startSmoothMove(Servo& servo, int from, int to, int steps, int duration)
{
    lidMotion.servo = &servo;
    lidMotion.from = from;
    lidMotion.to = to;
    lidMotion.steps = steps;
    lidMotion.duration = duration;
    lidMotion.startTime = millis();
    lidMotion.active = true;
    lidMotion.servo->attach(servoPIN, 500, 2400);
}

void updateSmoothMove()
{
    if (!lidMotion.active) return;

    unsigned long now = millis();
    unsigned long elapsed = now - lidMotion.startTime;

    if (elapsed >=
        (static_cast<unsigned long>(lidMotion.duration) + SERVO_DURATION_TOLERANCE_MS))
    {
        lidMotion.servo->detach();
        lidMotion.active = false;
        return;
    }

    if (elapsed >= static_cast<unsigned long>(lidMotion.duration))
    {
        lidMotion.servo->write(lidMotion.to); // final angle
        return;
    }

    float progress = static_cast<float>(elapsed) / lidMotion.duration; // 0..1
    // cosine ease-in/out
    float factor = (1 - cos(progress * PI)) / 2;
    int angle = lidMotion.from + (lidMotion.to - lidMotion.from) * factor;

    lidMotion.servo->write(angle);
}

void openLid()
{
    if (!lidOpen)
    {
        startSmoothMove(lidServo, closedAngle, openAngle, 50, 1000);
    }
    lidOpen = true;
    lastOpen = millis();
    Serial.println("Open lid!");
}

void closeLid()
{
    if (lidOpen)
    {
        startSmoothMove(lidServo, openAngle, closedAngle, 100, 1000);
    }
    lidOpen = false;
    Serial.println("Close lid!");
}

// Stepper
constexpr int stepPIN1 = 13;
constexpr int stepPIN2 = 14;
constexpr int stepPIN3 = 15;
constexpr int stepPIN4 = 2;
AccelStepper stepper(AccelStepper::FULL4WIRE, stepPIN1, stepPIN3, stepPIN2, stepPIN4);

constexpr int MAX_MOVEMENTS = 5;
long feedMovements[MAX_MOVEMENTS] = {-500, 500, -1000, 1000, 0};
int feedMovementIndex = 0;

void setupMotors()
{
    // Init servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    lidServo.setPeriodHertz(50); // standard 50 Hz servo

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

void updateStepper() {
    stepper.run();
    if (stepper.distanceToGo() == 0) {
        if (feedMovementIndex >= MAX_MOVEMENTS) {
            stepper.disableOutputs();
        } else {
            feedMovementIndex++;
            stepper.moveTo(feedMovements[feedMovementIndex]);
        }
    }
}
