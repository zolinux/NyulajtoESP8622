// Linear_c.hpp

#ifndef _LINEAR_C_h
#define _LINEAR_C_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
/*************************** Move params ***************************/

struct LinearConfig
{
    uint8_t pulsePerRevolution;
    float distanceMmPerRevolution;          // M4 screw
    float decelerationDistanceMm;           // start deceleration before the end
    float totalLinearLengthMm;              // max length linear movement can occur
    uint16_t motorMinSpeedPwm;              // minimal PWM value that can be used to rotate the motor
};

enum class LinearISREventType
{
    HomeTriggered,
    EncoderTriggered
};


class ISensorListener
{
public:
    virtual void onSensor (uint8_t idx, bool state) = 0;
};
class IEncoderListener
{
public:
    virtual void onPulse () = 0;
};


class LinearUnitWithEncoderAndHomeSensor : public ISensorListener, public IEncoderListener
{
public:
    typedef std::function<void (float, int32_t) > MotionStoppedCb;
    LinearUnitWithEncoderAndHomeSensor (uint8_t pinDirPos, uint8_t pinDirNeg, uint8_t pinEncIn, uint8_t pinHomeSensor);
    ~LinearUnitWithEncoderAndHomeSensor();
    void init (const LinearConfig &hardware);
    void moveTo (float pos);
    void setSpeed (uint16_t speed);
    void doTasks();
    float getPos() const;
    float getEndPos() const;
    void setStoppedHandler (MotionStoppedCb handler);
    void onSensor (uint8_t idx, bool state) override;
    void onPulse() override;
    void freeRun (bool start, bool dir, uint16_t speed);

protected:
    struct LinearUnits
    {
        uint8_t pulsePerRevolution;
        float distancePerRevolution;            // M4 screw
        float distancePerPulse;                 // distance moved by 1 pulse
        uint16_t decelerationDistanceInPulses;  // number of pulses for the total length of decDist
        uint16_t totalLinearLengthPulses;       // max length linear movement can occur
        uint16_t motorMinSpeedPwm;              // minimal PWM value that can be used to rotate the motor
    };

    struct LinearCtl
    {
        uint16_t currentPosition;               // 0 is home, always absolute position in pulses
        uint16_t currentSpeed;                  //
        uint16_t movingTo;                      // unit should stop at position if reached
        uint16_t maxSpeed;                      // speed should not go above this
        bool isMoving;                          // should be/is in move
        bool initialized;                       // true if calibrated to home position happened
        bool direction;                         // position will be updated using this information (-1 or +1)
        uint8_t pinDriven;                      // pin of motor pole currently driven
        uint8_t pinSink;                        // pin of motor pole sinking current
        unsigned long opStarted;                // timestamp of operation start
        unsigned long lastPulse;                // ms of last pulse - for debouncing
    };

    uint8_t m_pinDirPos;
    uint8_t m_pinDirNeg;
    uint8_t m_pinEncIn;
    uint8_t m_pinHomeSensor;
    volatile LinearCtl m_motor;
    LinearUnits m_units;
    bool m_motorNeedsStop;
    uint16_t m_pulseDetected;

private:
    const bool m_homeSensorActiveStatus = false; // home sensor is true if active or false if active
    const uint16_t m_moveTimeoutPulseMs = 1500; // time for pulse must arrive in case motor is moving
    const uint16_t m_pwmFreq = 100;
    MotionStoppedCb m_stoppedCb;
    bool m_stoppedHandlerShouldBeInvoked;
    uint16_t *m_cosTable;

    void StartStopMotor (bool start);
    void updateMotorSpeed() const;
    void calculateMotorSpeed (uint16_t remainingPulses);
};

#endif

