//
//
//

#include "Linear_c.hpp"
LinearUnitWithEncoderAndHomeSensor::LinearUnitWithEncoderAndHomeSensor (uint8_t pinDirPos, uint8_t pinDirNeg, uint8_t pinEncIn, uint8_t pinHomeSensor) :
    m_pinDirPos (pinDirPos), m_pinDirNeg (pinDirNeg), m_pinEncIn (pinEncIn), m_pinHomeSensor (pinHomeSensor), m_motor (
{
    0
}), m_units ({0}), m_motorNeedsStop (false), m_pulseDetected (0), m_stoppedCb (nullptr), m_stoppedHandlerShouldBeInvoked (false), m_cosTable (nullptr)
{
    m_motor.initialized = false;
    m_motor.maxSpeed = PWMRANGE;
    m_motor.isMoving = false;
    m_motor.lastPulse = millis();

}
LinearUnitWithEncoderAndHomeSensor::~LinearUnitWithEncoderAndHomeSensor()
{
    if (m_cosTable != nullptr)
    {
        free (m_cosTable);
    }
}
void LinearUnitWithEncoderAndHomeSensor::init (const LinearConfig &hardware)
{
    pinMode (m_pinEncIn, INPUT);
    pinMode (m_pinHomeSensor, INPUT_PULLUP);
    pinMode (m_pinDirPos, OUTPUT);
    pinMode (m_pinDirNeg, OUTPUT);

    analogWriteFreq (m_pwmFreq);
    // make sure motor is stopped
    StartStopMotor (false);

    m_units.motorMinSpeedPwm = hardware.motorMinSpeedPwm;
    m_units.distancePerRevolution = hardware.distanceMmPerRevolution;
    m_units.pulsePerRevolution = hardware.pulsePerRevolution;
    m_units.distancePerPulse = m_units.distancePerRevolution / m_units.pulsePerRevolution;              // distance moved by 1 pulse
    m_units.decelerationDistanceInPulses = hardware.decelerationDistanceMm / m_units.distancePerPulse;  // number of pulses for the total length of decDist
    m_units.totalLinearLengthPulses = hardware.totalLinearLengthMm / m_units.distancePerPulse;          // number of pulses for the total end-to-end movement

    Serial.print ("Drive props: ");
    Serial.print (m_units.distancePerPulse);
    Serial.print (',');
    Serial.print (m_units.decelerationDistanceInPulses);
    Serial.print (',');
    Serial.println (m_units.totalLinearLengthPulses);

    // calculate cosine table
    if (m_cosTable != nullptr)
    {
        free (m_cosTable);
    }

    m_cosTable = (uint16_t *) malloc (m_units.decelerationDistanceInPulses * sizeof (uint16_t)); // allocate memory for the costable
    double step = ( (double) PI) / m_units.decelerationDistanceInPulses;
    Serial.println (F ("CosTBL: "));
    for (uint16_t i = 0; i < m_units.decelerationDistanceInPulses - 1; i++)
    {
        m_cosTable[i] = (cos (step * i + step * 0.5) + 1.0) * 32767;   // table entry normalized to full scale of 16 bit unsigned [1..0]
        Serial.print (i);
        Serial.print (":");
        Serial.println (m_cosTable[i]);
    }
    m_cosTable[m_units.decelerationDistanceInPulses - 1] = 3276;    // set lowest speed for reaching the target (5% scale)

    m_motorNeedsStop = false;
    // start homing
    if (static_cast<bool> (digitalRead (m_pinHomeSensor)) == m_homeSensorActiveStatus)
    {
        // is homed
        m_motor.currentPosition = 0;
        m_motor.currentSpeed = 0;
        m_motor.initialized = true;
        m_motor.movingTo = 0;
        StartStopMotor (false);
        m_motor.isMoving = false;
        Serial.println (F ("Home position detected"));
    }
    else
    {
        // home sensor not active, start homing
        m_motor.initialized = false;
        m_motor.currentSpeed = m_units.motorMinSpeedPwm;
        m_motor.opStarted = millis();
        m_motor.isMoving = true;    // start moving
        m_motor.currentPosition = 1;    // set for direction calculation
        m_motor.movingTo = 0;
        StartStopMotor (true);
        Serial.println (F ("Homing..."));
    }

}
void LinearUnitWithEncoderAndHomeSensor::setStoppedHandler (MotionStoppedCb handler)
{
    m_stoppedCb = handler;
}
void LinearUnitWithEncoderAndHomeSensor::setSpeed (uint16_t speed)
{
    m_motor.maxSpeed = speed > PWMRANGE ? PWMRANGE : speed;
    m_motor.currentSpeed = m_motor.maxSpeed;

    if (m_motor.isMoving)
    {
    }

    Serial.print (F ("Speed set to "));
    Serial.println (m_motor.maxSpeed);

}
float LinearUnitWithEncoderAndHomeSensor::getPos() const
{
    return m_motor.currentPosition;
}
float LinearUnitWithEncoderAndHomeSensor::getEndPos() const
{
    return m_units.distancePerPulse * m_units.totalLinearLengthPulses;
}

void LinearUnitWithEncoderAndHomeSensor::updateMotorSpeed() const
{
    analogWrite (m_motor.pinDriven, m_motor.currentSpeed);
}
void LinearUnitWithEncoderAndHomeSensor::freeRun (bool start, bool dir, uint16_t speed)
{
    m_motor.currentPosition = 42;
    m_motor.movingTo = m_motor.currentPosition + 10 * (dir ? 1 : -1);
    setSpeed (speed);
    m_motor.initialized = false;
    StartStopMotor (start);
}

void LinearUnitWithEncoderAndHomeSensor::moveTo (float pos)
{
    if (!m_motor.initialized)
    {
        Serial.println (F ("Motor not initialized"));
    }
    else
    {
        m_motor.movingTo = pos / m_units.distancePerPulse;
        if (m_motor.movingTo > m_units.totalLinearLengthPulses)
        {
            m_motor.movingTo = m_units.totalLinearLengthPulses;    // limit movement not to exceed the limits
        }
        auto distToMove = abs (m_motor.movingTo - m_motor.currentPosition);
        if (distToMove < 2)
        {
            Serial.println (F ("Not starting the motor for so short distance"));
            return;
        }
        bool dir = m_motor.movingTo > m_motor.currentPosition;
        calculateMotorSpeed (distToMove);

        Serial.print (F ("Start moving to "));
        Serial.print (m_motor.movingTo);
        Serial.print (F (" from "));
        Serial.print (m_motor.currentPosition);
        Serial.print (" Direction: ");
        Serial.println (dir ? '+' : '-');
        Serial.print (F (" at speed "));
        Serial.print (m_motor.currentSpeed);

        m_motor.opStarted = millis();
        m_motor.isMoving = true;
        StartStopMotor (true);
    }
}
void LinearUnitWithEncoderAndHomeSensor::doTasks()
{
    auto milli = millis();
    if (m_motor.opStarted < milli)
    {
        auto timeElapsed = milli - m_motor.opStarted;
        if (m_motor.isMoving && (timeElapsed > m_moveTimeoutPulseMs))
        {
            Serial.print (F ("ERROR: Motor does not move for msec: "));
            Serial.print (timeElapsed);
            Serial.print (F (" Last known position:"));
            Serial.println (m_motor.currentPosition);
            m_motorNeedsStop = true;
        }
    }

    if (m_motorNeedsStop)
    {
        m_motor.isMoving = false;
        m_motorNeedsStop = false;
        StartStopMotor (false);   // stop moving
        m_stoppedHandlerShouldBeInvoked = true;
    }

    if (m_stoppedHandlerShouldBeInvoked)
    {
        m_stoppedHandlerShouldBeInvoked = false;
        if (m_stoppedCb != nullptr)
        {
            m_stoppedCb (getPos(), 0);  // error not implemented
        }
    }
}
void LinearUnitWithEncoderAndHomeSensor::calculateMotorSpeed (uint16_t remainingPulses)
{
    if (remainingPulses > m_units.decelerationDistanceInPulses)
    {
        m_motor.currentSpeed = m_motor.maxSpeed;
    }
    else
    {
        uint16_t idx = m_units.decelerationDistanceInPulses - remainingPulses;
        uint16_t speedDynamicRange = m_motor.maxSpeed - m_units.motorMinSpeedPwm;
        uint32_t speed = static_cast<uint32_t> (speedDynamicRange) * m_cosTable[idx] / 64 / 1024 + m_units.motorMinSpeedPwm;
        m_motor.currentSpeed = static_cast<uint16_t> (speed);
        //Serial.print ("S:");
        //Serial.println (m_motor.currentSpeed);
    }
}
void LinearUnitWithEncoderAndHomeSensor::onSensor (uint8_t idx, bool state)
{
    if (m_motor.initialized)
    {
        // should not be done anythin but stop the movement
        bool dir = m_motor.movingTo > m_motor.currentPosition;
        m_motorNeedsStop = m_motor.isMoving && !dir;
    }
    else
    {
        // homing... origin reached
        m_motor.currentSpeed = m_motor.maxSpeed;    // enable max speed for next movement
        m_motor.movingTo = 0;
        m_motor.initialized = true;
        m_motorNeedsStop = true;
    }

    Serial.println ("H");
    m_motor.currentPosition = 0;
    m_motor.opStarted = millis();

    if (m_motorNeedsStop)
    {
        StartStopMotor (false); // stop moving
        m_motor.isMoving = false;
        m_motorNeedsStop = false;
        m_stoppedHandlerShouldBeInvoked = true;
    }
}
void LinearUnitWithEncoderAndHomeSensor::onPulse()
{
    auto ms = millis();
    if (ms - m_motor.lastPulse <= 12)
    {
        //one pulse should not be repeated in less than 6ms (5k rpm), then it is ignored
        return;
    }
    m_motor.lastPulse = ms;

    Serial.print (".");
    if (m_motor.initialized)
    {
        bool dir = m_motor.direction;

        // modify current position
        if ( (dir && m_motor.currentPosition < 65000)
                || (!dir && m_motor.currentPosition > 0))
        {
            int8_t dirFactor = dir ? 1 : -1;
            m_motor.currentPosition += 1 * dirFactor;

            if (m_motor.currentPosition >= m_units.totalLinearLengthPulses && dir)
            {
                // safety feature: stop motor if max position reached
                m_motorNeedsStop = true;
                Serial.println (F ("Motion forced stop at the end of the rail"));
            }
            else
            {
                // during move the speed should be controlled  for deceleration
                int16_t distanceToMove = !dir ? m_motor.currentPosition - m_motor.movingTo : m_motor.movingTo - m_motor.currentPosition; // get the remaining distance
                if (abs (distanceToMove) < 2)
                {
                    // destination has been reached, stop motor
                    m_motorNeedsStop = true;
                    Serial.print ("PR:");
                    Serial.println (m_motor.currentPosition);
                }
                else if (distanceToMove < 0)
                {
                    // at stopping we still move due to mass of system
                    m_motorNeedsStop = true;
                    Serial.print ("X:");
                    Serial.println (m_motor.currentPosition);
                }
                else if (distanceToMove <= m_units.decelerationDistanceInPulses)
                {
                    // we are in deceleration phase, update motor speed...
                    calculateMotorSpeed (distanceToMove);
                    noInterrupts();
                    updateMotorSpeed();
                    interrupts();
                }
                //else
                //{
                //}
            }
        }
        if (m_motorNeedsStop)
        {
            StartStopMotor (false);  // stop moving
            m_motor.isMoving = false;
            m_motorNeedsStop = false;
            m_stoppedHandlerShouldBeInvoked = true;
        }

    }
    else
    {
        // homing..., do nothing

    }
    m_motor.opStarted = millis();
}

void LinearUnitWithEncoderAndHomeSensor::StartStopMotor (bool start)
{
    if (start)
    {
        // update pin config on start
        bool dir = m_motor.movingTo > m_motor.currentPosition;
        m_motor.direction = dir;
        m_motor.pinDriven = dir ? m_pinDirPos : m_pinDirNeg;
        m_motor.pinSink = dir ? m_pinDirNeg : m_pinDirPos;

        analogWrite (m_motor.pinSink, 0);
        updateMotorSpeed();
        m_motor.opStarted = millis();
        Serial.println ("Start");
    }
    else
    {
        // turn off drive
        m_motor.currentSpeed = 0;
        analogWrite (m_pinDirNeg, 0);
        analogWrite (m_pinDirPos, 0);
        Serial.println ("Stop");
    }

}