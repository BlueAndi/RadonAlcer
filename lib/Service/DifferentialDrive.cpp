/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Differential drive
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <DifferentialDrive.h>
#include <Board.h>
#include <RobotConstants.h>
#include <Speedometer.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

int16_t DifferentialDrive::getMaxMotorSpeed() const
{
    return m_maxMotorSpeed;
}

void DifferentialDrive::setMaxMotorSpeed(int16_t maxMotorSpeed)
{
    m_maxMotorSpeed = maxMotorSpeed;

    m_motorSpeedLeftPID.setLimits(-m_maxMotorSpeed, m_maxMotorSpeed);
    m_motorSpeedRightPID.setLimits(-m_maxMotorSpeed, m_maxMotorSpeed);
}

int16_t DifferentialDrive::getLinearSpeed() const
{
    return m_linearSpeedCenterSetPoint;
}

void DifferentialDrive::setLinearSpeed(int16_t linearSpeed)
{
    m_linearSpeedCenterSetPoint = linearSpeed;
    calculateLinearSpeedLeftRight(m_linearSpeedCenterSetPoint, m_angularSpeedSetPoint, m_linearSpeedLeftSetPoint,
                                  m_linearSpeedRightSetPoint);
}

void DifferentialDrive::getLinearSpeed(int16_t& linearSpeedLeft, int16_t& linearSpeedRight)
{
    linearSpeedLeft  = m_linearSpeedLeftSetPoint;
    linearSpeedRight = m_linearSpeedRightSetPoint;
}

void DifferentialDrive::setLinearSpeed(int16_t linearSpeedLeft, int16_t linearSpeedRight)
{
    m_linearSpeedLeftSetPoint  = linearSpeedLeft;
    m_linearSpeedRightSetPoint = linearSpeedRight;
    calculateLinearAndAngularSpeedCenter(m_linearSpeedLeftSetPoint, m_linearSpeedRightSetPoint,
                                         m_linearSpeedCenterSetPoint, m_angularSpeedSetPoint);
}

int16_t DifferentialDrive::getAngularSpeed() const
{
    return m_angularSpeedSetPoint;
}

void DifferentialDrive::setAngularSpeed(int16_t angularSpeed)
{
    m_angularSpeedSetPoint = angularSpeed;
    calculateLinearSpeedLeftRight(m_linearSpeedCenterSetPoint, m_angularSpeedSetPoint, m_linearSpeedLeftSetPoint,
                                  m_linearSpeedRightSetPoint);
}

void DifferentialDrive::process(uint32_t period)
{
    /* The differential drive must be enabled.
     * The calibration is essential! The max. motor speed in [steps/s] is needed for closed-loop-control.
     */
    if ((true == m_isEnabled) &&
        (0 < m_maxMotorSpeed))
    {
        Speedometer& speedometer        = Speedometer::getInstance();
        IMotors&     motors             = Board::getInstance().getMotors();
        int32_t      maxMotorSpeed      = static_cast<int32_t>(m_maxMotorSpeed);      /* [steps/s] */
        int32_t      pwmMaxMotorSpeed   = static_cast<int32_t>(motors.getMaxSpeed()); /* [digits] */
        int16_t      pwmMotorSpeedLeft  = 0;                                          /* [digits] */
        int16_t      pwmMotorSpeedRight = 0;                                          /* [digits] */

        m_motorSpeedLeftPID.setSampleTime(period);
        m_motorSpeedRightPID.setSampleTime(period);

        /* If left motor is stopped, the PID controller shall be cleared. */
        if (0 == m_linearSpeedLeftSetPoint)
        {
            m_motorSpeedLeftPID.clear();
            m_lastLinearSpeedLeft = 0;
        }
        /* Handle left motor PID control. */
        else
        {
            int16_t linearSpeedLeft = speedometer.getLinearSpeedLeft();
            int32_t motorSpeedLeft =
                m_lastLinearSpeedLeft +
                m_motorSpeedLeftPID.calculate(m_linearSpeedLeftSetPoint, linearSpeedLeft); /* [steps/s] */

            /* Limit to max. motor speed in [steps/s] */
            motorSpeedLeft = constrain(motorSpeedLeft, -m_maxMotorSpeed, m_maxMotorSpeed);

            /* For the velocity PID remember the last PID output value. */
            m_lastLinearSpeedLeft = motorSpeedLeft;

            /* Convert speed from [steps/s] to [digits]. */
            pwmMotorSpeedLeft = static_cast<int16_t>(motorSpeedLeft * pwmMaxMotorSpeed / maxMotorSpeed); /* [digits] */
        }

        /* If right motor is stopped, the PID controller shall be cleared. */
        if (0 == m_linearSpeedRightSetPoint)
        {
            m_motorSpeedRightPID.clear();
            m_lastLinearSpeedRight = 0;
        }
        /* Handle right motor PID control. */
        else
        {
            int16_t linearSpeedRight = speedometer.getLinearSpeedRight();
            int32_t motorSpeedRight =
                m_lastLinearSpeedRight +
                m_motorSpeedRightPID.calculate(m_linearSpeedRightSetPoint, linearSpeedRight); /* [steps/s] */

            /* Limit to max. motor speed in [steps/s] */
            motorSpeedRight = constrain(motorSpeedRight, -m_maxMotorSpeed, m_maxMotorSpeed);

            /* For the velocity PID remember the last PID output value. */
            m_lastLinearSpeedRight = motorSpeedRight;

            /* Convert speed from [steps/s] to [digits]. */
            pwmMotorSpeedRight =
                static_cast<int16_t>(motorSpeedRight * pwmMaxMotorSpeed / maxMotorSpeed); /* [digits] */
        }

        motors.setSpeeds(pwmMotorSpeedLeft, pwmMotorSpeedRight);
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void DifferentialDrive::calculateLinearSpeedLeftRight(int16_t linearSpeedCenter, int16_t angularSpeed,
                                                      int16_t& linearSpeedLeft, int16_t& linearSpeedRight)
{
    int32_t linearSpeedCenter32 = static_cast<int32_t>(linearSpeedCenter);              /* [steps/s] */
    int32_t angularSpeed32      = static_cast<int32_t>(angularSpeed);                   /* [mrad/s] */
    int32_t wheelDiameter32     = static_cast<int32_t>(RobotConstants::WHEEL_DIAMETER); /* [mm] */
    int32_t wheelBase32         = static_cast<int32_t>(RobotConstants::WHEEL_BASE);     /* [mm] */

    linearSpeedLeft  = (linearSpeedCenter32 * 2 - (angularSpeed32 * wheelBase32) / 1000) / wheelDiameter32;
    linearSpeedRight = (linearSpeedCenter32 * 2 + (angularSpeed32 * wheelBase32) / 1000) / wheelDiameter32;
}

void DifferentialDrive::calculateLinearAndAngularSpeedCenter(int16_t linearSpeedLeft, int16_t linearSpeedRight,
                                                             int16_t& linearSpeedCenter, int16_t& angularSpeed)
{
    int32_t linearSpeedLeft32   = static_cast<int32_t>(linearSpeedLeft);                            /* [steps/s] */
    int32_t linearSpeedRight32  = static_cast<int32_t>(linearSpeedRight);                           /* [steps/s] */
    int32_t wheelDiameter32     = static_cast<int32_t>(RobotConstants::WHEEL_DIAMETER);             /* [mm] */
    int32_t wheelBase32         = static_cast<int32_t>(RobotConstants::WHEEL_BASE);                 /* [mm] */
    int32_t linearSpeedCenter32 = (wheelDiameter32 * (linearSpeedRight32 + linearSpeedLeft32)) / 4; /* [steps/s] */
    int32_t angularSpeed32 =
        (wheelDiameter32 * (linearSpeedRight32 - linearSpeedLeft32) * 1000) / wheelBase32; /* [mrad/s] */

    linearSpeedCenter = static_cast<int16_t>(linearSpeedCenter32);
    angularSpeed      = static_cast<int16_t>(angularSpeed32);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
