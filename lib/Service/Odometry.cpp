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
 * @brief  Odometry
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Odometry.h>
#include <Board.h>
#include <RobotConstants.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/** 2 * PI in mrad fixpoint */
#define FP_2PI() (static_cast<int16_t>(2000.0f * PI))

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

void Odometry::process()
{
    IEncoders& encoders      = Board::getInstance().getEncoders();
    IMotors&   motors        = Board::getInstance().getMotors();
    int16_t    relStepsLeft  = m_relEncLeft.calculate(encoders.getCountsLeft());
    int16_t    relStepsRight = m_relEncRight.calculate(encoders.getCountsRight());
    uint16_t   absStepsLeft  = abs(relStepsLeft);
    uint16_t   absStepsRight = abs(relStepsRight);

    /* Calculate absolute accumulated number steps for the left encoder. */
    m_absEncStepsLeft += absStepsLeft - m_lastAbsRelEncStepsLeft;
    m_lastAbsRelEncStepsLeft = absStepsLeft;

    /* Calculate absolute accumulated number steps for the left encoder. */
    m_absEncStepsRight += absStepsRight - m_lastAbsRelEncStepsRight;
    m_lastAbsRelEncStepsRight = absStepsRight;

    /* Orientation shall not be calculated from stand still to driving,
     * because there can not be any orientation change before.
     */
    if ((0 == m_lastMotorSpeedLeft) && (0 == m_lastMotorSpeedRight))
    {
        /* Calculate it with the next motor speed change. */
        m_lastMotorSpeedLeft  = motors.getLeftSpeed();
        m_lastMotorSpeedRight = motors.getRightSpeed();
    }
    /* If one of the motor speed changes, calculate the delta orientation. */
    else if ((m_lastMotorSpeedLeft != motors.getLeftSpeed()) || (m_lastMotorSpeedRight != motors.getRightSpeed()))
    {
        /* The alpha is approximated for performance reason. */
        int16_t alpha = 1000 * (relStepsRight - relStepsLeft);
        alpha /= static_cast<int16_t>(RobotConstants::ENCODER_STEPS_PER_MM);
        alpha /= static_cast<int16_t>(RobotConstants::WHEEL_BASE); /* [mrad] */
        alpha %= FP_2PI(); /* -2*PI < alpha < +2*PI */

        /* Calculate orientation */
        m_orientation += alpha;
        m_orientation %= FP_2PI(); /* -2*PI < orientation < +2*PI */

        /* Reset to be able to calculate the next delta. */
        m_lastMotorSpeedLeft      = motors.getLeftSpeed();
        m_lastMotorSpeedRight     = motors.getRightSpeed();
        m_lastAbsRelEncStepsLeft  = 0;
        m_lastAbsRelEncStepsRight = 0;
        m_relEncLeft.setSteps(encoders.getCountsLeft());
        m_relEncRight.setSteps(encoders.getCountsRight());
    }
}

uint32_t Odometry::getMileageCenter() const
{
    uint32_t encoderSteps = (m_absEncStepsLeft + m_absEncStepsRight) / 2;

    return encoderSteps / RobotConstants::ENCODER_STEPS_PER_MM;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
