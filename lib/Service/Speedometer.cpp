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
 * @brief  Speedometer
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Speedometer.h>
#include <Board.h>
#include <RobotConstants.h>

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

Speedometer Speedometer::m_instance;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void Speedometer::process()
{
    IMotors&       motors           = Board::getInstance().getMotors();
    IEncoders&     encoders         = Board::getInstance().getEncoders();
    uint32_t       timestamp        = millis();                                  /* [ms] */
    int16_t        absEncStepsLeft  = encoders.getCountsLeft();                  /* [steps] */
    int16_t        absEncStepsRight = encoders.getCountsRight();                 /* [steps] */
    int32_t        diffStepsLeft    = m_relEncLeft.calculate(absEncStepsLeft);   /* [steps] */
    int32_t        diffStepsRight   = m_relEncRight.calculate(absEncStepsRight); /* [steps] */
    DriveDirection driveDirection   = DRIVE_DIRECTION_STOPPED;
    const int32_t  ONE_SECOND       = 1000; /* 1s in ms */

    /* Determine current drive direction.
     *
     * TODO Consider inertia by calibration. The motor voltage must be higher than a
     * certain threshold to move the robot. This threshold must be determined.
     */
    if ((0 == motors.getLeftSpeed()) && (0 == motors.getRightSpeed()))
    {
        driveDirection = DRIVE_DIRECTION_STOPPED;
    }
    else if (motors.getLeftSpeed() == (-motors.getRightSpeed()))
    {
        driveDirection = DRIVE_DIRECTION_STOPPED;
    }
    else if ((-motors.getLeftSpeed()) == motors.getRightSpeed())
    {
        driveDirection = DRIVE_DIRECTION_STOPPED;
    }
    else if ((0 <= motors.getLeftSpeed()) && (0 <= motors.getRightSpeed()))
    {
        driveDirection = DRIVE_DIRECTION_FORWARD;
    }
    else
    {
        driveDirection = DRIVE_DIRECTION_BACKWARD;
    }

    /* If a motor is stopped, it is assumed that the robot has no movement on the
     * corresponding motor side.
     * All other effects which may lead the robot to move are ignored for simplicity.
     * If the application needs more accuracy, the application shall handle that
     * by its own.
     *
     * If the driving direction changed, the relative encoders will be reset to
     * avoid using a invalid driving distance for speed calculation.
     */

    if ((0 == motors.getLeftSpeed()) || (m_driveDirection != driveDirection))
    {
        m_linearSpeedLeft = 0;
        m_timestampLeft   = timestamp;

        m_relEncLeft.setSteps(absEncStepsLeft);
    }
    /* Moved long enough to be able to calculate the linear speed? */
    else if (MIN_ENCODER_COUNT <= abs(diffStepsLeft))
    {
        uint32_t dTimeLeft = timestamp - m_timestampLeft; /* [ms] */

        m_linearSpeedLeft = diffStepsLeft * ONE_SECOND / static_cast<int32_t>(dTimeLeft);
        m_timestampLeft   = timestamp;

        m_relEncLeft.setSteps(absEncStepsLeft);
    }
    else
    {
        ;
    }

    if ((0 == motors.getRightSpeed()) || (m_driveDirection != driveDirection))
    {
        m_linearSpeedRight = 0;
        m_timestampRight   = timestamp;

        m_relEncRight.setSteps(absEncStepsRight);
    }
    /* Moved long enough to be able to calculate the linear speed? */
    else if (MIN_ENCODER_COUNT <= abs(diffStepsRight))
    {
        uint32_t dTimeRight = timestamp - m_timestampRight; /* [ms] */

        m_linearSpeedRight = diffStepsRight * ONE_SECOND / static_cast<int32_t>(dTimeRight);
        m_timestampRight   = timestamp;

        m_relEncRight.setSteps(absEncStepsRight);
    }
    else
    {
        ;
    }

    m_driveDirection = driveDirection;
}

int16_t Speedometer::getLinearSpeedCenter() const
{
    int32_t linearSpeedLeft   = static_cast<int32_t>(m_linearSpeedLeft);
    int32_t linearSpeedRight  = static_cast<int32_t>(m_linearSpeedRight);
    int32_t linearSpeedCenter = (linearSpeedLeft + linearSpeedRight) / 2;

    return linearSpeedCenter;
}

int16_t Speedometer::getLinearSpeedLeft() const
{
    return m_linearSpeedLeft;
}

int16_t Speedometer::getLinearSpeedRight() const
{
    return m_linearSpeedRight;
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
