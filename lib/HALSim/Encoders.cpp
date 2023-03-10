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
 * @brief  Encoders realization
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Encoders.h"
#include "RobotConstants.h"

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

int16_t Encoders::getCountsLeft()
{
    int16_t steps = 0;

    /* The position sensor provides the driven distance in [m].
     * The encoder steps are dervived from it.
     */
    if (nullptr != m_posSensorLeft)
    {
        steps = calculateSteps(m_lastResetValueLeft, m_posSensorLeft->getValue());
    }

    return steps;
}

int16_t Encoders::getCountsRight()
{
    int16_t steps = 0;

    /* The position sensor provides the driven distance in [m].
     * The encoder steps are dervived from it.
     */
    if (nullptr != m_posSensorRight)
    {
        steps = calculateSteps(m_lastResetValueRight, m_posSensorRight->getValue());
    }

    return steps;
}

int16_t Encoders::getCountsAndResetLeft()
{
    int16_t steps = getCountsLeft();

    m_lastResetValueLeft = m_posSensorLeft->getValue();

    return steps;
}

int16_t Encoders::getCountsAndResetRight()
{
    int16_t steps = getCountsRight();

    m_lastResetValueRight = m_posSensorRight->getValue();

    return steps;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

int16_t Encoders::calculateSteps(double lastPos, double pos) const
{
    int16_t steps = 0;

    /* Position sensor provides NAN until the first simulation step was perfomed. */
    if (false == isnan(pos))
    {
        double deltaPos     = (pos - lastPos) * 1000.0f;                                            /* [mm] */
        double encoderSteps = deltaPos * static_cast<double>(RobotConstants::ENCODER_STEPS_PER_MM); /* [steps] */

        /* Simulate int16_t wrap around. */
        if (static_cast<double>(INT16_MAX) < encoderSteps)
        {
            encoderSteps = static_cast<double>(INT16_MIN) + encoderSteps - static_cast<double>(INT16_MAX);
        }
        else if (static_cast<double>(INT16_MIN) > encoderSteps)
        {
            encoderSteps = static_cast<double>(INT16_MIN) - encoderSteps + static_cast<double>(INT16_MAX);
        }
        else
        {
            ;
        }

        steps = static_cast<int16_t>(encoderSteps);
    }

    return steps;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
