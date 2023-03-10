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
 * @brief  Mileage
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Mileage.h>
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

Mileage Mileage::m_instance;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void Mileage::clear()
{
    IEncoders& encoders = Board::getInstance().getEncoders();

    m_encoderStepsLeft  = 0;
    m_encoderStepsRight = 0;

    m_relEncLeft.setSteps(encoders.getCountsLeft());
    m_relEncRight.setSteps(encoders.getCountsRight());
}

void Mileage::process()
{
    if (false == m_timer.isRunning())
    {
        m_timer.start(MIN_PERIOD);
    }
    else if (true == m_timer.isTimeout())
    {
        IEncoders& encoders      = Board::getInstance().getEncoders();
        int16_t    stepsLeft     = m_relEncLeft.calculate(encoders.getCountsLeft());
        int16_t    stepsRight    = m_relEncRight.calculate(encoders.getCountsRight());
        uint16_t   absStepsLeft  = abs(stepsLeft);
        uint16_t   absStepsRight = abs(stepsRight);

        /* Calculate absolute accumulated number steps for the left encoder. */
        m_encoderStepsLeft += absStepsLeft;

        /* Calculate absolute accumulated number steps for the left encoder. */
        m_encoderStepsRight += absStepsRight;

        /* Clear relative encoders */
        m_relEncLeft.setSteps(encoders.getCountsLeft());
        m_relEncRight.setSteps(encoders.getCountsRight());

        m_timer.restart();
    }
    else
    {
        /* No processing. */
        ;
    }
}

uint32_t Mileage::getMileageCenter() const
{
    uint32_t encoderSteps = (m_encoderStepsLeft + m_encoderStepsRight) / 2;

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
