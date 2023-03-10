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
 * @brief  Calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ParameterSets.h"

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

void ParameterSets::choose(uint8_t setId)
{
    if (MAX_SETS > setId)
    {
        m_currentSetId = setId;
    }
}

void ParameterSets::next()
{
    ++m_currentSetId;
    m_currentSetId %= MAX_SETS;
}

uint8_t ParameterSets::getCurrentSetId() const
{
    return m_currentSetId;
}

const ParameterSets::ParameterSet& ParameterSets::getParameterSet() const
{
    return m_parSets[m_currentSetId];
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

ParameterSets::ParameterSets() : m_currentSetId(0), m_parSets()
{
    m_parSets[0] = {
        "PID Slow", /* Name */
        1920,       /* Top speed in steps/s */
        3,          /* Kp Numerator */
        2,          /* Kp Denominator */
        1,          /* Ki Numerator */
        60,         /* Ki Denominator */
        4,          /* Kd Numerator */
        1           /* Kd Denominator */
    };

    m_parSets[1] = {
        "PID Fast", /* Name */
        2400,       /* Top speed in steps/s */
        3,          /* Kp Numerator */
        2,          /* Kp Denominator */
        1,          /* Ki Numerator */
        40,         /* Ki Denominator */
        10,         /* Kd Numerator */
        1           /* Kd Denominator */
    };

    m_parSets[2] = {
        "PD Fast", /* Name */
        2400,      /* Top speed in steps/s */
        3,         /* Kp Numerator */
        1,         /* Kp Denominator */
        0,         /* Ki Numerator */
        1,         /* Ki Denominator */
        10,        /* Kd Numerator */
        1          /* Kd Denominator */
    };
}

ParameterSets::~ParameterSets()
{
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
