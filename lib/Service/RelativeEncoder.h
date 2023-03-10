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
 * @brief  Relative encoder
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef RELATIVE_ENCODER_H
#define RELATIVE_ENCODER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Board.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * A relative encoder provides always the step difference between a reference
 * point (absolute number of encoder steps) to the current position (current
 * number of encoder steps).
 */
class RelativeEncoder
{
public:
    /**
     * Constructs a relative encoder.
     * The reference point is set to 0 steps.
     */
    RelativeEncoder() : m_lastSteps(0)
    {
    }

    /**
     * Constructs a relative encoder.
     * The reference point is assigned by the parameter.
     *
     * @param[in] steps Reference point (absolute encoder steps)
     */
    RelativeEncoder(int16_t steps) : m_lastSteps(steps)
    {
    }

    /**
     * Destroys the relative encoder instance.
     */
    ~RelativeEncoder()
    {
    }

    /**
     * Set the reference point of the encoder.
     * The delta steps are calculated from this.
     *
     * @param[in] steps Reference point (absolute number of encoder steps)
     */
    void setSteps(int16_t steps)
    {
        m_lastSteps = steps;
    }

    /**
     * Calculate the relative number of encoder steps from the given
     * absolute encoder step number to the reference point.
     *
     * @param[in] steps Current absolute number of encoder steps.
     *
     * @return Delta steps
     */
    int16_t calculate(int16_t steps);

private:
    int16_t m_lastSteps; /**< Last absolute number of encoder steps. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ENCODERS_H */