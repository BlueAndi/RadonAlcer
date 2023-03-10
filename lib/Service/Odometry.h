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
 * @brief  Odometry unit
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <RelativeEncoder.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * This class provides odometry data, based on the encoder informations.
 * Odometry data:
 * - orientation and delta orientation
 * - position and delta position
 * - mileage
 */
class Odometry
{
public:
    /**
     * Get odometry instance.
     *
     * @return Odometry instance.
     */
    static Odometry& getInstance()
    {
        static Odometry instance; /* idiom */

        return instance;
    }

    /**
     * Process the odometry unit periodically.
     */
    void process();

    /**
     * Get center mileage in mm.
     *
     * @return Mileage in mm
     */
    uint32_t getMileageCenter() const;

    /**
     * Get current orientation in mrad.
     * 
     * @return Orientation in mrad
     */
    int16_t getOrientation() const
    {
        return m_orientation;
    }

private:
    /** Odometry instance */
    static Odometry m_instance;

    /**
     * Last number of relative encoder steps left. Its used to avoid permanent
     * clearing of the relative encoders.
     */
    uint16_t m_lastAbsRelEncStepsLeft;

    /**
     * Last number of relative encoder steps right. Its used to avoid permanent
     * clearing of the relative encoders.
     */
    uint16_t m_lastAbsRelEncStepsRight;

    /** Absolute number of encoder steps left. Used to determine the mileage. */
    uint32_t m_absEncStepsLeft;

    /** Absolute number of encoder steps right. Used to determine the mileage. */
    uint32_t m_absEncStepsRight;

    /** Relative encoder left. */
    RelativeEncoder m_relEncLeft;

    /** Relative encoder right. */
    RelativeEncoder m_relEncRight;

    /** Absolute orientation in mrad. 0 mrad means the robot drives parallel to the y-axis.  */
    int16_t m_orientation;

    /** Last motor speed value left in digits. Necessary to determine delta calculation. */
    int16_t m_lastMotorSpeedLeft;

    /** Last motor speed value right in digits. Necessary to determine delta calculation. */
    int16_t m_lastMotorSpeedRight;

    /**
     * Construct the odometry instance.
     */
    Odometry() :
        m_lastAbsRelEncStepsLeft(0),
        m_lastAbsRelEncStepsRight(0),
        m_absEncStepsLeft(0),
        m_absEncStepsRight(0),
        m_relEncLeft(),
        m_relEncRight(),
        m_orientation(0),
        m_lastMotorSpeedLeft(0),
        m_lastMotorSpeedRight(0)
    {
    }

    /**
     * Destroy the odometry instance.
     */
    ~Odometry()
    {
    }

    Odometry(const Odometry& value);
    Odometry& operator=(const Odometry& value);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ODOMETRY_H */