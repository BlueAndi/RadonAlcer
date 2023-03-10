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
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <RelativeEncoder.h>
#include <RobotConstants.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides the linear speed in [steps/s], based on the encoder informations. */
class Speedometer
{
public:
    /**
     * Get speedometer instance.
     *
     * @return Speedometer instance.
     */
    static Speedometer& getInstance()
    {
        return m_instance;
    }

    /**
     * Update linear speed, based on the measured encoder steps.
     * Call this function cyclic.
     */
    void process();

    /**
     * Get linear speed center in steps/s.
     *
     * @return Linear speed center in steps/s
     */
    int16_t getLinearSpeedCenter() const;

    /**
     * Get linear speed left in steps/s.
     *
     * @return Linear speed left in steps/s
     */
    int16_t getLinearSpeedLeft() const;

    /**
     * Get linear speed right in steps/s.
     *
     * @return Linear speed right in steps/s
     */
    int16_t getLinearSpeedRight() const;

private:
    /** Possible drive directions */
    enum DriveDirection
    {
        DRIVE_DIRECTION_STOPPED = 0, /**< No movement */
        DRIVE_DIRECTION_FORWARD,     /**< Moves forward */
        DRIVE_DIRECTION_BACKWARD     /**< Moves backward */
    };

    /**
     * The minimum number of counted encoder steps until the speed is measured.
     * It shall avoid a noisy speed.
     */
    static const int16_t MIN_ENCODER_COUNT = RobotConstants::ENCODER_RESOLUTION / 2;

    /** Speedometer instance */
    static Speedometer m_instance;

    /** Relative encoder left */
    RelativeEncoder m_relEncLeft;

    /** Relative encoder right */
    RelativeEncoder m_relEncRight;

    /** Timestamp of last left speed calculation. */
    uint32_t m_timestampLeft;

    /** Timestamp of last right speed calculation. */
    uint32_t m_timestampRight;

    /** Linear speed left in steps/s */
    int16_t m_linearSpeedLeft;

    /** Linear speed right in steps/s */
    int16_t m_linearSpeedRight;

    /** Current drive direction */
    DriveDirection m_driveDirection;

    /**
     * Construct the mileage instance.
     */
    Speedometer() :
        m_relEncLeft(),
        m_relEncRight(),
        m_timestampLeft(0),
        m_timestampRight(0),
        m_linearSpeedLeft(0),
        m_linearSpeedRight(0),
        m_driveDirection(DRIVE_DIRECTION_STOPPED)
    {
    }

    /**
     * Destroy the mileage instance.
     */
    ~Speedometer()
    {
    }

    Speedometer(const Speedometer& value);
    Speedometer& operator=(const Speedometer& value);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SPEEDOMETER_H */