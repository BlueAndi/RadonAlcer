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
 * @brief  Robot specific constants
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef ROBOTCONSTANTS_H
#define ROBOTCONSTANTS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Abstracts the physical robot constants.
 */
namespace RobotConstants
{
    /**
     * Gear ratio multiplied with 1000.
     */
    static const uint32_t   GEAR_RATIO              = 75810;

    /**
     * Encoder resolution in counts per revolution of the motor shaft.
     */
    static const uint16_t   ENCODER_RESOLUTION      = 12;

    /**
     * Calibrated wheel diameter in mm.
     * This means the real wheel diameter was adapted after calibration drive.
     */
    static const uint32_t   WHEEL_DIAMETER          = 36;

    /**
     * Wheel circumference in um.
     */
    static const uint32_t   WHEEL_CIRCUMFERENCE     = static_cast<uint32_t>(static_cast<float>(WHEEL_DIAMETER) * PI * 1000.0f);

    /**
     * Wheel base in mm.
     * Distance between the left wheel center to the right wheel center.
     */
    static const uint32_t   WHEEL_BASE              = 90;

    /**
     * Number of encoder steps per mm.
     */
    static const uint32_t   ENCODER_STEPS_PER_MM    = (ENCODER_RESOLUTION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ROBOTCONSTANTS_H */