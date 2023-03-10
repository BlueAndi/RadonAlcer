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
 * @brief  Abstract line sensors array interface
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup HALInterfaces
 *
 * @{
 */
#ifndef ILINESENSORS_H
#define ILINESENSORS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The abstract line sensors interface. */
class ILineSensors
{
public:
    /**
     * Destroys the interface.
     */
    virtual ~ILineSensors()
    {
    }

    /**
     * Initializes the line sensors.
     */
    virtual void init() = 0;

    /**
     * Reads the sensors for calibration. Call this method several times during
     * turning the sensors over the line to determine the minimum and maximum
     * values.
     *
     * The calibration factors are stored internally.
     */
    virtual void calibrate() = 0;

    /**
     * Determines the deviation and returns an estimated position of the robot
     * with respect to a line. The estimate is made using a weighted average of
     * the sensor indices multiplied by 1000, so that a return value of 0
     * indicates that the line is directly below sensor 0, a return value of
     * 1000 indicates that the line is directly below sensor 1, 2000
     * indicates that it's below sensor 2000, etc.  Intermediate values
     * indicate that the line is between two sensors. The formula is:
     *
     *   0*value0 + 1000*value1 + 2000*value2 + ...
     *  --------------------------------------------
     *      value0  +  value1  +  value2 + ...
     *
     * This function assumes a dark line (high values) surrounded by white
     * (low values).
     * 
     * @return Estimated position with respect to track.
     */
    virtual int16_t readLine() = 0;

    /**
     * Get last line sensor values.
     *
     * @return Line sensor values
     */
    virtual const uint16_t* getSensorValues() = 0;

    /**
     * Checks whether the calibration was successful or not.
     * It assumes that the environment brightness compensation is active.
     * 
     * @return If successful, it will return true otherwise false.
     */
    virtual bool isCalibrationSuccessful() = 0;

    /**
     * It will return the index of the sensor, which caused to fail the calibration.
     * If calibration was successful, it will return 0xFF.
     * If calibration was not not done yet, it will return 0xFE.
     * 
     * @return Sensor index, starting with 0. Note the other cases in description.
     */
    virtual uint8_t getCalibErrorInfo() const = 0;

    /**
     * Get number of used line sensors.
     *
     * @return Number of used line sensors
     */
    virtual uint8_t getNumLineSensors() const = 0;

    /**
     * Get max. value of a single line sensor in digits.
     * The sensor value is indirect proportional to the reflectance.
     *
     * @return Max. line sensor value
     */
    virtual uint16_t getSensorValueMax() const = 0;

    /**
     * Calibration error information: Calibration successful.
     */
    static const uint8_t    CALIB_ERROR_OK              = 0xFF;

    /**
     * Calibration error information: Calibration not done yet.
     */
    static const uint8_t    CALIB_ERROR_NOT_CALIBRATED  = 0xFE;

protected:
    /**
     * Constructs the interface.
     */
    ILineSensors()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ILINESENSORS_H */