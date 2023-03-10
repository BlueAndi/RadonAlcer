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
 * @brief  Line sensors array realization
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef LINESENSORS_H
#define LINESENSORS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ILineSensors.h"
#include "Zumo32U4.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the Zumo target line sensors. */
class LineSensors : public ILineSensors
{
public:
    /**
     * Constructs the line sensors adapter.
     */
    LineSensors() : ILineSensors(), m_sensorValues(), m_sensorValuesU16(), m_calibErrorInfo(CALIB_ERROR_NOT_CALIBRATED)
    {
    }

    /**
     * Destroys the line sensors adapter.
     */
    ~LineSensors()
    {
    }

    /**
     * Initializes the line sensors.
     */
    void init() final
    {
        m_lineSensors.initFiveSensors();
    }

    /**
     * Reads the sensors for calibration. Call this method several times during
     * turning the sensors over the line to determine the minimum and maximum
     * values.
     *
     * The calibration factors are stored internally.
     */
    void calibrate() final
    {
        /* The environment brightness compensation is enabled and must be considered
         * during calibration as well.
         * See LineSensors::readLine()
         */
        m_lineSensors.calibrate(QTR_EMITTERS_ON_AND_OFF);
    }

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
    int16_t readLine() final
    {
        /* The environment brightness compensation is enabled by using QTR_EMITTERS_ON_AND_OFF.
         * This is done by measuring the sensor on values with enabled IR emitters. After that
         * the IR emitters are disabled and the sensor off values are measured.
         * The compensation is quite simple: compenstated value = on value - off value
         */
        return m_lineSensors.readLine(m_sensorValues, QTR_EMITTERS_ON_AND_OFF);
    }

    /**
     * Get last line sensor values.
     *
     * @return Line sensor values
     */
    const uint16_t* getSensorValues() final
    {
        uint8_t index = 0;

        /* This is only done to be able to provide a platform independed data type. */
        for (index = 0; index < MAX_SENSORS; ++index)
        {
            m_sensorValuesU16[index] = static_cast<uint16_t>(m_sensorValues[index]);
        }

        return m_sensorValuesU16;
    }

    /**
     * Checks whether the calibration was successful or not.
     * It assumes that the environment brightness compensation is active.
     * 
     * @return If successful, it will return true otherwise false.
     */
    bool isCalibrationSuccessful() final;

    /**
     * It will return the index of the sensor, which caused to fail the calibration.
     * If calibration was successful, it will return 0xFF.
     * If calibration was not not done yet, it will return 0xFE.
     * 
     * @return Sensor index, starting with 0. Note the other cases in description.
     */
    uint8_t getCalibErrorInfo() const final
    {
        return m_calibErrorInfo;
    }

    /**
     * Get number of used line sensors.
     *
     * @return Number of used line sensors
     */
    uint8_t getNumLineSensors() const final
    {
        return MAX_SENSORS;
    }

    /**
     * Get max. value of a single line sensor in digits.
     * The sensor value is indirect proportional to the reflectance.
     *
     * @return Max. line sensor value
     */
    uint16_t getSensorValueMax() const final
    {
        return SENSOR_MAX_VALUE;
    }

private:
    /**
     * Number of used line sensors. This depends on the Zumo hardware configuration.
     */
    static const uint8_t    MAX_SENSORS         = 5;

    /**
     * Max. value of a single line sensor in digits (calibration already considered).
     * It depends on the Zumo32U4LineSensors implementation.
     * See Zumo32U4\QTRSensors.cpp @ readCalibrated()
     */
    static const int16_t    SENSOR_MAX_VALUE    = 1000;

    /**
     * Max. value for a single line sensor in us. In other words, it is the max. measure duration.
     * Note, the measured time is indirect proportional to the measured reflectance.
     * That means a black line is detected in case of the measured value is around this max. value.
     * For the value see Zumo32U4\Zumo32U4LineSensors.h @ initFiveSensors()
     */
    static const uint16_t   MEASURE_DURATION    = 2000;

    Zumo32U4LineSensors m_lineSensors;                  /**< Zumo line sensors driver from Pololu */
    unsigned int        m_sensorValues[MAX_SENSORS];    /**< The last value of each sensor. */
    uint16_t            m_sensorValuesU16[MAX_SENSORS]; /**< The last value of each sensor as unsigned 16-bit values. */
    uint8_t             m_calibErrorInfo;               /**< Calibration error information. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINESENSORS_H */