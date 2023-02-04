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
 * @addtogroup HALSim
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

#include <webots/Emitter.hpp>
#include <webots/DistanceSensor.hpp>

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
     *
     * @param[in] emitter0  The most left infrared emitter 0
     * @param[in] emitter1  The infrared emitter 1
     * @param[in] emitter2  The infrared emitter 2
     * @param[in] emitter3  The infrared emitter 3
     * @param[in] emitter4  The most right infrared emitter 4
     * @param[in] lightSensor0  The most left light sensor 0
     * @param[in] lightSensor1  The light sensor 1
     * @param[in] lightSensor2  The light sensor 2
     * @param[in] lightSensor3  The light sensor 3
     * @param[in] lightSensor4  The most right light sensor 4
     */
    LineSensors(webots::Emitter* emitter0, webots::Emitter* emitter1, webots::Emitter* emitter2,
                webots::Emitter* emitter3, webots::Emitter* emitter4, webots::DistanceSensor* lightSensor0,
                webots::DistanceSensor* lightSensor1, webots::DistanceSensor* lightSensor2, webots::DistanceSensor* lightSensor3,
                webots::DistanceSensor* lightSensor4) :
        ILineSensors(),
        m_sensorValuesU16(),
        m_emitter0(emitter0),
        m_emitter1(emitter1),
        m_emitter2(emitter2),
        m_emitter3(emitter3),
        m_emitter4(emitter4),
        m_lightSensor0(lightSensor0),
        m_lightSensor1(lightSensor1),
        m_lightSensor2(lightSensor2),
        m_lightSensor3(lightSensor3),
        m_lightSensor4(lightSensor4),
        m_sensorCalibSuccessfull(false),
        m_sensorCalibStarted(false),
        m_calibErrorInfo(CALIB_ERROR_NOT_CALIBRATED)
    {
        for (uint8_t sensorIndex = 0; sensorIndex < MAX_SENSORS; ++sensorIndex)
        {
            m_sensorMaxValues[sensorIndex] = 0;
            m_sensorMinValues[sensorIndex] = SENSOR_MAX_VALUE;
        }
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
        /* TODO */
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
        getSensorValues();
        
        for (uint8_t sensorIndex = 0; sensorIndex < MAX_SENSORS; ++sensorIndex)
        {
            if (m_sensorValuesU16[sensorIndex] < m_sensorMinValues[sensorIndex])
            {
                m_sensorMinValues[sensorIndex] = m_sensorValuesU16[sensorIndex];
            }
            if (m_sensorValuesU16[sensorIndex] > m_sensorMaxValues[sensorIndex])
            {
                m_sensorMaxValues[sensorIndex] = m_sensorValuesU16[sensorIndex];
            }
        }
        
        m_sensorCalibStarted = true;
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
     */
    int16_t readLine() final;

    /**
     * Get last line sensor values.
     *
     * @return Line sensor values
     */
    const uint16_t* getSensorValues() final;

    /**
     * Checks whether the calibration was successful or not.
     * It assumes that the environment brightness compensation is active.
     *
     * @return If successful, it will return true otherwise false.
     */
    bool isCalibrationSuccessful() final
    {
        bool isSuccessful = false;

        m_calibErrorInfo = CALIB_ERROR_NOT_CALIBRATED;
        
        if (true == m_sensorCalibStarted)
        {
            uint8_t index = 0;

            isSuccessful = true;
            while((MAX_SENSORS > index) && (true == isSuccessful))
            {
                uint16_t distance = 0;
                
                /* Check whether the max. value is really greater than the min. value.
                * It can happen that someone try to calibrate over a blank surface.
                */
                if (m_sensorMaxValues[index] > m_sensorMinValues[index])
                {
                    distance = m_sensorMaxValues[index] - m_sensorMinValues[index];
                }

                /* The assumption here is, that the distance (max. value - min. value) must be
                * higher than a quarter of the max. measure duration.
                */
                if ((SENSOR_MAX_VALUE / 4) > distance)
                {
                    m_calibErrorInfo = index;
                    isSuccessful = false;
                }

                ++index;
            }
        }

        return isSuccessful;
    }

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
    static const uint8_t MAX_SENSORS = 5;

    /**
     * Max. value of a single line sensor in digits.
     * It depends on the Zumo32U4LineSensors implementation.
     */
    static const int16_t SENSOR_MAX_VALUE = 1000;

    uint16_t m_sensorValuesU16[MAX_SENSORS]; /**< The last value of each sensor as unsigned 16-bit values. */

    webots::Emitter*     m_emitter0;     /**< The infrared emitter 0 (most left) */
    webots::Emitter*     m_emitter1;     /**< The infrared emitter 1 */
    webots::Emitter*     m_emitter2;     /**< The infrared emitter 2 */
    webots::Emitter*     m_emitter3;     /**< The infrared emitter 3 */
    webots::Emitter*     m_emitter4;     /**< The infrared emitter 4 (most right) */

    webots::DistanceSensor* m_lightSensor0; /**< The light sensor 0 (most left) */
    webots::DistanceSensor* m_lightSensor1; /**< The light sensor 1 */
    webots::DistanceSensor* m_lightSensor2; /**< The light sensor 2 */
    webots::DistanceSensor* m_lightSensor3; /**< The light sensor 3 */
    webots::DistanceSensor* m_lightSensor4; /**< The light sensor 4 (most right) */

    bool m_sensorCalibSuccessfull;  /**< Indicates weather the calibration was successfull or not. */
    bool m_sensorCalibStarted;      /**< Indicates weather the calibration has started or not. */
    uint8_t m_calibErrorInfo;       /**< Indicates which sensor failed the calibration, if the calibration failed. */

    uint16_t m_sensorMinValues[MAX_SENSORS]; /**< Stores the minimal calibration values for the sensors. */
    uint16_t m_sensorMaxValues[MAX_SENSORS]; /**< Stores the minimal calibration values for the sensors. */

    /* Default constructor not allowed. */
    LineSensors();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINESENSORS_H */