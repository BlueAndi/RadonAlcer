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
 * @brief  Differential drive
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <SimpleTimer.h>
#include <PIDController.h>
#include <RelativeEncoder.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The differential drive control considers a two wheeled robot.
 *
 * Drive the robot by controlling the center linear speed or the
 * angular speed.
 *
 * The left and right speed can be controlled separately as well.
 *
 * All values used for control and measurement are in [steps/s] or [mrad/s].
 *
 * Calculations are performed in fixed point arithmetic for better performance.
 */
class DifferentialDrive
{
public:
    /**
     * Get the instance of the differential drive control.
     *
     * @return Instance
     */
    static DifferentialDrive& getInstance()
    {
        static DifferentialDrive instance; /* idiom */

        return instance;
    }

    /**
     * Enable the differential drive control.
     * It will control the motors during processing if the max. speed is available too.
     *
     * The linear and angular speeds will be set to 0. This avoid any
     * unexpected driving behaviour. The internal PID controllers will
     * be reset.
     */
    void enable()
    {
        m_linearSpeedCenterSetPoint = 0;
        m_linearSpeedLeftSetPoint   = 0;
        m_linearSpeedRightSetPoint  = 0;
        m_angularSpeedSetPoint      = 0;

        m_motorSpeedLeftPID.clear();
        m_motorSpeedRightPID.clear();

        m_isEnabled = true;
    }

    /**
     * Disable the differential drive control.
     * It will stop the motors to avoid any unexpected driving behaviour.
     * After that the motors are not controlled anymore.
     * 
     * Important: Processing the differential drive control shall not be stopped.
     */
    void disable()
    {
        m_linearSpeedCenterSetPoint = 0;
        m_linearSpeedLeftSetPoint   = 0;
        m_linearSpeedRightSetPoint  = 0;
        m_angularSpeedSetPoint      = 0;

        m_isEnabled = false;
    }

    /**
     * Get the max. motor speed in [steps/s].
     * If 0 is returned, it means there was no calibration yet.
     *
     * @return Max. motor speed [steps/s]
     */
    int16_t getMaxMotorSpeed() const;

    /**
     * Set the max. motor speed in [steps/s].
     * Determine the max. motor speed by calibration.
     *
     * @param[in] maxMotorSpeed Max. motor speed in [steps/s]
     */
    void setMaxMotorSpeed(int16_t maxMotorSpeed);

    /**
     * Get the linear speed center set point.
     * Note, this is the speed which is commanded and not the measured one.
     *
     * @return Linear speed center [steps/s]
     */
    int16_t getLinearSpeed() const;

    /**
     * Set the linear speed center in [steps/s].
     * The speed will be limited to the max. motor speed.
     *
     * @param[in] linearSpeed   Linear speed center [steps/]
     */
    void setLinearSpeed(int16_t linearSpeed);

    /**
     * Gett the linear speed left and right set points.
     * Note, these are the speeds which are commanded and not the measured ones.
     *
     * @param[in] linearSpeedLeft   Linear speed left set point [steps/s]
     * @param[in] linearSpeedRight  Linear speed right set point [steps/s]
     */
    void getLinearSpeed(int16_t& linearSpeedLeft, int16_t& linearSpeedRight);

    /**
     * Set the linear speed left and right in [steps/s].
     * The speed will be limited to the max. motor speed.
     *
     * If left and right speed values are equal, it will drive straight forward
     * otherwise will turn into left or right direction.
     *
     * @param[in] linearSpeedLeft   Linear speed left [steps/s]
     * @param[in] linearSpeedRight  Linear speed right [steps/s]
     */
    void setLinearSpeed(int16_t linearSpeedLeft, int16_t linearSpeedRight);

    /**
     * Get the angular speed set point.
     * Note, this is the speed which is commanded and not the measured one.
     *
     * @return Angular speed [mrad/s]
     */
    int16_t getAngularSpeed() const;

    /**
     * Set the angular speed in [mrad/s].
     *
     * @param[in] angularSpeed  Angular speed [mrad/s]
     */
    void setAngularSpeed(int16_t angularSpeed);

    /**
     * Process the differential drive periodically.
     *
     * @param[in] period    Calling period in [ms]
     */
    void process(uint32_t period);

private:
    /**
     * The PID proportional factor numerator for the speed control.
     */
    static const int16_t PID_P_NUMERATOR = 1;

    /**
     * The PID proportional factor denominator for the speed control.
     */
    static const int16_t PID_P_DENOMINATOR = 1;

    /**
     * The PID integral factor numerator for the speed control.
     */
    static const int16_t PID_I_NUMERATOR = 1;

    /**
     * The PID integral factor denominator for the speed control.
     */
    static const int16_t PID_I_DENOMINATOR = 40;

    /**
     * The PID derivative factor numerator for the speed control.
     */
    static const int16_t PID_D_NUMERATOR = 1;

    /**
     * The PID derivative factor denominator for the speed control.
     */
    static const int16_t PID_D_DENOMINATOR = 200;

    int16_t m_isInit;    /**< Used to determine the initialization in the first time process() is called. */
    bool    m_isEnabled; /**< Enable/Disable the differential drive control. */

    int16_t m_maxMotorSpeed; /**< Max. motor speed [steps/s] */

    int16_t m_linearSpeedCenterSetPoint; /**< Linear speed central in [steps/s] set point */
    int16_t m_angularSpeedSetPoint;      /**< Angular speed central in [mrad/s set] point */

    int16_t m_linearSpeedLeftSetPoint;  /**< Linear speed left in [steps/s] set point */
    int16_t m_linearSpeedRightSetPoint; /**< Linear speed right in [steps/s] set point */

    PIDController<int16_t> m_motorSpeedLeftPID;  /**< PID controller for the left motor speed. */
    PIDController<int16_t> m_motorSpeedRightPID; /**< PID controller for the right motor speed. */

    int32_t m_lastLinearSpeedLeft;  /**< Last linear speed left PID output in [steps/s]. */
    int32_t m_lastLinearSpeedRight; /**< Last linear speed right PID output in [steps/s]. */

    /**
     * Construct differential drive control.
     * It is disabled by default.
     */
    DifferentialDrive() :
        m_isInit(false),
        m_isEnabled(false),
        m_maxMotorSpeed(0),
        m_linearSpeedCenterSetPoint(0),
        m_angularSpeedSetPoint(0),
        m_linearSpeedLeftSetPoint(0),
        m_linearSpeedRightSetPoint(0),
        m_motorSpeedLeftPID(),
        m_motorSpeedRightPID(),
        m_lastLinearSpeedLeft(0),
        m_lastLinearSpeedRight(0)
    {
        m_motorSpeedLeftPID.setPFactor(PID_P_NUMERATOR, PID_P_DENOMINATOR);
        m_motorSpeedLeftPID.setIFactor(PID_I_NUMERATOR, PID_I_DENOMINATOR);
        m_motorSpeedLeftPID.setDFactor(PID_D_NUMERATOR, PID_D_DENOMINATOR);

        m_motorSpeedRightPID.setPFactor(PID_P_NUMERATOR, PID_P_DENOMINATOR);
        m_motorSpeedRightPID.setIFactor(PID_I_NUMERATOR, PID_I_DENOMINATOR);
        m_motorSpeedRightPID.setDFactor(PID_D_NUMERATOR, PID_D_DENOMINATOR);
    }

    /**
     * Destroy differential drive control.
     */
    ~DifferentialDrive()
    {
        /* Never called. */
    }

    /* Not allowed. */
    DifferentialDrive(const DifferentialDrive& diffDrive);
    DifferentialDrive& operator=(const DifferentialDrive& diffDrive);

    /**
     * Calculate the linear speed left and right from the linear speed center and
     * the angular speed.
     *
     * @param[in]   linearSpeedCenter   Linear speed center in [steps/s]
     * @param[in]   angularSpeed        Angular speed in [mrad/s]
     * @param[out]  linearSpeedLeft     Linear speed left in [steps/s]
     * @param[out]  linearSpeedRight    Linear speed right in [steps/s]
     */
    void calculateLinearSpeedLeftRight(int16_t linearSpeedCenter, int16_t angularSpeed, int16_t& linearSpeedLeft,
                                       int16_t& linearSpeedRight);

    /**
     * Calculate the linear speed center and the angular speed from the
     * linear speed left and right.
     *
     * @param[in]   linearSpeedLeft     Linear speed left in [steps/s]
     * @param[in]   linearSpeedRight    Linear speed right in [steps/s]
     * @param[out]  linearSpeedCenter   Linear speed center in [steps/s]
     * @param[out]  angularSpeed        Angular speed in [mrad/s]
     */
    void calculateLinearAndAngularSpeedCenter(int16_t linearSpeedLeft, int16_t linearSpeedRight,
                                              int16_t& linearSpeedCenter, int16_t& angularSpeed);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* DIFFERENTIAL_DRIVE_H */