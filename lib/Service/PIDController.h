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
 * @brief  PID regulator
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Meta template to get the min. or max. value of a template type.
 *
 * @tparam T The datatype, where the min. or max. value is needed.
 */
template<typename T>
struct Type
{
    /* Empty, to get compiler error for unsupported datatypes. */
};

/**
 * Meta template specialization to get the min. or max. value of a int8_t.
 *
 * @tparam T The datatype, where the min. or max. value is needed.
 */
template<>
struct Type<int8_t>
{
    /** Enumeration used to determine the min. value. */
    enum Min
    {
        MIN_RESULT = INT8_MIN /**< Minimum result of datatype */
    };

    /** Enumeration used to determine the max. value. */
    enum Max
    {
        MAX_RESULT = INT8_MAX /**< Maximum result of datatype. */
    };
};

/**
 * Meta template specialization to get the min. or max. value of a int16_t.
 *
 * @tparam T The datatype, where the min. or max. value is needed.
 */
template<>
struct Type<int16_t>
{
    /** Enumeration used to determine the min. value. */
    enum Min
    {
        MIN_RESULT = INT16_MIN /**< Minimum result of datatype */
    };

    /** Enumeration used to determine the max. value. */
    enum Max
    {
        MAX_RESULT = INT16_MAX /**< Maximum result of datatype. */
    };
};

/**
 * Meta template specialization to get the min. or max. value of a int32_t.
 *
 * @tparam T The datatype, where the min. or max. value is needed.
 */
template<>
struct Type<int32_t>
{
    /** Enumeration used to determine the min. value. */
    enum Min
    {
        MIN_RESULT = INT32_MIN /**< Minimum result of datatype */
    };

    /** Enumeration used to determine the max. value. */
    enum Max
    {
        MAX_RESULT = INT32_MAX /**< Maximum result of datatype. */
    };
};

/**
 * A proportional–integral–derivative controller (PID controller).
 * It uses fixed point arithmetic for better performance.
 *
 * e(t) = sp(t) - pv(t)
 * co(t) = Kp * e(t) + Ki * Integral(e(t) dt) + Kd * e(t) / dt
 *
 * e(t):  Error
 * sp(t): Set Point
 * pv(t): Process Value
 * co(t): Controller Output
 *
 * Derivate on error: Kd * e(t) / dt
 * Derivate on measurement: Kd * -pv(t)
 *
 * @tparam T The datatype used for PID calculation.
 */
template<typename T>
class PIDController
{
public:
    /**
     * Constructs the PID controller.
     * Derivative on measurement is disabled.
     */
    PIDController() :
        m_kPNumerator(0),
        m_kPDenominator(1),
        m_kINumerator(0),
        m_kIDenominator(1),
        m_kDNumerator(0),
        m_kDDenominator(1),
        m_kINumeratorDT(0),
        m_kIDenominatorDT(SAMPLE_TIME_DEFAULT),
        m_kDNumeratorDT(0),
        m_kDDenominatorDT(SAMPLE_TIME_DEFAULT),
        m_min(Type<T>::MIN_RESULT),
        m_max(Type<T>::MAX_RESULT),
        m_lastError(0),
        m_integral(0),
        m_lastOutput(0),
        m_sampleTime(SAMPLE_TIME_DEFAULT),
        m_resync(false),
        m_isDerivativeOnMeasurement(false),
        m_lastProcessValue(0)
    {
        denominatorsShallNotBeZero();
    }

    /**
     * Constructs the PID controller.
     * Note, the factors are sample time (dT) depended.
     * Derivative on measurement is disabled.
     *
     * @param[in] kPNumerator   Numerator of proportional factor
     * @param[in] kPDenominator Denominator of proportional factor
     * @param[in] kINumerator   Numerator of integral factor
     * @param[in] kIDenominator Denominator of integral factor
     * @param[in] kDNumerator   Numerator of derivative factor
     * @param[in] kDDenominator Denominator of derivative factor
     * @param[in] max           Max. output value, used for limiting.
     * @param[in] min           Min. output value, used for limiting.
     */
    PIDController(T kPNumerator, T kPDenominator, T kINumerator, T kIDenominator, T kDNumerator, T kDDenominator, T max,
                  T min) :
        m_kPNumerator(kPNumerator),
        m_kPDenominator(kPDenominator),
        m_kINumerator(kINumerator),
        m_kIDenominator(kIDenominator),
        m_kDNumerator(kDNumerator),
        m_kDDenominator(kDDenominator),
        m_kINumeratorDT(kINumerator),
        m_kIDenominatorDT(kIDenominator * SAMPLE_TIME_DEFAULT),
        m_kDNumeratorDT(kDNumerator),
        m_kDDenominatorDT(kDDenominator * SAMPLE_TIME_DEFAULT),
        m_min(min),
        m_max(max),
        m_lastError(0),
        m_integral(0),
        m_lastOutput(0),
        m_sampleTime(SAMPLE_TIME_DEFAULT),
        m_resync(false),
        m_isDerivativeOnMeasurement(false),
        m_lastProcessValue(0)
    {
        denominatorsShallNotBeZero();
        reduceFraction(m_kPNumerator, m_kPDenominator);
        reduceFraction(m_kINumerator, m_kIDenominator);
        reduceFraction(m_kDNumerator, m_kDDenominator);
        reduceFraction(m_kINumeratorDT, m_kIDenominatorDT);
        reduceFraction(m_kDNumeratorDT, m_kDDenominatorDT);
    }

    /**
     * Destroys the PID controller.
     */
    ~PIDController()
    {
    }

    /**
     * Constructs the PID controller by copying another one.
     *
     * @param[in] ctrl  PID controller, which to copy
     */
    PIDController(const PIDController& ctrl) :
        m_kPNumerator(ctrl.m_kPNumerator),
        m_kPDenominator(ctrl.m_kPDenominator),
        m_kINumerator(ctrl.m_kINumerator),
        m_kIDenominator(ctrl.m_kIDenominator),
        m_kDNumerator(ctrl.m_kDNumerator),
        m_kDDenominator(ctrl.m_kDDenominator),
        m_kINumeratorDT(ctrl.m_kINumeratorDT),
        m_kIDenominatorDT(ctrl.m_kIDenominatorDT),
        m_kDNumeratorDT(ctrl.m_kDNumeratorDT),
        m_kDDenominatorDT(ctrl.m_kDDenominatorDT),
        m_min(ctrl.m_min),
        m_max(ctrl.m_max),
        m_lastError(ctrl.m_lastError),
        m_integral(ctrl.m_integral),
        m_lastOutput(ctrl.m_lastOutput),
        m_sampleTime(ctrl.m_sampleTime),
        m_resync(ctrl.m_resync),
        m_isDerivativeOnMeasurement(ctrl.m_isDerivativeOnMeasurement),
        m_lastProcessValue(ctrl.m_lastProcessValue)
    {
    }

    /**
     * Assign a PID controller.
     *
     * @param[in] ctrl PID controller, which to assign
     *
     * @return PID controller
     */
    PIDController& operator=(const PIDController& ctrl)
    {
        if (this != &ctrl)
        {
            m_kPNumerator               = ctrl.m_kPNumerator;
            m_kPDenominator             = ctrl.m_kPDenominator;
            m_kINumerator               = ctrl.m_kINumerator;
            m_kIDenominator             = ctrl.m_kIDenominator;
            m_kDNumerator               = ctrl.m_kDNumerator;
            m_kDDenominator             = ctrl.m_kDDenominator;
            m_kINumeratorDT             = ctrl.m_kINumeratorDT;
            m_kIDenominatorDT           = ctrl.m_kIDenominatorDT;
            m_kDNumeratorDT             = ctrl.m_kDNumeratorDT;
            m_kDDenominatorDT           = ctrl.m_kDDenominatorDT;
            m_min                       = ctrl.m_min;
            m_max                       = ctrl.m_max;
            m_lastError                 = ctrl.m_lastError;
            m_integral                  = ctrl.m_integral;
            m_lastOutput                = ctrl.m_lastOutput;
            m_sampleTime                = ctrl.m_sampleTime;
            m_resync                    = ctrl.m_resync;
            m_isDerivativeOnMeasurement = ctrl.m_isDerivativeOnMeasurement;
            m_lastProcessValue          = ctrl.m_lastProcessValue;
        }

        return *this;
    }

    /**
     * Process the PID controller and calculate the output according to the
     * measured process value. The process value is handled periodically, set
     * by sample time. If the calculation is called more often, the last
     * controller output will be returned, until next sample point. This way
     * it shall ensure a constant dT.
     *
     * @param[in] setpoint      Setpoint
     * @param[in] processValue  Process value
     *
     * @return PID controller output
     */
    T calculate(T setpoint, T processValue)
    {
        T output;

        if (true == m_resync)
        {
            m_integral         = m_lastOutput;
            m_lastError        = setpoint - processValue;
            m_resync           = false;
            m_lastProcessValue = processValue;
        }

        {
            T error        = setpoint - processValue;
            T proportional = (m_kPNumerator * error) / m_kPDenominator;
            T integral     = (m_kINumeratorDT * (m_integral + error)) / m_kIDenominatorDT;
            T derivative   = 0;

            /* Avoid integral windup. */
            integral = constrain(integral, m_min, m_max);

            if (false == m_isDerivativeOnMeasurement)
            {
                derivative = ((error - m_lastError) * m_kDNumeratorDT) / m_kDDenominatorDT;
            }
            else
            {
                derivative = ((m_lastProcessValue - processValue) * m_kDNumeratorDT) / m_kDDenominatorDT;
            }

            output = proportional + integral + derivative;

            /* Limit the controller output */
            output = constrain(output, m_min, m_max);

            m_integral         = integral;
            m_lastError        = error;
            m_lastOutput       = output;
            m_lastProcessValue = processValue;
        }

        return output;
    }

    /**
     * Get proportional factor.
     *
     * @param[out] numerator    Numerator
     * @param[out] denominator  Denominator
     */
    void getPFactor(T& numerator, T& denominator) const
    {
        numerator   = m_kPNumerator;
        denominator = m_kPDenominator;
    }

    /**
     * Set proportional factor.
     *
     * @param[in] numerator     Numerator
     * @param[in] denominator   Denominator (> 0)
     */
    void setPFactor(T numerator, T denominator)
    {
        if (0 < denominator)
        {
            m_kPNumerator   = numerator;
            m_kPDenominator = denominator;

            reduceFraction(m_kPNumerator, m_kPDenominator);
        }
    }

    /**
     * Get integral factor.
     *
     * @param[out] numerator    Numerator
     * @param[out] denominator  Denominator
     */
    void getIFactor(T& numerator, T& denominator) const
    {
        numerator   = m_kINumerator;
        denominator = m_kIDenominator;
    }

    /**
     * Set integral factor.
     * Note, the sample time will be considered internally.
     *
     * @param[in] numerator     Numerator
     * @param[in] denominator   Denominator (> 0)
     */
    void setIFactor(T numerator, T denominator)
    {
        if (0 < denominator)
        {
            m_kINumerator   = numerator;
            m_kIDenominator = denominator;

            reduceFraction(m_kINumerator, m_kIDenominator);

            if (0 == m_sampleTime)
            {
                m_kINumeratorDT   = m_kINumerator;
                m_kIDenominatorDT = m_kIDenominator;
            }
            else
            {
                m_kINumeratorDT   = m_kINumerator;
                m_kIDenominatorDT = m_kIDenominator * m_sampleTime;

                reduceFraction(m_kINumeratorDT, m_kIDenominatorDT);
            }
        }
    }

    /**
     * Get derivative factor.
     *
     * @param[out] numerator    Numerator
     * @param[out] denominator  Denominator
     */
    void getDFactor(T& numerator, T& denominator) const
    {
        numerator   = m_kDNumerator;
        denominator = m_kDDenominator;
    }

    /**
     * Set derivative factor.
     * Note, the sample time will be considered internally.
     *
     * @param[in] numerator     Numerator
     * @param[in] denominator   Denominator (> 0)
     */
    void setDFactor(T numerator, T denominator)
    {
        if (0 < denominator)
        {
            m_kDNumerator   = numerator;
            m_kDDenominator = denominator;

            reduceFraction(m_kDNumerator, m_kDDenominator);

            if (0 == m_sampleTime)
            {
                m_kDNumeratorDT   = m_kDNumerator;
                m_kDDenominatorDT = m_kDDenominator;
            }
            else
            {
                m_kDNumeratorDT   = m_kDNumerator;
                m_kDDenominatorDT = m_kDDenominator * m_sampleTime;

                reduceFraction(m_kDNumeratorDT, m_kDDenominatorDT);
            }
        }
    }

    /**
     * Set output limits.
     * Note, a 0 means it shall not be limited.
     *
     * @param[in] min   Min. value
     * @param[in] max   Max. value
     */
    void setLimits(T min, T max)
    {
        m_min = min;
        m_max = max;
    }

    /**
     * Clear last error and integral value.
     */
    void clear()
    {
        m_lastError = 0;
        m_integral  = 0;
    }

    /**
     * Get sample time (dT).
     *
     * @return Sample time in ms
     */
    uint32_t getSampleTime() const
    {
        return m_sampleTime;
    }

    /**
     * Set sample time (dT).
     * Note, internally the integral and derivative factors will be automatically
     * adjusted.
     * 
     * Ensure that the calculate() method is called once in this period.
     *
     * @param[in]   sampleTime  Sample time in ms
     */
    void setSampleTime(uint32_t sampleTime)
    {
        if (m_sampleTime != sampleTime)
        {
            if (0 == sampleTime)
            {
                m_kINumeratorDT   = m_kINumerator;
                m_kIDenominatorDT = m_kIDenominator;
                m_kDNumeratorDT   = m_kDNumerator;
                m_kDDenominatorDT = m_kDDenominator;
            }
            else
            {
                m_kINumeratorDT   = m_kINumerator;
                m_kIDenominatorDT = m_kIDenominator * sampleTime;
                m_kDNumeratorDT   = m_kDNumerator;
                m_kDDenominatorDT = m_kDDenominator * sampleTime;

                reduceFraction(m_kINumeratorDT, m_kIDenominatorDT);
                reduceFraction(m_kDNumeratorDT, m_kDDenominatorDT);
            }

            m_sampleTime = sampleTime;
        }
    }

    /**
     * If the PID controller was not processed for more than one dT and now
     * it will be processed again, it may happen that the output bumps, caused
     * by the integral and derivative part. This happens especically if the
     * setpoint changes as well. To avoid the output bump, call this method once.
     */
    void resync()
    {
        m_resync = true;
    }

    /**
     * To avoid the derivative kick, enable derivative on measurement.
     * The derivative term is based on the negative process value changes, instead
     * of the error changes.
     *
     * @param[in] enable    Enables (true) or disables (false) derivative on measurement.
     */
    void setDerivativeOnMeasurement(bool enable)
    {
        m_isDerivativeOnMeasurement = enable;
    }

    /**
     * Default sample time in ms.
     * Keep value lower than 128 to avoid conflict in case T is int8_t.
     */
    static const uint32_t SAMPLE_TIME_DEFAULT = 10;

protected:
private:
    T m_kPNumerator;   /**< Numerator of proportional factor */
    T m_kPDenominator; /**< Denominator of proportional factor */
    T m_kINumerator;   /**< Numerator of integral factor */
    T m_kIDenominator; /**< Denominator of integral factor */
    T m_kDNumerator;   /**< Numerator of derivative factor */
    T m_kDDenominator; /**< Denominator of derivative factor */

    T m_kINumeratorDT;   /**< Numerator of integral factor with considered sample time */
    T m_kIDenominatorDT; /**< Denominator of integral factor with considered sample time */
    T m_kDNumeratorDT;   /**< Numerator of derivative factor with considered sample time */
    T m_kDDenominatorDT; /**< Denominator of derivative factor with considered sample time */

    T        m_min;                       /**< Min. output value, used for limiting. */
    T        m_max;                       /**< Max. output value, used for limiting. */
    T        m_lastError;                 /**< Last calculated error */
    T        m_integral;                  /**< Integral value */
    T        m_lastOutput;                /**< Last output value, which is used till next sample time. */
    uint32_t m_sampleTime;                /**< Sample time period in ms */
    bool     m_resync;                    /**< A resync avoids a output bump */
    bool     m_isDerivativeOnMeasurement; /**< Enables/Disables derivative on measurement. */
    T        m_lastProcessValue;          /**< Last process value is used for derivative on measurement only. */

    /**
     * Check all demoniator values for zero.
     * If one is zero, it will be set to 1.
     */
    void denominatorsShallNotBeZero()
    {
        if (0 == m_kPDenominator)
        {
            m_kPDenominator = 1;
        }

        if (0 == m_kIDenominator)
        {
            m_kIDenominator = 1;
        }

        if (0 == m_kDDenominator)
        {
            m_kDDenominator = 1;
        }

        if (0 == m_kIDenominatorDT)
        {
            m_kIDenominatorDT = m_sampleTime;
        }

        if (0 == m_kDDenominatorDT)
        {
            m_kDDenominatorDT = m_sampleTime;
        }
    }

    /**
     * Calculate greatest common divisor.
     *
     * @param[in] number1   Number 1
     * @param[in] number2   Number 2
     *
     * @return Greatest common divisor
     */
    T calcGreatestCommonDivisor(T number1, T number2) const
    {
        if (0 > number1)
        {
            number1 = -number1;
        }

        if (0 > number2)
        {
            number2 = -number2;
        }

        while (0 != number2)
        {
            T rest  = number1 % number2;
            number1 = number2;
            number2 = rest;
        }

        return number1;
    }

    /**
     * Reduce fraction.
     *
     * @param[in,out] number1   Number 1
     * @param[in,out] number2   Number 2
     */
    void reduceFraction(T& number1, T& number2)
    {
        T divisor = calcGreatestCommonDivisor(number1, number2);

        if (0 == divisor)
        {
            divisor = 1;
        }

        number1 /= divisor;
        number2 /= divisor;
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PIDCONTROLLER_H */