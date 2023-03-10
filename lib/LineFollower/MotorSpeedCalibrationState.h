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
 * @brief  Motor speed calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef MOTOR_SPEED_CALIBRATION_STATE_H
#define MOTOR_SPEED_CALIBRATION_STATE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <IState.h>
#include <SimpleTimer.h>
#include <RelativeEncoder.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The motor speed calibration state. */
class MotorSpeedCalibrationState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static MotorSpeedCalibrationState& getInstance()
    {
        static MotorSpeedCalibrationState instance;

        /* Singleton idiom to force initialization during first usage. */

        return instance;
    }

    /**
     * If the state is entered, this method will called once.
     */
    void entry() final;

    /**
     * Processing the state.
     *
     * @param[in] sm State machine, which is calling this state.
     */
    void process(StateMachine& sm) final;

    /**
     * If the state is left, this method will be called once.
     */
    void exit() final;

protected:
private:
    /** Calibration phases */
    enum Phase
    {
        PHASE_1_WAIT = 0, /**< Wait that the operator removed its fingers from the robot. */
        PHASE_2_BACK,     /**< Drive with max. speed backwards. */
        PHASE_3_FORWARD,  /**< Drive with max. speed forwards. */
        PHASE_4_FINISHED  /**< Calibration is finished. */
    };

    /**
     * Duration in ms about to wait, until the calibration drive starts.
     */
    static const uint32_t WAIT_TIME = 1000;

    /**
     * Calibration drive duration in ms.
     * It means how long the robot is driven with max. speed forward/backward.
     */
    static const uint32_t CALIB_DURATION = 1000;

    SimpleTimer     m_timer; /**< Timer used to wait, until the calibration drive starts and for drive duration. */
    Phase           m_phase; /**< Current calibration phase */
    int16_t         m_maxSpeedLeft;  /**< Max. determined left motor speed [steps/s]. */
    int16_t         m_maxSpeedRight; /**< Max. determined right motor speed [steps/s]. */
    RelativeEncoder m_relEncLeft;    /**< Relative encoder left */
    RelativeEncoder m_relEncRight;   /**< Relative encoder right */

    /**
     * Default constructor.
     */
    MotorSpeedCalibrationState() : m_timer(), m_phase(PHASE_1_WAIT), m_maxSpeedLeft(0), m_maxSpeedRight(0), m_relEncLeft(), m_relEncRight()
    {
    }

    /**
     * Default destructor.
     */
    ~MotorSpeedCalibrationState()
    {
    }

    MotorSpeedCalibrationState(const MotorSpeedCalibrationState& state);
    MotorSpeedCalibrationState& operator=(const MotorSpeedCalibrationState& state);

    /**
     * Phase 1: Wait a short time to avoid that the operator still touches the robot.
     */
    void phase1Wait();

    /**
     * Phase 2: Determine max. backward motor speed.
     */
    void phase2Back();

    /**
     * Phase 3: Determine max. forward motor speed.
     */
    void phase3Forward();

    /**
     * Phase 4: Calibration finished, continue to next state.
     *
     * @param[in] sm    State machine
     */
    void phase4Finished(StateMachine& sm);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* MOTOR_SPEED_CALIBRATION_STATE_H */