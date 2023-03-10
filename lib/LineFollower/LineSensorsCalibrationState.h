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
 * @brief  Line sensors calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef LINE_SENSORS_CALIBRATION_STATE_H
#define LINE_SENSORS_CALIBRATION_STATE_H

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

/** The line sensors calibration state. */
class LineSensorsCalibrationState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static LineSensorsCalibrationState& getInstance()
    {
        static LineSensorsCalibrationState instance;

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
        PHASE_1_WAIT = 0,   /**< Wait a short time before starting the calibration. */
        PHASE_2_TURN_LEFT,  /**< Turn line sensors left over the line. */
        PHASE_3_TURN_RIGHT, /**< Turn line sensor right over the line. */
        PHASE_4_TURN_ORIG,  /**< Turn back to origin. */
        PHASE_5_FINISHED    /**< Calibration is finished. */
    };

    /**
     * Duration in ms about to wait, until the calibration drive starts.
     */
    static const uint32_t WAIT_TIME = 1000;

    /**
     * Calibration turn angle in mrad (corresponds to 60°).
     */
    static const int16_t CALIB_ANGLE = static_cast<int16_t>((2000.0f * PI) / 6.0f);

    SimpleTimer m_timer;            /**< Timer used to wait, until the calibration drive starts. */
    Phase       m_phase;            /**< Current calibration phase */
    int16_t     m_calibrationSpeed; /**< Speed in steps/s for the calibration drive. */

    /**
     * Default constructor.
     */
    LineSensorsCalibrationState() : m_timer(), m_phase(PHASE_1_WAIT), m_calibrationSpeed(0)
    {
    }

    /**
     * Default destructor.
     */
    ~LineSensorsCalibrationState()
    {
    }

    LineSensorsCalibrationState(const LineSensorsCalibrationState& state);
    LineSensorsCalibrationState& operator=(const LineSensorsCalibrationState& state);

    /**
     * Phase 1: Wait a short time to avoid that the operator still touches the robot.
     */
    void phase1Wait();

    /**
     * Phase 2: Turn line sensors left over the line.
     */
    void phase2TurnLeft();

    /**
     * Phase 3: Turn line sensors right over the line.
     */
    void phase3TurnRight();

    /**
     * Phase 4: Turn back to origin.
     */
    void phase4TurnOrigin();

    /**
     * Phase 5: Calibration finished, continue to next state.
     *
     * @param[in] sm    State machine
     */
    void phase5Finished(StateMachine& sm);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINE_SENSORS_CALIBRATION_STATE_H */