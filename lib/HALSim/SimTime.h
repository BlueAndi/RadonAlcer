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
 * @brief  Simulation time
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef SIM_TIME_H
#define SIM_TIME_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <webots/Robot.hpp>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Simulation time handler
 * Its responsibility is to provide elapsed time information since reset and
 * to step the simulation time forward.
 */
class SimTime
{
public:
    /**
     * Construct simulation time handler.
     *
     * @param[in] robot The simulation environment.
     */
    SimTime(webots::Robot& robot) :
        m_robot(robot),
        m_timeStep(static_cast<int>(m_robot.getBasicTimeStep())),
        m_elapsedTimeSinceReset(0)
    {
    }

    /**
     * Destroy the simulation time handler.
     */
    ~SimTime()
    {
    }

    /**
     * Step the simulation one single step forward.
     * How long one step is depends on the basic basic time step configured in
     * the webots simulation. See there in the world information.
     *
     * @return If successful stepped, it will return true otherwise false.
     */
    bool step()
    {
        int result = m_robot.step(m_timeStep);
        m_elapsedTimeSinceReset += m_timeStep;

        return (-1 != result);
    }

    /**
     * Get basic time step in [ms].
     *
     * @return Basic time step [ms]
     */
    int getTimeStep() const
    {
        return m_timeStep;
    }

    /**
     * Get the elapsed time since reset in [ms].
     *
     * @return Elapsed time since reset [ms]
     */
    unsigned long int getElapsedTimeSinceReset() const
    {
        return m_elapsedTimeSinceReset;
    }

private:
    webots::Robot&    m_robot;                 /**< Simulation environment, used to step the simulation forward. */
    int               m_timeStep;              /**< Time in ms of one simulation step. */
    unsigned long int m_elapsedTimeSinceReset; /**< Elapsed time since reset in [ms] */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* SIM_TIME_H */