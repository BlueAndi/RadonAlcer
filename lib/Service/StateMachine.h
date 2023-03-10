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
 * @brief  Statemachine
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup Application
 *
 * @{
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IState.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The system state machine. */
class StateMachine
{
public:
    /**
     * Default constructor.
     */
    StateMachine() : m_currentState(nullptr), m_nextState(nullptr)
    {
    }

    /**
     * Default destructor.
     */
    ~StateMachine()
    {
    }

    /**
     * Set next state.
     *
     * @param[in] state Next state.
     */
    void setState(IState* state)
    {
        m_nextState = state;
    }

    /**
     * Get current state.
     *
     * @return Current state.
     */
    IState* getState()
    {
        return m_currentState;
    }

    /**
     * Process state machine.
     */
    void process();

protected:
private:
    IState* m_currentState; /**< Current active state */
    IState* m_nextState;    /**< Next state */

    StateMachine(const StateMachine& sm);
    StateMachine& operator=(const StateMachine& sm);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* STATE_MACHINE_H */