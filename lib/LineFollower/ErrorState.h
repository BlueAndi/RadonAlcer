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
 * @brief  Error state
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup Application
 *
 * @{
 */

#ifndef ERROR_STATE_H
#define ERROR_STATE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stddef.h>
#include <IState.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The system error state. */
class ErrorState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static ErrorState& getInstance()
    {
        static ErrorState instance;

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

    /**
     * Set error message, which to show on the display.
     * 
     * @param[in] msg   Error message
     */
    void setErrorMsg(const char* msg);

protected:
private:

    /**
     * The error message string size in bytes, which
     * includes the terminating character.
     */
    static const size_t ERROR_MSG_SIZE  = 20;

    char    m_errorMsg[ERROR_MSG_SIZE]; /**< Error message, which to show. */

    /**
     * Default constructor.
     */
    ErrorState() : m_errorMsg()
    {
        m_errorMsg[0] = '\0';
    }

    /**
     * Default destructor.
     */
    ~ErrorState()
    {
    }

    ErrorState(const ErrorState& state);
    ErrorState& operator=(const ErrorState& state);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ERROR_STATE_H */