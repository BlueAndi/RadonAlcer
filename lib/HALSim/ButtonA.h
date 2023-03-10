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
 * @brief  Button A realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef BUTTONA_H
#define BUTTONA_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IButton.h"

#include <Keyboard.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class provides access to the robot simulation button A. */
class ButtonA : public IButton
{
public:
    /**
     * Constructs the button A adapter.
     *
     * @param[in] keyboard  The robot keyboard.
     */
    ButtonA(Keyboard& keyboard) : IButton(), m_keyboard(keyboard)
    {
    }

    /**
     * Destroys the button A adapter.
     */
    ~ButtonA()
    {
    }

    /**
     * Indicates whether the button A is pressed or not.
     *
     * @return Returns true if button is pressed, otherwise returns false.
     */
    bool isPressed() final;

    /**
     * Wait until button A is released.
     */
    void waitForRelease() final;

private:
    Keyboard& m_keyboard; /**< Robot keyboard */

    /* Default constructor not allowed. */
    ButtonA();
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BUTTONA_H */