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
 * @brief  Keyboard realization
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef KEYBOARD_H
#define KEYBOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>

#include "SimTime.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** This class implements a custom keyboard for better functionality. */
class Keyboard
{
public:
    /**
     * Constructs the encoders adapter.
     */
    Keyboard(SimTime& simTime, webots::Keyboard* keyboard) :
        m_oldKeys(),
        m_newKeys(),
        m_simTime(simTime),
        m_keyboard(keyboard)
    {
        if (nullptr != keyboard)
        {
            keyboard->enable(simTime.getTimeStep());
        }
    }

    /**
     * Destroys the encoders adapter.
     */
    ~Keyboard()
    {
    }

    /**
     * Gets the current buttons pressed on the keyboard. Needs to be called
     * cyclically to work correct.
     */
    void getPressedButtons()
    {
        /* Copying the new values into the old values array. */
        for (uint8_t arrayIndex = 0; arrayIndex < (sizeof(m_oldKeys) / sizeof(*m_oldKeys)); ++arrayIndex)
        {
            m_oldKeys[arrayIndex] = m_newKeys[arrayIndex];
        }

        /* Getting the new values. Currently the limit of the simulation is seven
         * keypresses that can be detected simultaniously.
         */
        for (uint8_t keyIndex = 0; keyIndex < (sizeof(m_oldKeys) / sizeof(*m_oldKeys)); ++keyIndex)
        {
            m_newKeys[keyIndex] = m_keyboard->getKey();
        }
    }

    /**
     * Checks weather the button A was pressed since the last update.
     *
     * @return Return true if button A was pressed since the last update
     */
    bool buttonAPressed()
    {
        return isButtonPressed(KEY_CODE_A_LOWER_CASE, KEY_CODE_A_UPPER_CASE);
    }

    /**
     * Checks weather the button A was released since the last update.
     *
     * @return Return true if button A was released since the last update
     */
    bool buttonAReleased()
    {
        return isButtonReleased(KEY_CODE_A_LOWER_CASE, KEY_CODE_A_UPPER_CASE);
    }

    /**
     * Checks weather the button B was pressed since the last update.
     *
     * @return Return true if button B was pressed since the last update
     */
    bool buttonBPressed()
    {
        return isButtonPressed(KEY_CODE_B_LOWER_CASE, KEY_CODE_B_UPPER_CASE);
    }

    /**
     * Checks weather the button B was released since the last update.
     *
     * @return Return true if button B was released since the last update
     */
    bool buttonBReleased()
    {
        return isButtonReleased(KEY_CODE_B_LOWER_CASE, KEY_CODE_B_UPPER_CASE);
    }

    /**
     * Checks weather the button C was pressed since the last update.
     *
     * @return Return true if button C was pressed since the last update
     */
    bool buttonCPressed()
    {
        return isButtonPressed(KEY_CODE_C_LOWER_CASE, KEY_CODE_C_UPPER_CASE);
    }

    /**
     * Checks weather the button C was released since the last update.
     *
     * @return Return true if button C was released since the last update
     */
    bool buttonCReleased()
    {
        return isButtonReleased(KEY_CODE_C_LOWER_CASE, KEY_CODE_C_UPPER_CASE);
    }

    /**
     * Waits until Button A gets released.
     * Needs to call the robots step() method and getPressedButtons()
     * to update the keypresses correctly and avoid getting stuck
     * in the while loop.
     */
    void waitForReleaseA()
    {
        while (!buttonAReleased())
        {
            if (false == m_simTime.step())
            {
                break;
            }
            getPressedButtons();
        }
    }

    /**
     * Waits until Button B gets released.
     * Needs to call the robots step() method and getPressedButtons()
     * to update the keypresses correctly and avoid getting stuck
     * in the while loop.
     */
    void waitForReleaseB()
    {
        while (!buttonBReleased())
        {
            if (false == m_simTime.step())
            {
                break;
            }
            getPressedButtons();
        }
    }

    /**
     * Waits until Button C gets released.
     * Needs to call the robots step() method and getPressedButtons()
     * to update the keypresses correctly and avoid getting stuck
     * in the while loop.
     */
    void waitForReleaseC()
    {
        while (!buttonCReleased())
        {
            if (false == m_simTime.step())
            {
                break;
            }
            getPressedButtons();
        }
    }

private:
    /** The key code of the lower case a character, which simulates the button. */
    static const char KEY_CODE_A_LOWER_CASE = 'a';

    /** The key code of the upper case A character, which simulates the button. */
    static const char KEY_CODE_A_UPPER_CASE = 'A';

    /** The key code of the lower case b character, which simulates the button. */
    static const char KEY_CODE_B_LOWER_CASE = 'b';

    /** The key code of the upper case B character, which simulates the button. */
    static const char KEY_CODE_B_UPPER_CASE = 'B';

    /** The key code of the lower case c character, which simulates the button. */
    static const char KEY_CODE_C_LOWER_CASE = 'c';

    /** The key code of the upper case C character, which simulates the button. */
    static const char KEY_CODE_C_UPPER_CASE = 'C';

    /** The maximum number of keys pressed simultaniously, that the simulation can process. */
    static const uint8_t MAX_KEY_NUMBER = 7;

    /** The keys presses during the last update. */
    uint16_t m_oldKeys[MAX_KEY_NUMBER];

    /** The keys pressed during this update. */
    uint16_t m_newKeys[MAX_KEY_NUMBER];

    webots::Keyboard* m_keyboard; /**< Robot keyboard */

    SimTime& m_simTime; /**< Simulation time */

    /**
     * Is the button pressed?
     *
     * @param[in] lowerCaseChar Lower case character ASCII value
     * @param[in] upperCaseChar Upper case character ASCII value
     *
     * @return If pressed, it will return true otherwise false.
     */
    bool isButtonPressed(char lowerCaseChar, char upperCaseChar) const;

    /**
     * Is the button released?
     *
     * @param[in] lowerCaseChar Lower case character ASCII value
     * @param[in] upperCaseChar Upper case character ASCII value
     *
     * @return If released, it will return true otherwise false.
     */
    bool isButtonReleased(char lowerCaseChar, char upperCaseChar) const;

    /**
     * Checks whether the given array contains a element.
     *
     * @param[in]   array           The array where to search for the element.
     * @param[in]   arraySize       Number of array elements.
     * @param[in]   elemLowerCase   Lower case character
     * @param[in]   elemUppercase   Upper case character
     *
     * @return If found, it will return true otherwise false.
     */
    bool arrayContains(const uint16_t array[], uint16_t arraySize, char elemLowerCase, char elemUppercase) const;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* KEYBOARD_H */