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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Keyboard.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

bool Keyboard::isButtonPressed(char lowerCaseChar, char upperCaseChar) const
{
    bool buttonPressed = false;

    /* Checks if button is existing in the new values, but not the old ones.
     * If so, it's newly pressed and true is returned.
     */
    if ((false == arrayContains(m_oldKeys, sizeof(m_oldKeys), upperCaseChar, lowerCaseChar)) && 
        (true == arrayContains(m_newKeys, sizeof(m_newKeys), upperCaseChar, lowerCaseChar)))
    {
        buttonPressed = true;
    }

    return buttonPressed;
}

bool Keyboard::isButtonReleased(char lowerCaseChar, char upperCaseChar) const
{
    bool buttonReleased = false;

    /* Checks if button is existing in the new values, but not the old ones.
     * If so, it's newly released and true is returned.
     */
    if ((false == arrayContains(m_newKeys, sizeof(m_newKeys), upperCaseChar, lowerCaseChar)) && 
        (true == arrayContains(m_oldKeys, sizeof(m_oldKeys), upperCaseChar, lowerCaseChar)))
    {
        buttonReleased = true;
    }

    return buttonReleased;
}

bool Keyboard::arrayContains(const uint16_t array[], uint16_t arraySize, char elemLowerCase, char elemUppercase) const
{
    bool elementFound = false;

    for (uint8_t arrayIndex = 0; arrayIndex < (arraySize / sizeof(*array)); ++arrayIndex)
    {
        if((array[arrayIndex] == static_cast<uint8_t>(elemLowerCase)) || (array[arrayIndex] == static_cast<uint8_t>(elemUppercase)))
        {
            elementFound = true;;
        }
    }

    return elementFound;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
