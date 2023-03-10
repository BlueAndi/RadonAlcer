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
 * @brief  Calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ReleaseTrackState.h"
#include <Board.h>
#include <StateMachine.h>
#include "DrivingState.h"
#include "ParameterSets.h"

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

void ReleaseTrackState::entry()
{
    /* Start challenge after specific time. */
    m_releaseTimer.start(TRACK_RELEASE_DURATION);

    /* Choose parameter set 0 by default. */
    ParameterSets::getInstance().choose(0);
    showParSet();
}

void ReleaseTrackState::process(StateMachine& sm)
{
    IButton& buttonA = Board::getInstance().getButtonA();

    /* Change parameter set? */
    if (true == buttonA.isPressed())
    {
        /* Choose next parameter set (round-robin) */
        ParameterSets::getInstance().next();
        showParSet();

        buttonA.waitForRelease();
        m_releaseTimer.restart();
    }

    /* Release track after specific time. */
    if (true == m_releaseTimer.isTimeout())
    {
        sm.setState(&DrivingState::getInstance());
    }
}

void ReleaseTrackState::exit()
{
    m_releaseTimer.stop();
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void ReleaseTrackState::showParSet() const
{
    IDisplay&   display    = Board::getInstance().getDisplay();
    uint8_t     parSetId   = ParameterSets::getInstance().getCurrentSetId();
    const char* parSetName = ParameterSets::getInstance().getParameterSet().name;

    display.clear();
    display.print("Set ");
    display.print(parSetId);
    display.gotoXY(0, 1);
    display.print(parSetName);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
