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
#include "ReadyState.h"
#include <Board.h>
#include <StateMachine.h>
#include "ReleaseTrackState.h"
#include <Logging.h>

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

void ReadyState::entry()
{
    IDisplay&     display                 = Board::getInstance().getDisplay();
    const int32_t SENSOR_VALUE_OUT_PERIOD = 1000; /* ms */

    display.clear();
    display.print("Rdy.");

    if (true == m_isLapTimeAvailable)
    {
        display.gotoXY(0, 1);
        display.print(m_lapTime);
    }

    /* The line sensor value shall be output on console cyclic. */
    m_timer.start(SENSOR_VALUE_OUT_PERIOD);
}

void ReadyState::process(StateMachine& sm)
{
    IButton& buttonA = Board::getInstance().getButtonA();

    /* Shall track be released? */
    if (true == buttonA.isPressed())
    {
        buttonA.waitForRelease();
        sm.setState(&ReleaseTrackState::getInstance());
    }
    /* Shall the line sensor values be printed out on console? */
    else if (true == m_timer.isTimeout())
    {
        ILineSensors&   lineSensors  = Board::getInstance().getLineSensors();
        uint8_t         index        = 0;
        int16_t         position     = lineSensors.readLine();
        const uint16_t* sensorValues = lineSensors.getSensorValues();

        char msg[Logging::MESSAGE_BUFFER_SIZE] = "";
        char tmp[10U] = {0};

        /* Print line sensor value on console for debug purposes. */
        for (index = 0; index < lineSensors.getNumLineSensors(); ++index)
        {
            
            if (0 < index)
            {
                strcat(msg, " / ");
            }
            
            snprintf(tmp, 10U, "%u", sensorValues[index]);
            strcat(msg, tmp);
        }
        strcat(msg, " -> ");
        snprintf(tmp, 10U, "%u", position);
        strcat(msg, tmp);

        LOG_DEBUG("ReadyState", msg);

        m_timer.restart();
    }
}

void ReadyState::exit()
{
    m_timer.stop();
    m_isLapTimeAvailable = false;
}

void ReadyState::setLapTime(uint32_t lapTime)
{
    m_isLapTimeAvailable = true;
    m_lapTime            = lapTime;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
