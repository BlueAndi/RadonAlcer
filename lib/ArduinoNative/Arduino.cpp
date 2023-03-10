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
 * @brief  Arduino native
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>

#ifdef UNIT_TEST

#include <time.h>

#else

#include <Board.h>
#include <webots/Robot.hpp>
#include <Keyboard.h>

#endif

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

extern void setup();
extern void loop();

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial driver, used by Arduino applications. */
Serial_ Serial;

#ifndef UNIT_TEST

/**
 * The maximum duration a simulated time step can have.
 * Everything above would cause missbehaviour in the application.
 */
static const int MAX_TIME_STEP = 10;

/**
 * Simulation time handler, used by Arduino functions.
 */
static SimTime* gSimTime = nullptr;

#endif

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

#ifdef UNIT_TEST

extern int main(int argc, char** argv)
{
    setup(); /* Prepare test */
    loop();  /* Run test once */

    return 0;
}

extern unsigned long millis()
{
    clock_t now = clock();

    return (now * 1000UL) / CLOCKS_PER_SEC;
}

extern void delay(unsigned long ms)
{
    unsigned long timestamp = millis();

    while ((millis() - timestamp) < ms)
    {
        ;
    }
}

#else

extern int main(int argc, char** argv)
{
    int       status   = 0;
    Keyboard& keyboard = Board::getInstance().getKeyboard();
    
    /* Get simulation time handler. It will be used by millis() and delay(). */
    gSimTime = &Board::getInstance().getSimTime();

    if ((0 == gSimTime->getTimeStep()) || (MAX_TIME_STEP < gSimTime->getTimeStep()))
    {
        printf("Simulation time step is too high!\n");
        printf("This would cause missbehaviour in the application.\n");

        status = -1;
    }
    else
    {
        /**
         * Synchronization between the simulation steps and the control steps is done automatically
         * by Webots (If the synchronization field in the robot node is set to TRUE).
         * For a more detailed explanation see:
         * https://cyberbotics.com/doc/reference/robot#synchronous-versus-asynchronous-controllers
         */

        setup();

        while (true == gSimTime->step())
        {
            keyboard.getPressedButtons();
            loop();
        }

        status = 0;
    }

    return status;
}

extern unsigned long millis()
{
    return gSimTime->getElapsedTimeSinceReset();
}

extern void delay(unsigned long ms)
{
    unsigned long timestamp = millis();

    while ((millis() - timestamp) < ms)
    {
        if (false == gSimTime->step())
        {
            break;
        }
    }
}

#endif

/******************************************************************************
 * Local Functions
 *****************************************************************************/
