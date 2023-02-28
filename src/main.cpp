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
 * @brief  Main entry point
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include "StateMachine.h"
#include "StartupState.h"
#include <Board.h>
#include <Mileage.h>
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <NTServer.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY     (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Variables
 *****************************************************************************/

/** The system state machine. */
static StateMachine gSystemStateMachine;

/** Serial log sink */
static LogSinkPrinter   gLogSinkSerial("Serial", &Serial);

/** Instance of NT Server with 10 maximum Channels */
static NTServer<10> gNTServer;

/******************************************************************************
 * External functions
 *****************************************************************************/

/**
 * Initialize the system.
 * This function is called once during startup.
 */
void setup() // cppcheck-suppress unusedFunction
{
    Board::getInstance().init();
    Serial.begin(115200);

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        Logging::getInstance().selectSink("Serial");
    }
    
    Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);

    LOG_INFO("Logger Initialized");

    gSystemStateMachine.setState(&StartupState::getInstance());
}

/**
 * Main program loop.
 * This function is called cyclic.
 */
void loop() // cppcheck-suppress unusedFunction
{
    Mileage::getInstance().process();
    gSystemStateMachine.process();
    gNTServer.process();
}

/******************************************************************************
 * Local functions
 *****************************************************************************/
