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
 * @brief  Logging
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Logging.h"
#include <stdarg.h>
#include <stdint.h>

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

static void printHead(const char* filename, int lineNumber, Logging::LogLevel level);

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

/******************************************************************************
 * External Functions
 *****************************************************************************/

void Logging::print(const char* filename, int lineNumber, Logging::LogLevel level, const char* format, ...)
{
    const size_t MESSAGE_BUFFER_SIZE = 512;
    char         buffer[MESSAGE_BUFFER_SIZE];
    va_list      args;

    printHead(filename, lineNumber, level);

    va_start(args, format);
    vsnprintf(buffer, MESSAGE_BUFFER_SIZE, format, args);
    va_end(args);

    Serial.println(buffer);
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static void printHead(const char* filename, int lineNumber, Logging::LogLevel level)
{
    uint8_t     HEAD_BUFFER_SIZE         = 80U;
    char        buffer[HEAD_BUFFER_SIZE] = {0};
    uint16_t    written                  = 0;
    const char* levelStr                 = "U";

    switch (level)
    {
    case Logging::LOG_LEVEL_FATAL:
        levelStr = "F";
        break;

    case Logging::LOG_LEVEL_ERROR:
        levelStr = "E";
        break;
    
    case Logging::LOG_LEVEL_WARNING:
        levelStr = "W";
        break;

    case Logging::LOG_LEVEL_INFO:
        levelStr = "I";
        break;

    case Logging::LOG_LEVEL_DEBUG:
        levelStr = "D";
        break;

    default:
        break;
    }

    written = snprintf(buffer,
                       HEAD_BUFFER_SIZE,
                       "%6lu [%1s] %10s:%04u |   ",
                       millis(),
                       levelStr,
                       filename,
                       lineNumber);
    
    Serial.print(buffer);
}