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

#ifndef __LOGGING_H__
#define __LOGGING_H__

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

#ifndef LOG_FATAL_ENABLE
#define LOG_FATAL_ENABLE (1)
#endif /* LOG_FATAL_ENABLE */

#ifndef LOG_ERROR_ENABLE
#define LOG_ERROR_ENABLE (1)
#endif /* LOG_ERROR_ENABLE */

#ifndef LOG_WARNING_ENABLE
#define LOG_WARNING_ENABLE (1)
#endif /* LOG_WARNING_ENABLE */

#ifndef LOG_INFO_ENABLE
#define LOG_INFO_ENABLE (1)
#endif /* LOG_INFO_ENABLE */

#ifndef LOG_DEBUG_ENABLE
#define LOG_DEBUG_ENABLE (0)
#endif /* LOG_DEBUG_ENABLE */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

#if (0 == LOG_FATAL_ENABLE)

#define LOG_FATAL(...)

#else /* (0 == LOG_FATAL_ENABLE) */

/** Log fatal error message. */
#define LOG_FATAL(filename, ...) Logging::print(filename, __LINE__, Logging::LOG_LEVEL_ERROR, __VA_ARGS__)

#endif /* (0 == LOG_FATAL_ENABLE) */

#if (0 == LOG_ERROR_ENABLE)

#define LOG_ERROR(filename, ...)

#else /* (0 == LOG_ERROR_ENABLE) */

/** Log error message. */
#define LOG_ERROR(filename, ...) Logging::print(filename, __LINE__, Logging::LOG_LEVEL_ERROR, __VA_ARGS__)

#endif /* (0 == LOG_ERROR_ENABLE) */

#if (0 == LOG_WARNING_ENABLE)

#define LOG_WARNING(filename, ...)

#else /* (0 == LOG_WARNING_ENABLE) */

/** Log warning message. */
#define LOG_WARNING(filename, ...) Logging::print(filename, __LINE__, Logging::LOG_LEVEL_WARNING, __VA_ARGS__)

#endif /* (0 == LOG_WARNING_ENABLE) */

#if (0 == LOG_INFO_ENABLE)

#define LOG_INFO(filename, ...)

#else /* (0 == LOG_INFO_ENABLE) */

/** Log info error message. */
#define LOG_INFO(filename, ...) Logging::print(filename, __LINE__, Logging::LOG_LEVEL_INFO, __VA_ARGS__)

#endif /* (0 == LOG_INFO_ENABLE) */

#if (0 == LOG_DEBUG_ENABLE)

#define LOG_DEBUG(filename, ...)

#else /* (0 == LOG_DEBUG_ENABLE) */

/** Log debug message. */
#define LOG_DEBUG(filename, ...) Logging::print(filename, __LINE__, Logging::LOG_LEVEL_DEBUG, __VA_ARGS__)

#endif /* (0 == LOG_DEBUG_ENABLE) */

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Logging class for log messages depending on the previously set log level.
 */
namespace Logging
{
    /**
     * Enumeration to distinguish between different levels of severity.
     */
    enum LogLevel
    {
        LOG_LEVEL_FATAL = 0, /**< Any error that is forcing a shutdown of service or application, because there is no way out. */
        LOG_LEVEL_ERROR,     /**< Any error that is fatal for the operating, but not for the service or application. */
        LOG_LEVEL_WARNING,   /**< Anything that shows the user to pay attention, but can be automatically be recovered. */
        LOG_LEVEL_INFO,      /**< General useful information for the user. */
        LOG_LEVEL_DEBUG,     /**< A diagnostic message helpful for the developer. */
    };

    /** The maximum size of the logMessage buffer to get the variable arguments. */
    const uint16_t MESSAGE_BUFFER_SIZE   = 80U;

    /**
     * Print log message.
     *
     * @param[in] filename      The name of the file, where the log message is located.
     * @param[in] lineNumber    The line number in the file, where the log message is located.
     * @param[in] level         The severity level.
     * @param[in] format        The format with the variable argument list.
     */
    void print(const char* filename, int lineNumber, LogLevel level, const char* format, ...);

}; // namespace Logging

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* __LOGGING_H__ */