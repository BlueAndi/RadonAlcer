/* MIT License
 *
 * Copyright (c) 2019 - 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Print implementation for test
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup test
 *
 * @{
 */

#ifndef __PRINT_H__
#define __PRINT_H__

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/** Decimal number base */
#define DEC (10)

/** Hexadecimal number base */
#define HEX (16)

/** Octal number base */
#define OCT (8)

/** Binary number base */
#define BIN (2)

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Print class for test purposes only.
 * Note, the source implementation is from the orignal Arduino source.
 */
class Print
{
public:

    /**
     * Constructs a print object.
     */
    Print()
    {
    }

    /**
     * Destroys a print object.
     */
    virtual ~Print()
    {
    }

    virtual void print(const char str[]) = 0;   

private:

};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif  /* __PRINT_H__ */

/** @} */