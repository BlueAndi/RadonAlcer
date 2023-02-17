/* MIT License

Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  Definition of Zumo Communication Protocol
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "ComProtocol.h"

/******************************************************************************
 * Macros
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

ComProtocol& ComProtocol::getInstance()
{
    static ComProtocol instance; /* idiom */

    return instance;
}

void ComProtocol::init()
{
}

bool ComProtocol::appendData(uint8_t data)
{
    const uint8_t numberOfBytes      = 1U;
    const uint8_t buf[numberOfBytes] = {data};
    return appendData(buf, numberOfBytes);
}

bool ComProtocol::appendData(uint16_t data)
{
    const uint8_t numberOfBytes = 2U;

    uint8_t MSB = ((data & 0xFF00) >> 8U);
    uint8_t LSB = (data & 0x00FF);

    // Big Endian
    const uint8_t buf[numberOfBytes] = {MSB, LSB};

    return appendData(buf, numberOfBytes);
}

bool ComProtocol::appendData(uint32_t data)
{
    const uint8_t numberOfBytes = 4U;

    uint16_t hiBytes  = ((data & 0xFFFF0000) >> 16U);
    uint16_t lowBytes = (data & 0x0000FFFF);

    uint8_t hiMSB  = ((hiBytes & 0xFF00) >> 8U);
    uint8_t hiLSB  = (hiBytes & 0x00FF);
    uint8_t lowMSB = ((lowBytes & 0xFF00) >> 8U);
    uint8_t lowLSB = (lowBytes & 0x00FF);

    // Big Endian
    const uint8_t buf[numberOfBytes] = {hiMSB, hiLSB, lowMSB, lowLSB};

    return appendData(buf, numberOfBytes);
}

bool ComProtocol::appendData(const uint8_t* data, uint8_t length)
{
    bool isSuccess = false;

    // Check for enough space in frame
    if ((m_frame.fields.m_dlc + length) <= MAX_DATA_LEN)
    {
        for (uint8_t i = 0; i < length; i++)
        {
            m_frame.fields.m_data[m_frame.fields.m_dlc] = data[i];
            m_frame.fields.m_dlc++;
            m_frame.fields.m_checksum = ((m_frame.fields.m_checksum + data[i] + 1) % UINT8_MAX);
        }
        isSuccess = true;
    }

    return isSuccess;
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

ComProtocol::ComProtocol()
{
}

ComProtocol::~ComProtocol()
{
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
