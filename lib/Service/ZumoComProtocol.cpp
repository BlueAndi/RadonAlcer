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
#include "ZumoComProtocol.h"

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

ZumoComFrame::ZumoComFrame(uint8_t channel) : m_channel(channel), m_dlc(0U), m_data{0U}, m_checksum(0U)
{
}

ZumoComFrame::~ZumoComFrame()
{
}

bool ZumoComFrame::appendData(uint8_t data)
{
    const uint8_t buf[1] = {data};
    return appendData(buf, 1U);
}

bool ZumoComFrame::appendData(uint16_t data)
{
    uint8_t MSB = ((data & 0xFF00) >> 8U);
    uint8_t LSB = (data & 0x00FF);

    // Big Endian
    const uint8_t buf[2] = {MSB, LSB};

    return appendData(buf, 2U);
}

bool ZumoComFrame::appendData(const uint8_t* data, uint8_t length)
{
    bool isOK = true;

    // Check if length + dlc smaller is than max data bytes

    // For-loop

    // // Copy Data Byte to m_data
    // // Add (data byte + checksum + 1) (+1 because of dlc)
    // // New checksum = checksum % UINT8_MAX

    return isOK;
}

uint8_t ZumoComFrame::getFrame(uint8_t* buffer)
{
    uint8_t frameLength = CHANNEL_LEN + DLC_LEN + CHECKSUM_LEN;

    // Copy channel
    // Copy dlc
    // Copy data bytes according to dlc. frameLength++ for each copied byte
    // Copy checksum

    return frameLength;
}

uint8_t ZumoComFrame::getFrameLength()
{
    return (CHANNEL_LEN + DLC_LEN + m_dlc + CHECKSUM_LEN);
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
