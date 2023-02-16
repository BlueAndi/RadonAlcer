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
 *  @brief  Frame Definition for Comm Protocol
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

#ifndef FRAME_H_
#define FRAME_H_

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

typedef struct Frame
{
    /** Channel Field Length in Bytes */
    static const uint8_t CHANNEL_LEN = 1U;

    /** DLC Field Length in Bytes */
    static const uint8_t DLC_LEN = 1U;

    /** Data Field Length in Bytes */
    static const uint8_t MAX_DATA_LEN = 8U;

    /** Checksum Field Length in Bytes */
    static const uint8_t CHECKSUM_LEN = 1U;

    /** Total Frame Length in Bytes */
    static const uint8_t MAX_FRAME_LEN = CHANNEL_LEN + DLC_LEN + MAX_DATA_LEN + CHECKSUM_LEN;

    /** Max Number of Channels defined by the header */
    static const uint8_t MAX_CHANNEL_NUM = CHANNEL_LEN * UINT8_MAX;

    /** Maximum Channel Priority */
    static const uint8_t MAX_CHANNEL_PRIO = 0U;

    /** Minimum Channel Priority */
    static const uint8_t MIN_CHANNEL_PRIO = MAX_CHANNEL_NUM;

    /** Data container of the Frame Fields */
    union
    {
        struct
        {
            /** Channel ID */
            uint8_t m_channel;

            /** Data Length */
            uint8_t m_dlc;

            /** Data of the Frame */
            uint8_t m_data[MAX_DATA_LEN];

            /** Frame Checksum */
            uint8_t m_checksum;
        };

        /** Complete Frame */
        uint8_t m_frame[MAX_FRAME_LEN];
    };

    /**
     *  Class Constructor.
     */
    Frame(uint8_t channel);

    /**
     *  Appends a byte to the frame.
     *  @param[in] data Byte to be appended.
     *  @returns true if byte has been succesfully appended.
     */
    bool appendData(uint8_t data);

    /**
     *  Appends a 16-bit number to the frame.
     *  @param[in] data 16-bit number to be appended.
     *  @returns true if bytes have been succesfully appended.
     */
    bool appendData(uint16_t data);

    /**
     *  Appends a 32-bit number to the frame.
     *  @param[in] data 32-bit number to be appended.
     *  @returns true if bytes have been succesfully appended.
     */
    bool appendData(uint32_t data);

    /**
     *  Appends a number of bytes to the frame.
     *  @param[in] data Bytes to be appended.
     *  @param[in] length Number of Bytes to append.
     *  @returns true if bytes have been succesfully appended.
     */
    bool appendData(const uint8_t* data, uint8_t length);

    /**
     *  Length of the Frame in Bytes.
     *  @returns Length of the Frame
     */
    uint8_t getFrameLength();
};

Frame::Frame(uint8_t channel) : m_channel(channel),
                                m_dlc(0U),
                                m_data{0U},
                                m_checksum(channel % UINT8_MAX)
{
}

bool Frame::appendData(uint8_t data)
{
    const uint8_t numberOfBytes      = 1U;
    const uint8_t buf[numberOfBytes] = {data};
    return appendData(buf, numberOfBytes);
}

bool Frame::appendData(uint16_t data)
{
    const uint8_t numberOfBytes = 2U;

    uint8_t MSB = ((data & 0xFF00) >> 8U);
    uint8_t LSB = (data & 0x00FF);

    // Big Endian
    const uint8_t buf[numberOfBytes] = {MSB, LSB};

    return appendData(buf, numberOfBytes);
}

bool Frame::appendData(uint32_t data)
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

bool Frame::appendData(const uint8_t* data, uint8_t length)
{
    bool isSuccess = false;

    // Check for enough space in frame
    if ((m_dlc + length) <= MAX_DATA_LEN)
    {
        for (uint8_t i = 0; i < length; i++)
        {
            m_data[m_dlc] = data[i];
            m_dlc++;
            m_checksum = ((m_checksum + data[i] + 1) % UINT8_MAX);
        }
        isSuccess = true;
    }

    return isSuccess;
}

uint8_t Frame::getFrameLength()
{
    return (CHANNEL_LEN + DLC_LEN + m_dlc + CHECKSUM_LEN);
}


#endif /* FRAME_H_ */