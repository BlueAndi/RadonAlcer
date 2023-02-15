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

#ifndef ZUMO_COM_PROTOCOL_H_
#define ZUMO_COM_PROTOCOL_H_

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

/**
 *  Protocol for Communication with ZumoComSystem.
 */
class ZumoComFrame
{
public:
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

public:
    /**
     *  Class Constructor.
     */
    ZumoComFrame(uint8_t channel);

    /**
     *  Default Destructor.
     */
    ~ZumoComFrame();

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
     *  Appends a number of bytes to the frame.
     *  @param[in] data Bytes to be appended.
     *  @param[in] length Number of Bytes to append.
     *  @returns true if bytes have been succesfully appended.
     */
    bool appendData(const uint8_t* data, uint8_t length);

    /**
     *  Packs and writes the complete frame to a buffer.
     *  Includes checksum.
     *  Buffer must be at least MAX_FRAME_LEN bytes long.
     *  @param[in] buffer Buffer to write bytes to.
     *  @returns Total length of the frame in bytes.
     */
    uint8_t getFrame(uint8_t* buffer);

    /**
     *  Length of the Frame in Bytes.
     *  @returns Length of the Frame
     */
    uint8_t getFrameLength();

private:
    /** Channel ID */
    uint8_t m_channel;

    /** Data Length */
    uint8_t m_dlc;

    /** Data of the Frame */
    uint8_t m_data[MAX_DATA_LEN];

    /** Frame Checksum */
    uint8_t m_checksum;

private:
    /**
     *  Default Constructor.
     *  Not allowed to be called.
     */
    ZumoComFrame();
};

#endif /* ZUMO_COM_PROTOCOL_H_ */