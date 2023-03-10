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
 * @brief  Yet Another Protocol (YAP) Server
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */

#ifndef YAP_SERVER_H
#define YAP_SERVER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <YAPCommon.hpp>
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/**
 * Maximum allowed of configurable channels. 
 */
#ifndef YAP_MAXCHANNELS
    #define YAP_MAXCHANNELS (10)
#endif

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 *  Class for the YAP Server.
 */
class YAPServer
{
public:

    /**
     * Get YAPServer instance.
     *
     * @return YAPServer instance
     */
    static YAPServer& getInstance();

    /**
     * Manage the Server functions.
     * Call this function cyclic.
     */
    void process();

    /**
     * Send a frame with the selected bytes.
     * @param[in] channel Channel to send frame to.
     * @param[in] data Byte buffer to be sent.
     * @param[in] length Amount of bytes to send.
     */
    void sendData(uint8_t channel, const uint8_t* data, uint8_t length);

    /**
     * Creates a new channel on the server.
     * @param[in] channelName Name of the channel.
     * It will not be checked if the name already exists.
     * @param[in] dlc Length of the payload of this channel.
     * @param[in] cb Callback for data incoming to this channel.
     * @returns The channel number if succesfully created, or 0 if not able to create new channel.
     */
    uint8_t createChannel(const char* channelName, uint8_t dlc, ChannelCallback cb);

    /**
     * Debug Function. Print all channel info.
     */
    void printChannels();

    /**
     * Callback for the Control Channel
     * @param[in] rcvData Payload of received frame.
     */
    void callbackControlChannel(const uint8_t* rcvData);

private:
    /**
     * Receive and process RX Data.
     */
    void processRxData();

    /**
     * Periodic heartbeat.
     * Sends SYNC Command depending on the current Sync state.
     */
    void heartbeat();

    /**
     * Send a frame with the selected bytes.
     * @param[in] channel Channel to send frame to.
     * @param[in] data Byte buffer to be sent.
     * @param[in] length Amount of bytes to send.
     */
    void send(uint8_t channel, const uint8_t* data, uint8_t payloadLength);

    /**
     * Check if a Frame is valid using its checksum.
     * @param[in] frame Frame to be checked.
     * @returns true if the Frame's checksum is correct.
     */
    bool isFrameValid(const Frame& frame, uint8_t payloadLength);

private:
    /**
     * Instance of YAP Server.
     */
    static YAPServer m_instance;

    /**
     *  Array of Data Channels.
     */
    Channel* m_dataChannels[YAP_MAXCHANNELS];

    /**
     * Current Sync state.
     */
    bool m_isSynced;

    /**
     * Last Heartbeat timestamp.
     */
    uint32_t m_lastSyncCommand;

    /**
     * Last sync response timestamp.
     */
    uint32_t m_lastSyncResponse;

private:

    /**
     * Construct the YAP Server.
     */
    YAPServer();

    /**
     * Destroy the YAP Server.
     */
    ~YAPServer();

    /** Prevent instance copying */
    YAPServer(const YAPServer& avg);
    YAPServer& operator=(const YAPServer& avg);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* YAP_SERVER_H */