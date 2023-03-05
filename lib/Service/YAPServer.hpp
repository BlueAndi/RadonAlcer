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

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 *  Class for the YAP Server.
 *  @tparam maxChannels Number of data channels that can be configured.
 */
template<uint8_t maxChannels>
class YAPServer
{
public:
    /**
     * Construct the YAP Server.
     * @param[in] controlCallback Callback of control channel.
     */
    YAPServer(ChannelCallback controlCallback) : m_dataChannels{nullptr},
                  m_isSynced(false),
                  m_lastSyncCommand(0U),
                  m_lastSyncResponse(0U)
    {
        createChannel("CONTROL", CONTROL_CHANNEL_PAYLOAD_LENGTH, controlCallback);
    }

    /**
     * Destroy the YAP Server.
     */
    ~YAPServer()
    {
    }

    /**
     * Manage the Server functions.
     * Call this function cyclic.
     */
    void process()
    {
        // Process RX data
        processRxData();

        // Periodic Heartbeat
        heartbeat();
    }

    /**
     * Send a frame with the selected bytes.
     * @param[in] channel Channel to send frame to.
     * @param[in] data Byte buffer to be sent.
     * @param[in] length Amount of bytes to send.
     */
    void sendData(uint8_t channel, const uint8_t* data, uint8_t length)
    {
        if (CONTROL_CHANNEL_NUMBER != channel)
        {
            send(channel, data, length);
        }
    }

    /**
     * Creates a new channel on the server.
     * @param[in] channelName Name of the channel.
     * It will not be checked if the name already exists.
     * @param[in] dlc Length of the payload of this channel.
     * @param[in] cb Callback for data incoming to this channel.
     * @returns The channel number if succesfully created, or 0 if not able to create new channel.
     */
    uint8_t createChannel(const char* channelName, uint8_t dlc, ChannelCallback cb)
    {
        uint8_t itr;

        for (itr = 0; itr < maxChannels; itr++)
        {
            if (nullptr == m_dataChannels[itr])
            {
                m_dataChannels[itr] = new Channel(channelName, itr, dlc, cb);

                if (nullptr == m_dataChannels[itr])
                {
                    itr = 0U;
                }

                break;
            }
        }

        return itr;
    }

    /**
     * Debug Function. Print all channel info.
     */
    void printChannels()
    {
        for (uint8_t i = 0; i < maxChannels; i++)
        {
            if (nullptr == m_dataChannels[i])
            {
                Serial.print("Channel ");
                Serial.print(i);
                Serial.println(": Nullptr");
            }
            else
            {
                Serial.print("Channel ");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(m_dataChannels[i]->m_name);
                Serial.print(" --- DLC: ");
                Serial.println(m_dataChannels[i]->m_dlc);
            }
        }
        Serial.println("--------------------------");
    }

    /**
     * Callback for the Control Channel
     * @param[in] rcvData Payload of received frame.
     */
    void callbackControlChannel(const uint8_t* rcvData)
    {

        uint8_t cmdByte = rcvData[0];

        switch (cmdByte)
        {
        case COMMANDS::SYNC_RSP:
        {
            uint32_t rcvTimestamp = ((uint32_t)rcvData[1] << 24) |
                                    ((uint32_t)rcvData[2] << 16) |
                                    ((uint32_t)rcvData[3] << 8)  |
                                    ((uint32_t)rcvData[4]);

            // Check Timestamp with m_lastSyncCommand
            if (rcvTimestamp == m_lastSyncCommand)
            {
                m_lastSyncResponse = m_lastSyncCommand;
                m_isSynced         = true;
            }

            break;
        }

        case COMMANDS::SCRB:
        {
            break;
        }

        default:
        {
            break;
        }
        }
    }

private:
    /**
     * Receive and process RX Data.
     */
    void processRxData()
    {
        // Check for received data
        if (Serial.available())
        {
            // Create Frame and read header
            Frame rcvFrame;
            Serial.readBytes(rcvFrame.fields.header.rawHeader, HEADER_LEN);

            // Determine which callback to call, if any.
            if (nullptr != m_dataChannels[rcvFrame.fields.header.headerFields.m_channel])
            {
                uint8_t payloadLength = m_dataChannels[rcvFrame.fields.header.headerFields.m_channel]->m_dlc;

                // Read Payload
                Serial.readBytes(rcvFrame.fields.payload.m_data, payloadLength);

                if (isFrameValid(rcvFrame, payloadLength))
                {
                    // Callback
                    m_dataChannels[rcvFrame.fields.header.headerFields.m_channel]->m_callback(rcvFrame.fields.payload.m_data);
                }
            }
        }
    }

    /**
     * Periodic heartbeat.
     * Sends SYNC Command depending on the current Sync state.
     */
    void heartbeat()
    {
        uint32_t heartbeatPeriod  = HEATBEAT_PERIOD_UNSYNCED;
        uint32_t currentTimestamp = millis();

        if (m_isSynced)
        {
            heartbeatPeriod = HEATBEAT_PERIOD_SYNCED;
        }

        if ((currentTimestamp - m_lastSyncCommand) >= heartbeatPeriod)
        {
            // Timeout
            if (m_lastSyncCommand != m_lastSyncResponse)
            {
                m_isSynced = false;
            }
            
            // Send SYNC Command
            uint16_t hiBytes  = ((currentTimestamp & 0xFFFF0000) >> 16U);
            uint16_t lowBytes = (currentTimestamp & 0x0000FFFF);

            uint8_t hiMSB  = ((hiBytes & 0xFF00) >> 8U);
            uint8_t hiLSB  = (hiBytes & 0x00FF);
            uint8_t lowMSB = ((lowBytes & 0xFF00) >> 8U);
            uint8_t lowLSB = (lowBytes & 0x00FF);

            uint8_t buf[CONTROL_CHANNEL_PAYLOAD_LENGTH] = {COMMANDS::SYNC, hiMSB, hiLSB, lowMSB, lowLSB};

            send(CONTROL_CHANNEL_NUMBER, buf, CONTROL_CHANNEL_PAYLOAD_LENGTH);
            m_lastSyncCommand = currentTimestamp;
        }
    }

    /**
     * Send a frame with the selected bytes.
     * @param[in] channel Channel to send frame to.
     * @param[in] data Byte buffer to be sent.
     * @param[in] length Amount of bytes to send.
     */
    void send(uint8_t channel, const uint8_t* data, uint8_t payloadLength)
    {
        if ((MAX_DATA_LEN >= payloadLength) && (m_isSynced || (CONTROL_CHANNEL_NUMBER == channel)))
        {
            const uint8_t frameLength = HEADER_LEN + payloadLength;
            Frame         newFrame;
            uint32_t      sum                = channel;
            newFrame.fields.header.headerFields.m_channel = channel;

            for (uint8_t i = 0; i < payloadLength; i++)
            {
                newFrame.fields.payload.m_data[i] = data[i];
                sum += data[i];
            }

            newFrame.fields.header.headerFields.m_checksum = (sum % UINT8_MAX);

            Serial.write(newFrame.raw, frameLength);
        }
    }

    /**
     * Check if a Frame is valid using its checksum.
     * @param[in] frame Frame to be checked.
     * @returns true if the Frame's checksum is correct.
     */
    bool isFrameValid(const Frame& frame, uint8_t payloadLength)
    {
        uint32_t sum = frame.fields.header.headerFields.m_channel;

        for (uint8_t i = 0; i < payloadLength; i++)
        {
            sum += frame.fields.payload.m_data[i];
        }

        // Frame is valid when both checksums are the same.
        return ((sum % 255) == frame.fields.header.headerFields.m_checksum);
    }

private:
    /**
     *  Array of Data Channels.
     */
    Channel* m_dataChannels[maxChannels];

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
    YAPServer(const YAPServer& avg);
    YAPServer& operator=(const YAPServer& avg);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* YAP_SERVER_H */