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
 * @brief  Class for the YAP Server.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "YAPServer.hpp"

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

static void controlChannelCallback(const uint8_t* rcvData);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

YAPServer YAPServer::m_instance;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

YAPServer& YAPServer::getInstance()
{
    return m_instance;
}

void YAPServer::process()
{
    // Process RX data
    processRxData();

    // Periodic Heartbeat
    heartbeat();
}

void YAPServer::sendData(uint8_t channel, const uint8_t* data, uint8_t length)
{
    if (CONTROL_CHANNEL_NUMBER != channel)
    {
        send(channel, data, length);
    }
}

uint8_t YAPServer::createChannel(const char* channelName, uint8_t dlc, ChannelCallback cb)
{
    uint8_t itr;

    for (itr = 0; itr < YAP_MAXCHANNELS; itr++)
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

void YAPServer::printChannels()
{
    for (uint8_t i = 0; i < YAP_MAXCHANNELS; i++)
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

void YAPServer::callbackControlChannel(const uint8_t* rcvData)
{
    uint8_t cmdByte = rcvData[0];

    switch (cmdByte)
    {
    case COMMANDS::SYNC_RSP:
    {
        uint32_t rcvTimestamp = ((uint32_t)rcvData[1] << 24) | ((uint32_t)rcvData[2] << 16) |
                                ((uint32_t)rcvData[3] << 8) | ((uint32_t)rcvData[4]);

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
        char channelName[CHANNEL_NAME_MAX_LEN + 1];
        memcpy(channelName, &rcvData[1], CHANNEL_NAME_MAX_LEN);
        channelName[CHANNEL_NAME_MAX_LEN] = '\0';

        uint8_t itr;

        for (itr = 0; itr < YAP_MAXCHANNELS; itr++)
        {
            if (0U == strncmp(channelName, m_dataChannels[itr]->m_name, CHANNEL_NAME_MAX_LEN))
            {
                // Channel name found. Send SRCB_RSP
                uint8_t buf[CONTROL_CHANNEL_PAYLOAD_LENGTH] = {COMMANDS::SCRB_RSP, itr, m_dataChannels[itr]->m_dlc};
                send(CONTROL_CHANNEL_NUMBER, buf, CONTROL_CHANNEL_PAYLOAD_LENGTH);
                break;
            }
        }

        break;
    }

    default:
    {
        break;
    }
    }
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/
YAPServer::YAPServer() : m_dataChannels{nullptr}, m_isSynced(false), m_lastSyncCommand(0U), m_lastSyncResponse(0U)
{
    createChannel("CONTROL", CONTROL_CHANNEL_PAYLOAD_LENGTH, controlChannelCallback);
}

YAPServer::~YAPServer()
{
}

void YAPServer::processRxData()
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
                m_dataChannels[rcvFrame.fields.header.headerFields.m_channel]->m_callback(
                    rcvFrame.fields.payload.m_data);
            }
        }
    }
}

void YAPServer::heartbeat()
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

void YAPServer::send(uint8_t channel, const uint8_t* data, uint8_t payloadLength)
{
    if ((MAX_DATA_LEN >= payloadLength) && (m_isSynced || (CONTROL_CHANNEL_NUMBER == channel)))
    {
        const uint8_t frameLength = HEADER_LEN + payloadLength;
        Frame         newFrame;
        uint32_t      sum                             = channel;
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

bool YAPServer::isFrameValid(const Frame& frame, uint8_t payloadLength)
{
    uint32_t sum = frame.fields.header.headerFields.m_channel;

    for (uint8_t i = 0; i < payloadLength; i++)
    {
        sum += frame.fields.payload.m_data[i];
    }

    // Frame is valid when both checksums are the same.
    return ((sum % 255) == frame.fields.header.headerFields.m_checksum);
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

void controlChannelCallback(const uint8_t* rcvData)
{
    YAPServer::getInstance().callbackControlChannel(rcvData);
}