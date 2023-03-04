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

#ifndef NT_SERVER_H
#define NT_SERVER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

#define CONTROL_CHANNEL_NUMBER   (0)    /**< Number of Control Channel */
#define HEARTBEAT_PAYLOAD_LENGTH (5)    /**< DLC of Heartbeat Command */
#define HEATBEAT_PERIOD_SYNCED   (5000) /**< Period of Heartbeat when Synced */
#define HEATBEAT_PERIOD_UNSYNCED (1000) /**< Period of Heartbeat when Unsynced */

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
     * Channel Notification Prototype Callback.
     * Provides the received data in the respective channel to the application.
     */
    typedef void (* ChannelCallback)(uint8_t* rcvData, uint8_t length);

    /**
     * Construct the YAP Server.
     */
    YAPServer() : m_dataChannels{nullptr}, m_isSynced(false), m_lastSyncCommand(0U)
    {
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

private:
    /**
     * Channel Definition.
     */
    struct Channel
    {
        const char*     m_name;     /**< Name of the channel. */
        uint8_t         m_number;   /**< Number of the channel. */
        uint8_t         m_dlc;      /**< Payload length of channel */
        ChannelCallback m_callback; /**< Callback to provide received data to the application. */

        /**
         * Channel Constructor.
         */
        Channel(const char* name, uint8_t number, uint8_t dlc ChannelCallback cb) :
                m_name(name),
                m_number(number),
                m_dlc(dlc),
                m_callback(cb)
        {
        }
    };

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

    /** Data container of the Frame Fields */
    typedef union _Frame
    {
        union _Fields
        {
            /** Frame Fields */
            struct _Header
            {
                /** Channel ID */
                uint8_t m_channel;

                /** Data Length */
                uint8_t m_dlc;

                /** Frame Checksum */
                uint8_t m_checksum;

            } __attribute__((packed)) header;

            struct _Payload
            {
                /** Data of the Frame */
                uint8_t m_data[MAX_DATA_LEN];

            } __attribute__((packed)) payload;

        } __attribute__((packed)) fields;

        /** Raw Frame Data */
        uint8_t raw[MAX_FRAME_LEN] = {0};

    } __attribute__((packed)) Frame;

    /**
     * Enumeration of Commands of Control Channel.
     */
    enum COMMANDS : uint8_t
    {
        SYNC = 0x00, /**< SYNC Command */
        SYNC_RSP,    /**< SYNC Response */
        SCRB,        /**< Subscribe Command */
        SCRB_RSP     /**< Subscribe Response */
    };

    /**
     *  Array of Data Channels.
     */
    Channel* m_dataChannels[maxChannels];

    /**
     * Callback for the Control Channel
     */
    void callbackControlChannel(uint8_t* rcvData, uint8_t length)
    {
        if (0U != length)
        {
            uint8_t cmdByte = rcvData[0];

            switch (cmdByte)
            {
            case COMMANDS::SYNC_RSP:
            {
                uint32_t rcvTimestamp = ((uint32_t) rcvData[1] << 24) |
                                        ((uint32_t) rcvData[2] << 16) |
                                        ((uint32_t) rcvData[3] << 8) |
                                        ((uint32_t) rcvData[4]);

                // Check Timestamp with m_lastSyncCommand
                if (0U == (rcvTimestamp - m_lastSyncCommand))
                {
                    m_lastSyncResponse = m_lastSyncCommand;
                    m_isSynced = true;
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
    }

    /**
     * Receive and process RX Data.
     */
    void processRxData()
    {
        // Check for received data
        // TODO: Change rcvData for Serial.read();
        uint8_t rcvData[MAX_FRAME_LEN] = {0};

        // Create Frame and copy data into "raw" field.
        Frame rcvFrame;
        memcpy(&rcvFrame.raw, rcvData, MAX_FRAME_LEN);

        // Determine which callback to call, if any.
        if(CONTROL_CHANNEL_NUMBER == rcvFrame.fields.header.m_channel)
        {
            callbackControlChannel(rcvFrame.fields.payload.m_data, rcvFrame.fields.header.m_dlc);
        }
        else if (nullptr != m_dataChannels[rcvFrame.fields.header.m_channel])
        {
            // Callback
            m_dataChannels[rcvFrame.fields.header.m_channel]->m_callback(rcvFrame.fields.payload.m_data, rcvFrame.fields.header.m_dlc);
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
            if (0U != (m_lastSyncCommand - m_lastSyncResponse))
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

            uint8_t buf[HEARTBEAT_PAYLOAD_LENGTH] = {COMMANDS::SYNC, hiMSB, hiLSB, lowMSB, lowLSB};

            send(CONTROL_CHANNEL_NUMBER, buf, HEARTBEAT_PAYLOAD_LENGTH);
            m_lastSyncCommand = currentTimestamp;
        }
    }

    /**
     * Send a frame with the selected bytes.
     * @param[in] channel Channel to send frame to.
     * @param[in] data Byte buffer to be sent.
     * @param[in] length Amount of bytes to send.
     */
    void send(uint8_t channel, const uint8_t* data, uint8_t length)
    {
        if ((MAX_DATA_LEN >= length) && (m_isSynced || (CONTROL_CHANNEL_NUMBER == channel)))
        {
            Frame newFrame;
            newFrame.fields.header.m_channel  = channel;
            newFrame.fields.header.m_checksum = channel % UINT8_MAX;

            for (uint8_t i = 0; i < length; i++)
            {
                newFrame.fields.payload.m_data[i] = data[i];
                newFrame.fields.header.m_dlc++;
                newFrame.fields.header.m_checksum = ((newFrame.fields.header.m_checksum + data[i] + 1) % UINT8_MAX);
            }

            if (isFrameValid(newFrame))
            {
                Serial.write(newFrame.raw, MAX_FRAME_LEN);
            }
        }
    }

    /**
     * Check if a Frame is valid using its checksum.
     * @param[in] frame Frame to be checked.
     * @returns true if the Frame's checksum is correct.
     */
    bool isFrameValid(const Frame& frame)
    {
        uint8_t newChecksum = 0;

        for (uint8_t i = 0; i < (MAX_FRAME_LEN - 1); i++)
        {
            newChecksum += (frame.raw[i] % UINT8_MAX);
        }

        // Frame is valid when both checksums are the same.
        return !(newChecksum - frame.fields.header.m_checksum);
    }

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

#endif /* NT_SERVER_H */