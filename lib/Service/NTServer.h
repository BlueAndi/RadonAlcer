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
 * @brief  NT ComProtocol Server
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
#include <functional>

/******************************************************************************
 * Macros
 *****************************************************************************/

#define CONTROL_CHANNEL_NUMBER (0)

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 *  Class for the NT Server.
 *  @tparam maxChannels Number of channels that can be configured incl. the Control Channel.
 */
template<uint8_t maxChannels>
class NTServer
{
public:
    /**
     * Channel Notification Prototype Callback.
     * Provides the received data in the respective channel to the application.
     */
    typedef std::function<void(uint8_t* rcvData, uint8_t length)> ChannelCallback;

    /**
     * Construct the NT Server.
     */
    NTServer() : m_isSynced(false)
    {
        m_dataChannels[0] = new Channel("Control", CONTROL_CHANNEL_NUMBER,
            [this](uint8_t* data, uint8_t len) { this->callbackControlChannel(data, len); });
    }

    /**
     * Destroy the NT Server.
     */
    ~NTServer()
    {
    }

    /**
     * Manage the Server functions.
     * Call this function cyclic.
     */
    void process()
    {
        // Process RX data

        // Periodic Heartbeat
        heartbeat();
    }

private:
    /**
     * Channel Definition.
     */
    struct Channel
    {
        const char*     m_name;     /**< Name of the channel. */
        uint8_t         m_number;   /**< Number of the channel. */
        ChannelCallback m_callback; /**< Callback to provide received data to the application. */

        /**
         * Channel Constructor.
         */
        Channel(const char* name, uint8_t number, ChannelCallback cb) : m_name(name), m_number(number), m_callback(cb)
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
        /** Frame Fields */
        struct _Fields
        {
            /** Channel ID */
            uint8_t m_channel;

            /** Data Length */
            uint8_t m_dlc;

            /** Data of the Frame */
            uint8_t m_data[MAX_DATA_LEN];

            /** Frame Checksum */
            uint8_t m_checksum;
        } __attribute__((packed)) fields;

        /** Raw Frame Data */
        uint8_t raw[MAX_FRAME_LEN];

    } __attribute__((packed)) Frame;

    /**
     *  Array of Data Channels.
     */
    Channel* m_dataChannels[maxChannels];

    /**
     * Callback for the Control Channel
     */
    void callbackControlChannel(uint8_t* rcvData, uint8_t length)
    {
    }

    /**
     * Periodic heartbeat.
     * Sends SYNC Command depending on the current Sync state.
     */
    void heartbeat()
    {
        if (m_isSynced)
        {
            // Send every 5 seconds.
        }
        else
        {
            // Send every 1 second.
        }
    }

    /**
     * Current Sync state.
     */
    bool m_isSynced;

private:
    NTServer(const NTServer& avg);
    NTServer& operator=(const NTServer& avg);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* NT_SERVER_H */