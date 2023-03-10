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
 * @brief  Sound
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Sound.h>
#include <stdint.h>
#include <Board.h>
#include <Arduino.h>

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

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Alarm frequency in Hz. */
static const uint32_t ALARM_FREQ = 500;

/** Alarm tone duration in ms. */
static const uint32_t ALARM_DURATION = 333;

/** Silence duration in ms, between two consecutive alarm tones. */
static const uint32_t SILENCE_DURATION = 333;

/** Beep frequency in Hz. */
static const uint32_t BEEP_FREQ = 1000;

/** Beep tone duration in ms. */
static const uint32_t BEEP_DURATION = 333;

/** General volume in digits. */
static const uint8_t VOLUME = 10;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

void Sound::playAlarm()
{
    IBuzzer& buzzer = Board::getInstance().getBuzzer();

    /* Req. 3.4.5-2:
     * The Sound shall play two consecutive sounds of 500Hz frequency and 1/3s duration,
     * interrupted by 1/3s of silence.
     */
    buzzer.playFrequency(ALARM_FREQ, ALARM_DURATION, VOLUME);
    while (true == buzzer.isPlaying())
        ;
    delay(SILENCE_DURATION);
    buzzer.playFrequency(ALARM_FREQ, ALARM_DURATION, VOLUME);
}

void Sound::playBeep()
{
    IBuzzer& buzzer = Board::getInstance().getBuzzer();

    /* Req. 3.4.5-1:
     * The Sound shall play a sound of 1000Hz frequency and 1s duration.
     */
    buzzer.playFrequency(BEEP_FREQ, BEEP_DURATION, VOLUME);
}

void Sound::playStartup()
{
    IBuzzer& buzzer = Board::getInstance().getBuzzer();

    buzzer.playMelodyPGM(PSTR("O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"));
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
