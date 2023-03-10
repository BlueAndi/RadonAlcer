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
 * @brief  The physical robot board realization.
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup HALTarget
 *
 * @{
 */
#ifndef BOARD_H
#define BOARD_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <IBoard.h>
#include <ButtonA.h>
#include <ButtonB.h>
#include <ButtonC.h>
#include <Buzzer.h>
#include <Display.h>
#include <Encoders.h>
#include <LineSensors.h>
#include <Motors.h>
#include <LedRed.h>
#include <LedYellow.h>
#include <LedGreen.h>
#include <ProximitySensors.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The concrete physical board.
 */
class Board : public IBoard
{
public:

    /**
     * Get board instance.
     * 
     * @return Board instance
     */
    static Board& getInstance()
    {
        static Board instance; /* idiom */

        return instance;
    }

    /**
     * Initialize the hardware.
     */
    void init() final;

    /**
     * Get button A driver.
     *
     * @return Button A driver.
     */
    IButton& getButtonA() final
    {
        return m_buttonA;
    }

    /**
     * Get button B driver.
     *
     * @return Button B driver.
     */
    IButton& getButtonB() final
    {
        return m_buttonB;
    }

    /**
     * Get button C driver.
     *
     * @return Button C driver.
     */
    IButton& getButtonC() final
    {
        return m_buttonC;
    }

    /**
     * Get buzzer driver.
     *
     * @return Buzzer driver.
     */
    IBuzzer& getBuzzer() final
    {
        return m_buzzer;
    }

    /**
     * Get LCD driver.
     *
     * @return LCD driver.
     */
    IDisplay& getDisplay() final
    {
        return m_display;
    }

    /**
     * Get encoders.
     *
     * @return Encoders driver.
     */
    IEncoders& getEncoders() final
    {
        return m_encoders;
    }

    /**
     * Get line sensors driver.
     *
     * @return Line sensor driver.
     */
    ILineSensors& getLineSensors() final
    {
        return m_lineSensors;
    }

    /**
     * Get motor driver.
     *
     * @return Motor driver.
     */
    IMotors& getMotors() final
    {
        return m_motors;
    }

    /**
     * Get red LED driver.
     *
     * @return Red LED driver.
     */
    ILed& getRedLed() final
    {
        return m_ledRed;
    }

    /**
     * Get yellow LED driver.
     *
     * @return Yellow LED driver.
     */
    ILed& getYellowLed() final
    {
        return m_ledYellow;
    }

    /**
     * Get green LED driver.
     *
     * @return Green LED driver.
     */
    ILed& getGreenLed() final
    {
        return m_ledGreen;
    }

    /**
     * Get proximity sensors driver.
     * 
     * @return Proximity sensors driver
     */
    IProximitySensors& getProximitySensors() final
    {
        return m_proximitySensors;
    }

protected:

private:

    /** Button A driver */
    ButtonA m_buttonA;

    /** Button B driver */
    ButtonB m_buttonB;

    /** Button C driver */
    ButtonC m_buttonC;

    /** Buzzer driver */
    Buzzer m_buzzer;

    /** Display driver */
    Display m_display;

    /** Encoders driver */
    Encoders m_encoders;

    /** Line sensors driver */
    LineSensors m_lineSensors;

    /** Motors driver */
    Motors m_motors;

    /** Red LED driver */
    LedRed m_ledRed;

    /** Red LED driver */
    LedYellow m_ledYellow;

    /** Red LED driver */
    LedGreen m_ledGreen;

    /** Proximity sensors */
    ProximitySensors m_proximitySensors;

    /**
     * Constructs the concrete board.
     */
    Board() :
        IBoard(),
        m_buttonA(),
        m_buttonB(),
        m_buttonC(),
        m_buzzer(),
        m_display(),
        m_encoders(),
        m_lineSensors(),
        m_motors(),
        m_ledRed(),
        m_ledYellow(),
        m_ledGreen(),
        m_proximitySensors()
    {
    }

    /**
     * Destroys the concrete board.
     */
    ~Board()
    {
    }

};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* BOARD_H */