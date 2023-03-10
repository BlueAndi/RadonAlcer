/* MIT License
 *
 * Copyright (c) 2019 - 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  The simulation robot board realization.
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Board.h>

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

/* Name of the speaker in the robot simulation. */
const char* Board::SPEAKER_NAME = "robot_speaker";

/* Name of the display in the robot simulation. */
const char* Board::DISPLAY_NAME = "robot_display";

/* Name of the left motor in the robot simulation. */
const char* Board::LEFT_MOTOR_NAME = "motor_left";

/* Name of the right motor in the robot simulation. */
const char* Board::RIGHT_MOTOR_NAME = "motor_right";

/* Name of the infrared emitter 0 in the robot simulation. */
const char* Board::EMITTER_0_NAME = "emitter_l";

/* Name of the infrared emitter 1 in the robot simulation. */
const char* Board::EMITTER_1_NAME = "emitter_lm";

/* Name of the infrared emitter 2 in the robot simulation. */
const char* Board::EMITTER_2_NAME = "emitter_m";

/* Name of the infrared emitter 3 in the robot simulation. */
const char* Board::EMITTER_3_NAME = "emitter_rm";

/* Name of the infrared emitter 4 in the robot simulation. */
const char* Board::EMITTER_4_NAME = "emitter_r";

/** Name of the position sensor of the left motor in the robot simulation. */
const char* Board::POS_SENSOR_LEFT_NAME = "position_sensor_left";

/** Name of the position sensor of the right motor in the robot simulation. */
const char* Board::POS_SENSOR_RIGHT_NAME = "position_sensor_right";

/* Name of the light sensor 0 in the robot simulation. */
const char* Board::LIGHT_SENSOR_0_NAME = "lightsensor_l";

/* Name of the light sensor 1 in the robot simulation. */
const char* Board::LIGHT_SENSOR_1_NAME = "lightsensor_lm";

/* Name of the light sensor 2 in the robot simulation. */
const char* Board::LIGHT_SENSOR_2_NAME = "lightsensor_m";

/* Name of the light sensor 3 in the robot simulation. */
const char* Board::LIGHT_SENSOR_3_NAME = "lightsensor_rm";

/* Name of the light sensor 4 in the robot simulation. */
const char* Board::LIGHT_SENSOR_4_NAME = "lightsensor_r";

/* Name of the red LED in the robot simulation. */
const char* Board::LED_RED_NAME = "led_red";

/* Name of the yellow LED in the robot simulation. */
const char* Board::LED_YELLOW_NAME = "led_yellow";

/* Name of the green LED in the robot simulation. */
const char* Board::LED_GREEN_NAME = "led_green";

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

void Board::init()
{
    m_lineSensors.init();
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/
