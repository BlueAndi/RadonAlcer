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
 * @brief  Parameter state
 * @author Andreas Merkle <web@blue-andi.de>
 * 
 * @addtogroup Application
 *
 * @{
 */

#ifndef PARAMETER_SETS_H
#define PARAMETER_SETS_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Parameter set with different driving configurations. */
class ParameterSets
{
public:
    /**
     * A single parameter set.
     */
    struct ParameterSet
    {
        const char* name;          /**< Name of the parameter set */
        int16_t     topSpeed;      /**< Top speed in steps/s */
        int16_t     kPNumerator;   /**< Kp numerator value */
        int16_t     kPDenominator; /**< Kp denominator value */
        int16_t     kINumerator;   /**< Ki numerator value */
        int16_t     kIDenominator; /**< Ki denominator value */
        int16_t     kDNumerator;   /**< Kd numerator value */
        int16_t     kDDenominator; /**< Kd denominator value */
    };

    /**
     * Get parameter set instance.
     *
     * @return Parameter set instance.
     */
    static ParameterSets& getInstance()
    {
        static ParameterSets instance;

        /* Singleton idiom to force initialization during first usage. */

        return instance;
    }

    /**
     * Choose a specific parameter set.
     * If a invalid set id is given, nothing changes.
     *
     * @param[in] setId Parameter set id
     */
    void choose(uint8_t setId);

    /**
     * Change to next parameter set.
     * After the last set, the first will be choosen.
     */
    void next();

    /**
     * Get current set id.
     *
     * @return Parameter set id.
     */
    uint8_t getCurrentSetId() const;

    /**
     * Get selected parameter set.
     *
     * @return Parameter set
     */
    const ParameterSet& getParameterSet() const;

    /** Max. number of parameter sets. */
    static const uint8_t MAX_SETS = 3;

protected:
private:
    uint8_t      m_currentSetId;      /**< Set id of current selected set. */
    ParameterSet m_parSets[MAX_SETS]; /**< All parameter sets */

    /**
     * Default constructor.
     */
    ParameterSets();

    /**
     * Default destructor.
     */
    ~ParameterSets();

    ParameterSets(const ParameterSets& set);
    ParameterSets& operator=(const ParameterSets& set);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PARAMETER_SET_H */