/*!
 * \file
 * \brief GPIO header file
 * \author Dawid Babula
 * \email dbabula@adventurous.pl
 *
 * \par Copyright (C) Dawid Babula, 2019
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef GPIO_H
#define GPIO_H

/*!
 *
 * \addtogroup gpio
 * \ingroup drivers
 * \brief GPIO implementation
 */

/*@{*/

#include <stdint.h>
#include <stdbool.h>

/*!
 *
 * \addtogroup gpio_ports
 * \ingroup gpio
 * \brief GPIO ports ids
 */
/*@{*/
/*! \brief Gpio port D id  */
#define GPIO_PORTD              (0U)
/*! \brief Gpio port C id  */
#define GPIO_PORTC              (1U)
/*! \brief Gpio port B id  */
#define GPIO_PORTB              (2U)
/*@}*/

/*!
 *
 * \addtogroup gpio_modes
 * \ingroup gpio
 * \brief GPIO modes
 */
/*@{*/

/*! \brief Gpio output mode as push-pull  */
#define GPIO_OUTPUT_PUSH_PULL   (0U)
/*! \brief Gpio output mode as input floating  */
#define GPIO_INPUT_FLOATING     (1U)
/*! \brief Gpio output mode as input with pull up resistor  */
#define GPIO_INPUT_PULL_UP      (2U)

/*@}*/

/*!
 * \brief Gpio configuration structure
 */
typedef struct
{
    uint8_t port; /*!< Gpio port*/
    uint8_t pin; /*!< Gpio pin */
    uint8_t mode; /*!< Gpio mode */
    bool init_value; /*!< Gpio initial value, relavant for \ref GPIO_OUTPUT_PUSH_PULL */
} GPIO_config_t;

/*! \todo (DB) document how below function behaves in case gpio is in output mode */
/*!
 * \brief Gets state of gpio in input mode.
 *
 * \param id gpio identification from global config structure
 *
 * \retval true high state
 * \retval false low state
 */
bool GPIO_read_pin(uint8_t id);

/*! \todo (DB) document how below function behaves in case gpio is in input mode */
/*!
 * \brief Sets gpio state in output mode.
 *
 * \param id gpio identification from global config structure
 * \param is_high state to be set, true=high
 *
 */
void GPIO_write_pin(uint8_t id, bool is_high);

/*!
 * \brief Toggles gpio state in output mode.
 *
 * First \ref GPIO_read_pin is invoked and then \ref GPIO_write_pin
 * with inverted result of \ref GPIO_read_pin
 *
 * \param id gpio identification from global config structure
 *
 */
void GPIO_toggle_pin(uint8_t id);

/*!
 * \brief Configures gpio in given mode.
 *
 * \param id gpio identification from global config structure
 * \param mode gpio mode \ref gpio_modes
 *
 */
void GPIO_config_pin(uint8_t id, uint8_t mode);

/*!
 * \brief Configures gpio driver
 *
 * \param is_global_pullup \todo (DB) documents this later
 *
 */
void GPIO_configure(bool is_global_pullup);

/*@}*/
#endif
