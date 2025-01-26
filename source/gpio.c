/*!
 * \file
 * \brief GPIO implementation file
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
#define DEBUG_APP_ID    "GPIO"
#define DEBUG_ENABLED   DEBUG_GPIO_ENABLED
#define DEBUG_LEVEL     DEBUG_GPIO_LEVEL
#include "gpio.h"
#include <stddef.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "debug.h"
#include "hardware.h"

#define REG_BY_PORT(base_reg, port_no, port_offset)  \
    *((volatile uint8_t *)((volatile uint8_t *)&(base_reg) + (port_no) * (port_offset)))

#define DDR(port_no)    REG_BY_PORT(DDRD, (port_no), 0x03)
#define PORT(port_no)   REG_BY_PORT(PORTD, (port_no), 0x03)
#define PIN(port_no)    REG_BY_PORT(PIND, (port_no), 0x03)

bool GPIO_read_pin(uint8_t id)
{
    const uint8_t port = pgm_read_byte(&gpio_config[id].port);
    const uint8_t pin = pgm_read_byte(&gpio_config[id].pin);

    return ((PIN(port) & ( 1u << pin)) != 0);
}

void GPIO_write_pin(uint8_t id, bool is_high)
{
    const uint8_t port = pgm_read_byte(&gpio_config[id].port);
    const uint8_t pin = pgm_read_byte(&gpio_config[id].pin);

    if(is_high)
    {
        PORT(port) |= (1u << pin);
    }
    else
    {
        PORT(port)  &= ~(1u << pin);
    }
}

void GPIO_toggle_pin(uint8_t id)
{
    bool val = GPIO_read_pin(id);
    GPIO_write_pin(id, !val);
}

void GPIO_config_pin(uint8_t id, uint8_t mode)
{
    const uint8_t port = pgm_read_byte(&gpio_config[id].port);
    const uint8_t pin = pgm_read_byte(&gpio_config[id].pin);

    DDR(port) &= ~(1u << pin);
    PORT(port)  &= ~(1u << pin);

    switch(mode)
    {
        case GPIO_OUTPUT_PUSH_PULL:
            DDR(port) |= (1u << pin);
            break;
        case GPIO_INPUT_FLOATING:
            break;
        case GPIO_INPUT_PULL_UP:
            PORT(port) |= (1u << pin);
            break;
        default:
            ASSERT(false);
            break;
    }
}

void GPIO_configure(bool is_global_pullup)
{
    if(is_global_pullup)
    {
        SFIOR |= (1u << PUD);
    }
    else
    {
        SFIOR &= ~(1u << PUD);
    }

    for(uint8_t i = 0; i < (sizeof(gpio_config)/sizeof(gpio_config[0])); i++)
    {
        const uint8_t mode = pgm_read_byte(&gpio_config[i].mode);
        GPIO_config_pin(i, mode);
    }
}
