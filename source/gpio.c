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

#include "gpio.h"
#include <stddef.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "debug.h"


#define REG_BY_PORT(base_reg, port_no, port_offset)  \
    *((volatile uint8_t *)((volatile uint8_t *)&base_reg + port_no * port_offset))

#define DDR(port_no)    REG_BY_PORT(DDRD, port_no, 0x03)
#define PORT(port_no)   REG_BY_PORT(PORTD, port_no, 0x03)
#define PIN(port_no)    REG_BY_PORT(PIND, port_no, 0x03)

#if GPIO_DYNAMIC_CHECK == 1U

//static const char debug_prefix[] PROGMEM = "GPIO";
static const char debug_prefix[] = "GPIO";

static inline bool is_port_pin_valid(uint8_t port, uint8_t pin)
{
    switch(port)
    {
        case GPIO_PORTB:
            /* no break */
        case GPIO_PORTD:
            if(pin <= 7U)
            {
                return true;
            }
            break;
        case GPIO_PORTC:
            if(pin <= 6U)
            {
                return true;
            }
            break;
        default:
            break;
    }

    return false;
}

static inline bool is_mode_valid(uint8_t mode)
{
    switch(mode)
    {
        case GPIO_OUTPUT_PUSH_PULL:
            /* no break */
        case GPIO_INPUT_FLOATING:
            /* no break */
        case GPIO_INPUT_PULL_UP:
            return true;
        default:
            return false;
    }
}
#endif

bool GPIO_read_pin(uint8_t port, uint8_t pin)
{
#if GPIO_DYNAMIC_CHECK == 1U
    uint16_t line;
    if(!is_port_pin_valid(port, pin))
    {
        line = __LINE__;
        goto error;
    }
#endif

    return ((PIN(port) & ( 1 << pin)) != 0);

#if GPIO_DYNAMIC_CHECK == 1U
    error:
        DEBUG_halt(debug_prefix, line);
        return false;
#endif
}

void GPIO_write_pin(uint8_t port, uint8_t pin, bool is_high)
{
#if GPIO_DYNAMIC_CHECK == 1U
    uint16_t line;

    if(!is_port_pin_valid(port, pin))
    {
        line = __LINE__;
        goto error;
    }
#endif

    if(is_high)
    {
        PORT(port) |= (1 << pin);
    }
    else
    {
        PORT(port)  &= ~(1 << pin);
    }

    return;

#if GPIO_DYNAMIC_CHECK == 1U
    error:
        DEBUG_halt(debug_prefix, line);
#endif
}

void GPIO_config_pin(uint8_t port, uint8_t pin, uint8_t mode)
{
#if GPIO_DYNAMIC_CHECK == 1U
    uint16_t line;

    if(!is_mode_valid(mode))
    {
        line = __LINE__;
        goto error;
    }

    if(!is_port_pin_valid(port, pin))
    {
        line = __LINE__;
        goto error;
    }
#endif

    DDR(port) &= ~(1 << pin);
    PORT(port)  &= ~(1 << pin);

    switch(mode)
    {
        case GPIO_OUTPUT_PUSH_PULL:
            DDR(port) |= (1 << pin);
            break;
        case GPIO_INPUT_FLOATING:
            break;
        case GPIO_INPUT_PULL_UP:
            PORT(port) |= (1 << pin);
            break;
    }

    return;

#if GPIO_DYNAMIC_CHECK == 1U
    error:
        DEBUG_halt(debug_prefix, line);
#endif
}

void GPIO_configure(bool is_global_pullup)
{
    if(is_global_pullup)
    {
        SFIOR |= (1 << PUD);
    }
    else
    {
        SFIOR &= ~(1 << PUD);
    }
}
