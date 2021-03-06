/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Daniel Campora
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/mpstate.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#define TARGET_IS_TM4C123_RB2                                   
#include "rom.h"
#include "rom_map.h"

//#include "pin.h"
//#include "prcm.h"
#include "sysctl.h"
#include "gpio.h"
//#include "interrupt.h"
#include "pybpin.h"
#include "mpirq.h"
#include "pins.h"
//#include "pybsleep.h"
#include "mpexception.h"
#include "mperror.h"


/// \moduleref pyb
/// \class Pin - control I/O pins
///
/******************************************************************************
DECLARE PRIVATE FUNCTIONS
******************************************************************************/
STATIC pin_obj_t *pin_find_named_pin(const mp_obj_dict_t *named_pins, mp_obj_t name);
STATIC pin_obj_t *pin_find_pin_by_port_bit (const mp_obj_dict_t *named_pins, uint port, uint bit);
STATIC int8_t pin_obj_find_af (const pin_obj_t* pin, uint8_t fn, uint8_t unit, uint8_t type);
STATIC void pin_free_af_from_pins (uint8_t fn, uint8_t unit, uint8_t type);
STATIC void pin_deassign (pin_obj_t* pin);
STATIC void pin_obj_configure (const pin_obj_t *self);
STATIC void pin_get_hibernate_pin_and_idx (const pin_obj_t *self, uint *wake_pin, uint *idx);
STATIC void pin_irq_enable (mp_obj_t self_in);
STATIC void pin_irq_disable (mp_obj_t self_in);
STATIC void pin_extint_register(pin_obj_t *self, uint32_t intmode, uint32_t priority);
STATIC void pin_validate_mode (uint mode);
STATIC void pin_validate_type (uint type);
STATIC void pin_validate_drive (uint strength);
STATIC void pin_validate_af(const pin_obj_t* pin, int8_t idx, uint8_t *fn, uint8_t *unit, uint8_t *type);
STATIC uint8_t pin_get_value(const pin_obj_t* self);
//STATIC void GPIOA0IntHandler (void);
//STATIC void GPIOA1IntHandler (void);
//STATIC void GPIOA2IntHandler (void);
//STATIC void GPIOA3IntHandler (void);
STATIC void EXTI_Handler(uint port);

/******************************************************************************
DEFINE CONSTANTS
******************************************************************************/
#define PYBPIN_NUM_WAKE_PINS            (6)
#define PYBPIN_WAKES_NOT                (-1)

#define GPIO_DIR_MODE_ALT               0x00000002  // Pin is NOT controlled by the PGIO module
#define GPIO_DIR_MODE_ALT_OD            0x00000003  // Pin is NOT controlled by the PGIO module and is in open drain mode

#define PYB_PIN_FALLING_EDGE            0x01
#define PYB_PIN_RISING_EDGE             0x02
#define PYB_PIN_LOW_LEVEL               0x04
#define PYB_PIN_HIGH_LEVEL              0x08

/******************************************************************************
DEFINE TYPES
******************************************************************************/
typedef struct {
    bool       active;
    int8_t     lpds;
    int8_t     hib;
} pybpin_wake_pin_t;

/******************************************************************************
DECLARE PRIVATE DATA
******************************************************************************/
STATIC const mp_irq_methods_t pin_irq_methods;
//STATIC pybpin_wake_pin_t pybpin_wake_pin[PYBPIN_NUM_WAKE_PINS] =
//                                    { {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT},
//                                      {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT},
//                                      {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT},
//                                      {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT},
//                                      {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT},
//                                      {.active = false, .lpds = PYBPIN_WAKES_NOT, .hib = PYBPIN_WAKES_NOT} } ;

/******************************************************************************
 DEFINE PUBLIC FUNCTIONS
 ******************************************************************************/
//#define DEBUG
void pin_init0(void) {
// this initalization also reconfigures the JTAG/SWD pins
#ifndef DEBUG
    // assign all pins to the GPIO module so that peripherals can be connected to any
    // pins without conflicts after a soft reset
    mp_map_t *named_map = mp_obj_dict_get_map((mp_obj_t)&pin_board_pins_locals_dict);
    for (uint i = 0; i < named_map->used - 1; i++) {
        pin_obj_t * pin = (pin_obj_t *)named_map->table[i].value;
        pin_deassign (pin);
    }
#endif
}

// C API used to convert a user-supplied pin name into an ordinal pin number.
pin_obj_t *pin_find(mp_obj_t user_obj) {
    pin_obj_t *pin_obj;

    // if a pin was provided, use it
    if (MP_OBJ_IS_TYPE(user_obj, &pin_type)) {
        pin_obj = user_obj;
        return pin_obj;
    }

    // otherwise see if the pin name matches a cpu pin
    pin_obj = pin_find_named_pin(&pin_board_pins_locals_dict, user_obj);
    if (pin_obj) {
        return pin_obj;
    }

    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}

void pin_config (pin_obj_t *self, int af, uint mode, uint type, int value, uint strength) {
    self->mode = mode, self->type = type, self->strength = strength;
    // if af is -1, then we want to keep it as it is
    if (af != -1) {
        self->af = af;
    }

    // if value is -1, then we want to keep it as it is
    if (value != -1) {
        self->value = value;
    }

    // mark the pin as used
    self->used = true;
    pin_obj_configure ((const pin_obj_t *)self);

    // register it with the sleep module
    //pyb_sleep_add ((const mp_obj_t)self, (WakeUpCB_t)pin_obj_configure);
}

//void pin_assign_pins_af (mp_obj_t *pins, uint32_t n_pins, uint32_t type, uint32_t fn, uint32_t unit) {
//    for (int i = 0; i < n_pins; i++) {
//        pin_free_af_from_pins(fn, unit, i);
//        if (pins[i] != mp_const_none) {
//            pin_obj_t *pin = pin_find(pins[i]);
//            pin_config (pin, pin_find_af_index(pin, fn, unit, i), 0, type, -1, PIN_STRENGTH_2MA);
//        }
//    }
//}

uint8_t pin_find_peripheral_unit (const mp_obj_t pin, uint8_t fn, uint8_t type) {
    pin_obj_t *pin_o = pin_find(pin);
    for (int i = 0; i < pin_o->num_afs; i++) {
        if (pin_o->af_list[i].fn == fn && pin_o->af_list[i].type == type) {
            return pin_o->af_list[i].unit;
        }
    }
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}

uint8_t pin_find_peripheral_type (const mp_obj_t pin, uint8_t fn, uint8_t unit) {
    pin_obj_t *pin_o = pin_find(pin);
    for (int i = 0; i < pin_o->num_afs; i++) {
        if (pin_o->af_list[i].fn == fn && pin_o->af_list[i].unit == unit) {
            return pin_o->af_list[i].type;
        }
    }
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}

int8_t pin_find_af_index (const pin_obj_t* pin, uint8_t fn, uint8_t unit, uint8_t type) {
    int8_t af = pin_obj_find_af(pin, fn, unit, type);
    if (af < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
    }
    return af;
}

/******************************************************************************
DEFINE PRIVATE FUNCTIONS
 ******************************************************************************/
STATIC pin_obj_t *pin_find_named_pin(const mp_obj_dict_t *named_pins, mp_obj_t name) {
    mp_map_t *named_map = mp_obj_dict_get_map((mp_obj_t)named_pins);
    mp_map_elem_t *named_elem = mp_map_lookup(named_map, name, MP_MAP_LOOKUP);
    if (named_elem != NULL && named_elem->value != NULL) {
        return named_elem->value;
    }
    return NULL;
}

STATIC pin_obj_t *pin_find_pin_by_port_bit (const mp_obj_dict_t *named_pins, uint port, uint bit) {
    mp_map_t *named_map = mp_obj_dict_get_map((mp_obj_t)named_pins);
    for (uint i = 0; i < named_map->used; i++) {
        if ((((pin_obj_t *)named_map->table[i].value)->port == port) &&
                (((pin_obj_t *)named_map->table[i].value)->bit == bit)) {
            return named_map->table[i].value;
        }
    }
    return NULL;
}

STATIC int8_t pin_obj_find_af (const pin_obj_t* pin, uint8_t fn, uint8_t unit, uint8_t type) {
    for (int i = 0; i < pin->num_afs; i++) {
        if (pin->af_list[i].fn == fn && pin->af_list[i].unit == unit && pin->af_list[i].type == type) {
            return pin->af_list[i].idx;
        }
    }
    return -1;
}

//STATIC void pin_free_af_from_pins (uint8_t fn, uint8_t unit, uint8_t type) {
//    mp_map_t *named_map = mp_obj_dict_get_map((mp_obj_t)&pin_board_pins_locals_dict);
//    for (uint i = 0; i < named_map->used - 1; i++) {
//        pin_obj_t * pin = (pin_obj_t *)named_map->table[i].value;
//        // af is different than GPIO
//        if (pin->af > PIN_MODE_0) {
//            // check if the pin supports the target af
//            int af = pin_obj_find_af(pin, fn, unit, type);
//            if (af > 0 && af == pin->af) {
//                // the pin supports the target af, de-assign it
//                pin_deassign (pin);
//            }
//        }
//    }
//}
//
STATIC void pin_deassign (pin_obj_t* pin) {
    pin_config (pin, 0, GPIO_DIR_MODE_IN, GPIO_PIN_TYPE_STD, -1, GPIO_STRENGTH_2MA);
    pin->used = false;
}

STATIC void pin_obj_configure (const pin_obj_t *self) {


    switch (self->port) {
    case PORT_A:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        break;
    case PORT_B:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        break;
    case PORT_C:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        break;
    case PORT_D:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        break;
    case PORT_E:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        break;
    case PORT_F:
        //MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        break;
    default:
        break;
    }

    // configure the direction for Input or Output mode
    // if mode = GPIO_DIR_MODE_HW this function set the AFSEL bit
    MAP_GPIODirModeSet(self->port, self->bit, self->mode);


    // Configure the ouput pad
    MAP_GPIOPadConfigSet(self->port, self->bit, self->strength, self->type);


    // set the pin value if the port is set as output
    if (self->mode == GPIO_DIR_MODE_OUT) {
        MAP_GPIOPinWrite(self->port, self->bit, self->value ? self->bit : 0);
    }
}

//STATIC void pin_get_hibernate_pin_and_idx (const pin_obj_t *self, uint *hib_pin, uint *idx) {
//    // pin_num is actually : (package_pin - 1)
//    switch (self->pin_num) {
//    case 56:    // GP2
//        *hib_pin = PRCM_HIB_GPIO2;
//        *idx = 0;
//        break;
//    case 58:    // GP4
//        *hib_pin = PRCM_HIB_GPIO4;
//        *idx = 1;
//        break;
//    case 3:     // GP13
//        *hib_pin = PRCM_HIB_GPIO13;
//        *idx = 2;
//        break;
//    case 7:     // GP17
//        *hib_pin = PRCM_HIB_GPIO17;
//        *idx = 3;
//        break;
//    case 1:     // GP11
//        *hib_pin = PRCM_HIB_GPIO11;
//        *idx = 4;
//        break;
//    case 16:    // GP24
//        *hib_pin = PRCM_HIB_GPIO24;
//        *idx = 5;
//        break;
//    default:
//        *idx = 0xFF;
//        break;
//    }
//}

STATIC void pin_irq_enable (mp_obj_t self_in) {
    const pin_obj_t *self = self_in;
//    uint hib_pin, idx;
//
//    pin_get_hibernate_pin_and_idx (self, &hib_pin, &idx);
//    if (idx < PYBPIN_NUM_WAKE_PINS) {
//        if (pybpin_wake_pin[idx].lpds != PYBPIN_WAKES_NOT) {
//            // enable GPIO as a wake source during LPDS
//            MAP_PRCMLPDSWakeUpGPIOSelect(idx, pybpin_wake_pin[idx].lpds);
//            MAP_PRCMLPDSWakeupSourceEnable(PRCM_LPDS_GPIO);
//        }
//
//        if (pybpin_wake_pin[idx].hib != PYBPIN_WAKES_NOT) {
//            // enable GPIO as a wake source during hibernate
//            MAP_PRCMHibernateWakeUpGPIOSelect(hib_pin, pybpin_wake_pin[idx].hib);
//            MAP_PRCMHibernateWakeupSourceEnable(hib_pin);
//        }
//        else {
//            MAP_PRCMHibernateWakeupSourceDisable(hib_pin);
//        }
//    }
//    // if idx is invalid, the pin supports active interrupts for sure
//    if (idx >= PYBPIN_NUM_WAKE_PINS || pybpin_wake_pin[idx].active) {
        MAP_GPIOIntClear(self->port, self->bit);
        MAP_GPIOIntEnable(self->port, self->bit);
//    }
//    // in case it was enabled before
//    else if (idx < PYBPIN_NUM_WAKE_PINS && !pybpin_wake_pin[idx].active) {
//        MAP_GPIOIntDisable(self->port, self->bit);
//    }
}

STATIC void pin_irq_disable (mp_obj_t self_in) {
    const pin_obj_t *self = self_in;
//    uint hib_pin, idx;
//
//    pin_get_hibernate_pin_and_idx (self, &hib_pin, &idx);
//    if (idx < PYBPIN_NUM_WAKE_PINS) {
//        if (pybpin_wake_pin[idx].lpds != PYBPIN_WAKES_NOT) {
//            // disable GPIO as a wake source during LPDS
//            MAP_PRCMLPDSWakeupSourceDisable(PRCM_LPDS_GPIO);
//        }
//        if (pybpin_wake_pin[idx].hib != PYBPIN_WAKES_NOT) {
//            // disable GPIO as a wake source during hibernate
//            MAP_PRCMHibernateWakeupSourceDisable(hib_pin);
//        }
//    }
    // not need to check for the active flag, it's safe to disable it anyway
    MAP_GPIOIntDisable(self->port, self->bit);
}

STATIC int pin_irq_flags (mp_obj_t self_in) {
    const pin_obj_t *self = self_in;
    return self->irq_flags;
}

STATIC void pin_extint_register(pin_obj_t *self, uint32_t intmode, uint32_t priority) {
    //void *handler;
    uint32_t intnum;

    // configure the interrupt type
    MAP_GPIOIntTypeSet(self->port, self->bit, intmode);
    switch (self->port) {
    case GPIO_PORTA_BASE:
        intnum = INT_GPIOA_TM4C123;
        break;
    case GPIO_PORTB_BASE:
        intnum = INT_GPIOB_TM4C123;
        break;
    case GPIO_PORTC_BASE:
        intnum = INT_GPIOC_TM4C123;
        break;
    case GPIO_PORTD_BASE:
        intnum = INT_GPIOD_TM4C123;
    case GPIO_PORTE_BASE:
        intnum = INT_GPIOE_TM4C123;
    case GPIO_PORTF_BASE:
    default:
        intnum = INT_GPIOF_TM4C123;
        break;
    }
    // set the interrupt to the lowest priority, to make sure that
    // no other ISRs will be preemted by this one
    MAP_IntPrioritySet(intnum, priority);
}

STATIC void pin_validate_mode (uint mode) {
    if (mode != GPIO_DIR_MODE_IN && mode != GPIO_DIR_MODE_OUT && 
        mode != GPIO_DIR_MODE_HW /*&& mode != GPIO_DIR_MODE_ALT_OD */) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
    }
}
STATIC void pin_validate_type (uint type) {
    if (type != GPIO_PIN_TYPE_STD && type != GPIO_PIN_TYPE_STD_WPU && 
        type != GPIO_PIN_TYPE_STD_WPD && type != GPIO_PIN_TYPE_ANALOG && type != GPIO_PIN_TYPE_OD) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
    }
}

STATIC void pin_validate_drive(uint strength) {
    if (strength != GPIO_STRENGTH_2MA && strength != GPIO_STRENGTH_4MA && strength != GPIO_STRENGTH_8MA) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
    }
}

//STATIC void pin_validate_af(const pin_obj_t* pin, int8_t idx, uint8_t *fn, uint8_t *unit, uint8_t *type) {
//    for (int i = 0; i < pin->num_afs; i++) {
//        if (pin->af_list[i].idx == idx) {
//            *fn = pin->af_list[i].fn;
//            *unit = pin->af_list[i].unit;
//            *type = pin->af_list[i].type;
//            return;
//        }
//    }
//    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
//}
//
STATIC uint8_t pin_get_value (const pin_obj_t* self) {
    uint32_t value;
    //bool setdir = false;
    //if (self->mode == PIN_TYPE_OD || self->mode == GPIO_DIR_MODE_ALT_OD) {
    //    setdir = true;
    //    // configure the direction to IN for a moment in order to read the pin value
    //    MAP_GPIODirModeSet(self->port, self->bit, GPIO_DIR_MODE_IN);
    //}
    // now get the value
    value = MAP_GPIOPinRead(self->port, self->bit);
    //if (setdir) {
    //    // set the direction back to output
    //    MAP_GPIODirModeSet(self->port, self->bit, GPIO_DIR_MODE_OUT);
    //    if (self->value) {
    //        MAP_GPIOPinWrite(self->port, self->bit, self->bit);
    //    } else {
    //        MAP_GPIOPinWrite(self->port, self->bit, 0);
    //    }
    //}
    // return it
    return value ? 1 : 0;
}

void GPIOPortA_Handler (void) {
    EXTI_Handler(GPIO_PORTA_BASE);
}

void GPIOPortB_Handler (void) {
    EXTI_Handler(GPIO_PORTB_BASE);
}

void GPIOPortC_Handler (void) {
    EXTI_Handler(GPIO_PORTC_BASE);
}

void GPIOPortD_Handler (void) {
    EXTI_Handler(GPIO_PORTD_BASE);
}

void GPIOPortE_Handler (void) {
    EXTI_Handler(GPIO_PORTE_BASE);
}

void GPIOPortF_Handler (void) {
    EXTI_Handler(GPIO_PORTF_BASE);
}




 

// common interrupt handler
STATIC void EXTI_Handler(uint port) {
    uint32_t bits = MAP_GPIOIntStatus(port, true);
    MAP_GPIOIntClear(port, bits);

    // might be that we have more than one pin interrupt pending
    // therefore we must loop through all of the 8 possible bits
    for (int i = 0; i < 8; i++) {
        uint32_t bit = (1 << i);
        if (bit & bits) {
            pin_obj_t *self = (pin_obj_t *)pin_find_pin_by_port_bit(&pin_board_pins_locals_dict, port, bit);
            if (self->irq_trigger == (PYB_PIN_FALLING_EDGE | PYB_PIN_RISING_EDGE)) {
                // read the pin value (hoping that the pin level has remained stable)
                self->irq_flags = MAP_GPIOPinRead(self->port, self->bit) ? PYB_PIN_RISING_EDGE : PYB_PIN_FALLING_EDGE;
            } else {
                // same as the triggers
                self->irq_flags = self->irq_trigger;
            }
            mp_irq_handler(mp_irq_find(self));
            // always clear the flags after leaving the user handler
            self->irq_flags = 0;
        }
    }
}


/******************************************************************************/
// Micro Python bindings

STATIC const mp_arg_t pin_init_args[] = {
    { MP_QSTR_mode,                        MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_type,                        MP_ARG_OBJ, {.u_obj = mp_const_none} },
    { MP_QSTR_value,    MP_ARG_KW_ONLY  |  MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_drive,    MP_ARG_KW_ONLY  |  MP_ARG_INT, {.u_int = GPIO_STRENGTH_2MA} },
    { MP_QSTR_alt,      MP_ARG_KW_ONLY  |  MP_ARG_INT, {.u_int = -1} },
};
#define pin_INIT_NUM_ARGS MP_ARRAY_SIZE(pin_init_args)

STATIC mp_obj_t pin_obj_init_helper(pin_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // parse args
    mp_arg_val_t args[pin_INIT_NUM_ARGS];
    mp_arg_parse_all(n_args, pos_args, kw_args, pin_INIT_NUM_ARGS, pin_init_args, args);

    // get the io mode
    uint mode;
    //  default is input
    if (args[0].u_obj == MP_OBJ_NULL) {
        mode = GPIO_DIR_MODE_IN;
    } else {
        mode = mp_obj_get_int(args[0].u_obj);
        pin_validate_mode (mode);
    }

    // get the type type
    uint type;
    if (args[1].u_obj == mp_const_none) {
        type = GPIO_PIN_TYPE_STD;
    } else {
        type = mp_obj_get_int(args[1].u_obj);
        pin_validate_type (type);
    }

    // get the value
    int value = -1;
    if (args[2].u_obj != MP_OBJ_NULL) {
        if (mp_obj_is_true(args[2].u_obj)) {
            value = 1;
        } else {
            value = 0;
        }
    }

    // get the strenght
    uint strength = args[3].u_int;
    pin_validate_drive(strength);

    // get the alternate function
    int af = args[4].u_int;
    if (mode != GPIO_DIR_MODE_HW /*&& mode != GPIO_DIR_MODE_ALT_OD */) {
        if (af == -1) {
            af = 0;
        } else {
            goto invalid_args;
        }
    } else if (af < -1 || af > 15) {
        goto invalid_args;
    }
//
//    // check for a valid af and then free it from any other pins
//    if (af > PIN_MODE_0) {
//        uint8_t fn, unit, type;
//        pin_validate_af (self, af, &fn, &unit, &type);
//        pin_free_af_from_pins(fn, unit, type);
//    }
    pin_config (self, af, mode, type, value, strength);
//
    return mp_const_none;
//
invalid_args:
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}

STATIC void pin_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pin_obj_t *self = self_in;
    uint32_t type = self->type;
    uint32_t drive = self->strength;

    // pin name
    mp_printf(print, "Pin('%q'", self->name);

    // pin mode
    qstr mode_qst;
    uint32_t mode = self->mode;
    if (mode == GPIO_DIR_MODE_IN) {
        mode_qst = MP_QSTR_IN;
    } else if (mode == GPIO_DIR_MODE_OUT) {
        mode_qst = MP_QSTR_OUT;
    } else if (mode == GPIO_DIR_MODE_ALT) {
        mode_qst = MP_QSTR_ALT;
//    } else if (mode == GPIO_DIR_MODE_ALT_OD) {
//        mode_qst = MP_QSTR_ALT_OPEN_DRAIN;
    } else {
        mode_qst = MP_QSTR_OPEN_DRAIN;
    }
    mp_printf(print, ", mode=Pin.%q", mode_qst);

    // pin type
    qstr type_qst;
    switch (type) {
      case GPIO_PIN_TYPE_STD:
        type_qst = MP_QSTR_STD;
        break;
      case GPIO_PIN_TYPE_STD_WPU:
          type_qst = MP_QSTR_PULL_UP;
          break;
      default:
          type_qst = MP_QSTR_PULL_DOWN;
    }
    mp_printf(print, ", type=Pin.%q", type_qst);

    //if (type == GPIO_PIN_TYPE_STD) {
    //    mp_printf(print, ", type=%q", MP_QSTR_None);
    //} else {
    //    if (type == GPIO_PIN_TYPE_STD_WPU) {
    //        type_qst = MP_QSTR_PULL_UP;
    //    } else {
    //        type_qst = MP_QSTR_PULL_DOWN;
    //    }
    //    mp_printf(print, ", type=Pin.%q", type_qst);
    //}

    // pin drive
    qstr drv_qst;
    if (drive == GPIO_STRENGTH_2MA) {
        drv_qst = MP_QSTR_LOW_POWER;
    } else if (drive == GPIO_STRENGTH_4MA) {
        drv_qst = MP_QSTR_MED_POWER;
    } else {
        drv_qst = MP_QSTR_HIGH_POWER;
    }
    mp_printf(print, ", drive=Pin.%q", drv_qst);

    // pin af
    int alt = (self->af == 0) ? -1 : self->af;
    mp_printf(print, ", alt=%d)", alt);
}

STATIC mp_obj_t pin_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // Run an argument through the mapper and return the result.
    pin_obj_t *pin = (pin_obj_t *)pin_find(args[0]);

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    pin_obj_init_helper(pin, n_args - 1, args + 1, &kw_args);

    return (mp_obj_t)pin;
}

STATIC mp_obj_t pin_obj_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pin_obj_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
MP_DEFINE_CONST_FUN_OBJ_KW(pin_init_obj, 1, pin_obj_init);

STATIC mp_obj_t pin_value(mp_uint_t n_args, const mp_obj_t *args) {
    pin_obj_t *self = args[0];
    if (n_args == 1) {
        // get the value
        return MP_OBJ_NEW_SMALL_INT(pin_get_value(self));
    } else {
        // set the pin value
        if (mp_obj_is_true(args[1])) {
            self->value = 1;
            MAP_GPIOPinWrite(self->port, self->bit, self->bit);
        } else {
            self->value = 0;
            MAP_GPIOPinWrite(self->port, self->bit, 0);
        }
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_value_obj, 1, 2, pin_value);

STATIC mp_obj_t pin_toggle(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    MAP_GPIOPinWrite(self->port, self->bit, ~MAP_GPIOPinRead(self->port, self->bit));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_toggle_obj, pin_toggle);

STATIC mp_obj_t pin_id(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    return MP_OBJ_NEW_QSTR(self->name);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_id_obj, pin_id);

STATIC mp_obj_t pin_mode(mp_uint_t n_args, const mp_obj_t *args) {
    pin_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->mode);
    } else {
        uint32_t mode = mp_obj_get_int(args[1]);
        pin_validate_mode (mode);
        self->mode = mode;
        pin_obj_configure(self);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_mode_obj, 1, 2, pin_mode);

STATIC mp_obj_t pin_type_fun(mp_uint_t n_args, const mp_obj_t *args) {
    pin_obj_t *self = args[0];
    if (n_args == 1) {
        if (self->type == GPIO_PIN_TYPE_STD) {
            return mp_const_none;
        }
        return mp_obj_new_int(self->type);
    } else {
        uint32_t type;
        if (args[1] == mp_const_none) {
            type = GPIO_PIN_TYPE_STD;
        } else {
            type = mp_obj_get_int(args[1]);
            pin_validate_type (type);
        }
        self->type = type;
        pin_obj_configure(self);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_type_obj, 1, 2, pin_type_fun);

STATIC mp_obj_t pin_drive(mp_uint_t n_args, const mp_obj_t *args) {
    pin_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->strength);
    } else {
        uint32_t strength = mp_obj_get_int(args[1]);
        pin_validate_drive (strength);
        self->strength = strength;
        pin_obj_configure(self);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_drive_obj, 1, 2, pin_drive);

STATIC mp_obj_t pin_call(mp_obj_t self_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_obj_t _args[2] = {self_in, *args};
    return pin_value (n_args + 1, _args);
}

STATIC mp_obj_t pin_alt_list(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    mp_obj_t af[2];
    mp_obj_t afs = mp_obj_new_list(0, NULL);

    for (int i = 0; i < self->num_afs; i++) {
        af[0] = MP_OBJ_NEW_QSTR(self->af_list[i].name);
        af[1] = mp_obj_new_int(self->af_list[i].idx);
        mp_obj_list_append(afs, mp_obj_new_tuple(MP_ARRAY_SIZE(af), af));
    }
    return afs;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_alt_list_obj, pin_alt_list);

// \method irq(trigger, priority, handler, wake)
STATIC mp_obj_t pin_irq (mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    mp_arg_val_t args[mp_irq_INIT_NUM_ARGS];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, mp_irq_INIT_NUM_ARGS, mp_irq_init_args, args);
    pin_obj_t *self = pos_args[0];

    // convert the priority to the correct value
    uint priority = mp_irq_translate_priority (args[1].u_int);

    // verify and translate the interrupt mode
    uint mp_trigger = mp_obj_get_int(args[0].u_obj);
    uint trigger;
    if (mp_trigger == (PYB_PIN_FALLING_EDGE | PYB_PIN_RISING_EDGE)) {
        trigger = GPIO_BOTH_EDGES;
    } else {
        switch (mp_trigger) {
        case PYB_PIN_FALLING_EDGE:
            trigger = GPIO_FALLING_EDGE;
            break;
        case PYB_PIN_RISING_EDGE:
            trigger = GPIO_RISING_EDGE;
            break;
        case PYB_PIN_LOW_LEVEL:
            trigger = GPIO_LOW_LEVEL;
            break;
        case PYB_PIN_HIGH_LEVEL:
            trigger = GPIO_HIGH_LEVEL;
            break;
        default:
            goto invalid_args;
        }
    }

//    uint8_t pwrmode = (args[3].u_obj == mp_const_none) ? PYB_PWR_MODE_ACTIVE : mp_obj_get_int(args[3].u_obj);
//    if (pwrmode > (PYB_PWR_MODE_ACTIVE | PYB_PWR_MODE_LPDS | PYB_PWR_MODE_HIBERNATE)) {
//        goto invalid_args;
//    }
//
//    // get the wake info from this pin
//    uint hib_pin, idx;
//    pin_get_hibernate_pin_and_idx ((const pin_obj_t *)self, &hib_pin, &idx);
//    if (pwrmode & PYB_PWR_MODE_LPDS) {
//        if (idx >= PYBPIN_NUM_WAKE_PINS) {
//            goto invalid_args;
//        }
//        // wake modes are different in LDPS
//        uint wake_mode;
//        switch (trigger) {
//        case GPIO_FALLING_EDGE:
//            wake_mode = PRCM_LPDS_FALL_EDGE;
//            break;
//        case GPIO_RISING_EDGE:
//            wake_mode = PRCM_LPDS_RISE_EDGE;
//            break;
//        case GPIO_LOW_LEVEL:
//            wake_mode = PRCM_LPDS_LOW_LEVEL;
//            break;
//        case GPIO_HIGH_LEVEL:
//            wake_mode = PRCM_LPDS_HIGH_LEVEL;
//            break;
//        default:
//            goto invalid_args;
//            break;
//        }
//
//        // first clear the lpds value from all wake-able pins
//        for (uint i = 0; i < PYBPIN_NUM_WAKE_PINS; i++) {
//            pybpin_wake_pin[i].lpds = PYBPIN_WAKES_NOT;
//        }
//
//        // enable this pin as a wake-up source during LPDS
//        pybpin_wake_pin[idx].lpds = wake_mode;
//    } else if (idx < PYBPIN_NUM_WAKE_PINS) {
//        // this pin was the previous LPDS wake source, so disable it completely
//        if (pybpin_wake_pin[idx].lpds != PYBPIN_WAKES_NOT) {
//            MAP_PRCMLPDSWakeupSourceDisable(PRCM_LPDS_GPIO);
//        }
//        pybpin_wake_pin[idx].lpds = PYBPIN_WAKES_NOT;
//    }
//
//    if (pwrmode & PYB_PWR_MODE_HIBERNATE) {
//        if (idx >= PYBPIN_NUM_WAKE_PINS) {
//            goto invalid_args;
//        }
//        // wake modes are different in hibernate
//        uint wake_mode;
//        switch (trigger) {
//        case GPIO_FALLING_EDGE:
//            wake_mode = PRCM_HIB_FALL_EDGE;
//            break;
//        case GPIO_RISING_EDGE:
//            wake_mode = PRCM_HIB_RISE_EDGE;
//            break;
//        case GPIO_LOW_LEVEL:
//            wake_mode = PRCM_HIB_LOW_LEVEL;
//            break;
//        case GPIO_HIGH_LEVEL:
//            wake_mode = PRCM_HIB_HIGH_LEVEL;
//            break;
//        default:
//            goto invalid_args;
//            break;
//        }
//
//        // enable this pin as wake-up source during hibernate
//        pybpin_wake_pin[idx].hib = wake_mode;
//    } else if (idx < PYBPIN_NUM_WAKE_PINS) {
//        pybpin_wake_pin[idx].hib = PYBPIN_WAKES_NOT;
//    }
//
    // we need to update the callback atomically, so we disable the
    // interrupt before we update anything.
    pin_irq_disable(self);
//    if (pwrmode & PYB_PWR_MODE_ACTIVE) {
        // register the interrupt
        pin_extint_register((pin_obj_t *)self, trigger, priority);
//        if (idx < PYBPIN_NUM_WAKE_PINS) {
//            pybpin_wake_pin[idx].active = true;
//        }
//    } else if (idx < PYBPIN_NUM_WAKE_PINS) {
//        pybpin_wake_pin[idx].active = false;
//    }
//
    // all checks have passed, we can create the irq object
    mp_obj_t _irq = mp_irq_new (self, args[2].u_obj, &pin_irq_methods);
//    if (pwrmode & PYB_PWR_MODE_LPDS) {
//        pyb_sleep_set_gpio_lpds_callback (_irq);
//    }
//
    // save the mp_trigge for later
    self->irq_trigger = mp_trigger;

    // enable the interrupt just before leaving
    pin_irq_enable(self);

    return _irq;

invalid_args:
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pin_irq_obj, 1, pin_irq);

STATIC const mp_map_elem_t pin_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_init),                    (mp_obj_t)&pin_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_value),                   (mp_obj_t)&pin_value_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_toggle),                  (mp_obj_t)&pin_toggle_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_id),                      (mp_obj_t)&pin_id_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mode),                    (mp_obj_t)&pin_mode_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_type),                    (mp_obj_t)&pin_type_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_drive),                   (mp_obj_t)&pin_drive_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_alt_list),                (mp_obj_t)&pin_alt_list_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_irq),                     (mp_obj_t)&pin_irq_obj },

    // class attributes
    { MP_OBJ_NEW_QSTR(MP_QSTR_board),                   (mp_obj_t)&pin_board_pins_obj_type },

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_IN),                      MP_OBJ_NEW_SMALL_INT(GPIO_DIR_MODE_IN) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_OUT),                     MP_OBJ_NEW_SMALL_INT(GPIO_DIR_MODE_OUT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ALT),                     MP_OBJ_NEW_SMALL_INT(GPIO_DIR_MODE_HW) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_STD),                     MP_OBJ_NEW_SMALL_INT(GPIO_PIN_TYPE_STD) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PULL_UP),                 MP_OBJ_NEW_SMALL_INT(GPIO_PIN_TYPE_STD_WPU) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PULL_DOWN),               MP_OBJ_NEW_SMALL_INT(GPIO_PIN_TYPE_STD_WPD) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_OPEN_DRAIN),              MP_OBJ_NEW_SMALL_INT(GPIO_PIN_TYPE_OD) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ANALOG),                  MP_OBJ_NEW_SMALL_INT(GPIO_PIN_TYPE_ANALOG) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_LOW_POWER),               MP_OBJ_NEW_SMALL_INT(GPIO_STRENGTH_2MA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MED_POWER),               MP_OBJ_NEW_SMALL_INT(GPIO_STRENGTH_4MA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_HIGH_POWER),              MP_OBJ_NEW_SMALL_INT(GPIO_STRENGTH_8MA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_IRQ_FALLING),             MP_OBJ_NEW_SMALL_INT(PYB_PIN_FALLING_EDGE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_IRQ_RISING),              MP_OBJ_NEW_SMALL_INT(PYB_PIN_RISING_EDGE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_IRQ_LOW_LEVEL),           MP_OBJ_NEW_SMALL_INT(PYB_PIN_LOW_LEVEL) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_IRQ_HIGH_LEVEL),          MP_OBJ_NEW_SMALL_INT(PYB_PIN_HIGH_LEVEL) },
};

STATIC MP_DEFINE_CONST_DICT(pin_locals_dict, pin_locals_dict_table);

const mp_obj_type_t pin_type = {
    { &mp_type_type },
    .name = MP_QSTR_Pin,
    .print = pin_print,
    .make_new = pin_make_new,
    .call = pin_call,
    .locals_dict = (mp_obj_t)&pin_locals_dict,
};

STATIC const mp_irq_methods_t pin_irq_methods = {
    .init = pin_irq,
    .enable = pin_irq_enable,
    .disable = pin_irq_disable,
    .flags = pin_irq_flags,
};

STATIC void pin_named_pins_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pin_named_pins_obj_t *self = self_in;
    mp_printf(print, "<Pin.%q>", self->name);
}

const mp_obj_type_t pin_board_pins_obj_type = {
    { &mp_type_type },
    .name = MP_QSTR_board,
    .print = pin_named_pins_obj_print,
    .locals_dict = (mp_obj_t)&pin_board_pins_locals_dict,
};

