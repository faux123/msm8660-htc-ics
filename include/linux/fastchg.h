/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#ifndef _LINUX_FASTCHG_H
#define _LINUX_FASTCHG_H

#define FAST_CHARGE_DISABLED 0
#define FAST_CHARGE_FORCE_AC 1
#define FAST_CHARGE_FORCE_AC_IF_NO_USB 2

#define USB_ACC_NOT_DETECTED 0
#define USB_ACC_DETECTED 1

extern int force_fast_charge;
extern int USB_peripheral_detected;

#endif
