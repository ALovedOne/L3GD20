/*
 * Copyright (C) 2014 Mike Franklin
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 *  L3GD20 Accelerometer
 */
#define L3GD20_ADDRESS 0x6b
#define L3GD20_SIG 0b11010100
#define L3GD20_REG_SIG  0xf

#define L3GD20_I_PIN     17

extern struct i2c_driver frz_L3GD20_driver;
