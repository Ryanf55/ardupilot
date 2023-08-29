/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
///
/// @file   AP_Checksum.h
/// @brief	Common checksum calculations
///

#include <type_traits>
#include <stdint.h>
#include "AP_Common.h"

namespace AP {

// https://en.wikipedia.org/wiki/Fletcher%27s_checksum
uint16_t Fletcher16(const uint8_t *data, const size_t count) WARN_IF_UNUSED;

} // namespace AP
