/*
 * Copyright (c) 2020-2026 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include <switch.h>

namespace ams::controller {

    /*
     * What the Bluetooth LE host needs from a controller beyond being a SwitchController: the payloads of its
     * HID input Report notifications (report id stripped, as HOGP delivers them) and a few questions about them.
     */
    class BleHidController {

        public:
            virtual ~BleHidController() { }

            // Payload of a notification from the HID input Report characteristic
            virtual Result HandleInputReport(const u8 *data, size_t size) = 0;

            // The pad only reports when something is touched; its resting state is delivered on its behalf
            virtual Result HandleNeutralReport() = 0;

            // L and R shoulder buttons both held (what brings a controller back on the Controllers screen)
            virtual bool ReportHasShoulderButtons(const u8 *data, size_t size) const = 0;

            virtual bool ReportHasAnyInput(const u8 *data, size_t size) const = 0;

            // Rumble the system started is not stopped by it once the controller has been withdrawn
            virtual void StopRumble() { }
    };

}
