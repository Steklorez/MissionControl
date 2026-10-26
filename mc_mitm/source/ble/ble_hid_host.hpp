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
#include <stratosphere.hpp>

namespace ams::ble {

    /*
     * HID-over-GATT host for Bluetooth LE controllers (Xbox controllers on firmware 5.x and newer).
     *
     * btm leaves BLE disabled on a retail console and never connects to LE devices, so this module acts as the
     * GATT client itself: it brings BLE up, finds the controller (a remembered one by connecting to it directly,
     * a new one by scanning for a gamepad in pairing mode), pairs when the link needs it, subscribes to the HID
     * input Report characteristic and presents the device to hid as a Pro Controller. Input reports arrive on
     * btdrv's LE HID event stream and are handed to the controller driver.
     */

    // True when the host runs. hid only learns of a controller btm doesn't manage through the device condition
    // list of 13.0.0 and later, so the rest of the mitm must make way for the host on those versions only.
    bool IsEnabled();

    void Launch();
    void WaitFinished();

    // Called from the power module thread when the console starts going to sleep, while the bluetooth module
    // is still awake: drops the controller's link so nothing is left on the air to wake the console, then marks
    // the radio unavailable to the host.
    void PrepareForSleep();
    // btm switching the radio off or on (Bluetooth in the settings, flight mode)
    void OnBluetoothRadio(bool on);

    // Called from the btdrv mitm when btm starts or stops searching for controllers, which it does while the
    // Controllers screen is open. A connected controller is then withdrawn from hid, as the screen does with
    // every other controller, and comes back when LB+RB are pressed (any button once the screen is closed).
    void OnPairingWindow(bool open);

}
