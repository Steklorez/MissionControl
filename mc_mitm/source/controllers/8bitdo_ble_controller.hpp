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
#include "emulated_switch_controller.hpp"
#include "ble_hid_controller.hpp"

namespace ams::controller {

    struct EightBitDoBleButtonData {
        u8 A      : 1;
        u8 B      : 1;
        u8 C      : 1;
        u8 X      : 1;
        u8 Y      : 1;
        u8 Z      : 1;
        u8 L1     : 1;
        u8 R1     : 1;

        u8 L2     : 1;
        u8 R2     : 1;
        u8 select : 1;
        u8 start  : 1;
        u8 home   : 1;
        u8 L3     : 1;
        u8 R3     : 1;
        u8        : 1;

        u8 L4     : 1;
        u8 R4     : 1;
        u8        : 6;
    } PACKED;

    // Input report 0x01 as delivered over HID-over-GATT (report id stripped)
    struct EightBitDoBleInputReport0x01 {
        u8 dpad;
        AnalogStick<u8> left_stick;
        AnalogStick<u8> right_stick;
        u8 right_trigger;
        u8 left_trigger;
        EightBitDoBleButtonData buttons;
        u8 _unk0[3];
        u8 battery;
        // Accelerometer at 4096 per g (1 g on z with the pad lying face up), then the gyroscope
        s16 accel[3];
        s16 gyro[3];
        u8 _unk1[7];
    } PACKED;

    struct EightBitDoBleOutputReport0x05 {
        u8 magnitude[4];
    } PACKED;

    /*
     * 8BitDo controller connected over Bluetooth LE in its Android mode (Ultimate 2 Wireless). It presents a
     * generic HID-over-GATT gamepad: a hat switch, four 8-bit stick axes, two analog triggers and 24 buttons,
     * followed by vendor data (battery level and motion). Rumble is output report 0x05.
     */
    class EightBitDoBleController final : public EmulatedSwitchController, public BleHidController {

        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x2dc8, 0x6012}  // 8BitDo Ultimate 2 Wireless (Bluetooth)
            };

            // The leading items of the input report's descriptor: report id 0x01 opening with a hat switch
            static bool MatchesReportMap(const u8 *map, size_t size);

            EightBitDoBleController(bluetooth::Address address, u32 conn_id, const BtdrvGattId &service_id, bool service_primary, const BtdrvGattId &output_report_id)
            : EmulatedSwitchController(address, hardware_ids[0])
            , m_conn_id(conn_id)
            , m_service_id(service_id)
            , m_service_primary(service_primary)
            , m_output_report_id(output_report_id) { }

            Result HandleInputReport(const u8 *data, size_t size) override;
            Result HandleNeutralReport() override;
            bool ReportHasShoulderButtons(const u8 *data, size_t size) const override;
            bool ReportHasAnyInput(const u8 *data, size_t size) const override;
            void StopRumble() override;

        protected:
            void ProcessInputData(const bluetooth::HidReport *report) override;
            Result SetVibration(const SwitchMotorData *motor_data) override;
            Result CancelVibration() override;

        private:
            Result WriteRumble(u8 strong, u8 weak);
            void HandleVolumeButtons(bool down, bool up);
            void MapMotion(const EightBitDoBleInputReport0x01 *src);

            u32 m_conn_id;
            BtdrvGattId m_service_id;
            bool m_service_primary;
            BtdrvGattId m_output_report_id;

            u8 m_last_rumble_strong = 0;
            u8 m_last_rumble_weak = 0;

            bool m_volume_down = false;
            bool m_volume_up = false;

            float m_gyro_bias[3] = {};

            bluetooth::HidReportEventInfo m_event_info;
    };

}
