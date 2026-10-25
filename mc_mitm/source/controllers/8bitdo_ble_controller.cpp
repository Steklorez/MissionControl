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
#include "8bitdo_ble_controller.hpp"
#include "controller_utils.hpp"
#include "../ble/ble_log.hpp"
#include <stratosphere.hpp>

namespace ams::controller {

    namespace {

        // HOGP strips report ids from the data; the input report we decode is report 0x01
        constexpr u8 InputReportId = 0x01;

        constexpr u8 TriggerMax = UINT8_MAX;

        constexpr float AccelPerG = 4096.0f;
        // Measured against the console's own Joy-Con on the same quarter turns (with the packer's
        // GyroOutputScale applied); the usual +-2000 dps scale of 16.4 reads about 5% short
        constexpr float GyroPerDps = 15.5f;

        // Raw readings below this count as the pad lying still, for tracking the gyroscope's resting offset
        constexpr float GyroRestThreshold = 40.0f;
        constexpr float GyroBiasRate = 0.01f;

        constexpr u8 DpadReleased = 0x0f;

        // Usage Page (Generic Desktop), Usage (Game Pad), Collection (Application), Report ID (1), Usage (Hat switch)
        constexpr u8 ReportMapPrefix[] = { 0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x39 };

        // Hat released, sticks centred, triggers and buttons released
        constexpr u8 NeutralReport[offsetof(EightBitDoBleInputReport0x01, _unk0)] = { DpadReleased, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };

        enum EightBitDoBleDPadDirection {
            EightBitDoBleDPad_N,
            EightBitDoBleDPad_NE,
            EightBitDoBleDPad_E,
            EightBitDoBleDPad_SE,
            EightBitDoBleDPad_S,
            EightBitDoBleDPad_SW,
            EightBitDoBleDPad_W,
            EightBitDoBleDPad_NW,
        };

    }

    bool EightBitDoBleController::MatchesReportMap(const u8 *map, size_t size) {
        return (size >= sizeof(ReportMapPrefix)) && (std::memcmp(map, ReportMapPrefix, sizeof(ReportMapPrefix)) == 0);
    }

    Result EightBitDoBleController::HandleInputReport(const u8 *data, size_t size) {
        // Only the 9.0.0+ event layout is filled in, as for the other Bluetooth LE controllers
        R_UNLESS(hos::GetVersion() >= hos::Version_9_0_0, sf::ResultNotSupported());

        auto &event = m_event_info.data_report.v9;
        R_UNLESS(size + 1 <= sizeof(event.report.data), svc::ResultInvalidSize());

        event.res = 0;
        event.proto_mode = 0;
        event.addr = m_address;
        event.pad = 0;
        event.report.size = size + 1;
        event.report.data[0] = InputReportId;
        std::memcpy(&event.report.data[1], data, size);

        R_RETURN(this->HandleDataReportEvent(&m_event_info));
    }

    Result EightBitDoBleController::HandleNeutralReport() {
        R_RETURN(this->HandleInputReport(NeutralReport, sizeof(NeutralReport)));
    }

    bool EightBitDoBleController::ReportHasShoulderButtons(const u8 *data, size_t size) const {
        if (size < offsetof(EightBitDoBleInputReport0x01, _unk0)) {
            return false;
        }
        auto report = reinterpret_cast<const EightBitDoBleInputReport0x01 *>(data);
        return report->buttons.L1 && report->buttons.R1;
    }

    bool EightBitDoBleController::ReportHasAnyInput(const u8 *data, size_t size) const {
        if (size < offsetof(EightBitDoBleInputReport0x01, _unk0)) {
            return false;
        }
        auto report = reinterpret_cast<const EightBitDoBleInputReport0x01 *>(data);
        auto buttons = reinterpret_cast<const u8 *>(&report->buttons);
        return ((report->dpad & 0x0f) != DpadReleased) || (report->right_trigger != 0) || (report->left_trigger != 0) || (buttons[0] != 0) || (buttons[1] != 0) || (buttons[2] != 0);
    }

    void EightBitDoBleController::ProcessInputData(const bluetooth::HidReport *report) {
        if ((report->data[0] != InputReportId) || (report->size < offsetof(EightBitDoBleInputReport0x01, _unk0) + 1)) {
            return;
        }

        auto src = reinterpret_cast<const EightBitDoBleInputReport0x01 *>(&report->data[1]);

        m_left_stick  = PackAnalogStickValues(src->left_stick.x,  InvertAnalogStickValue(src->left_stick.y));
        m_right_stick = PackAnalogStickValues(src->right_stick.x, InvertAnalogStickValue(src->right_stick.y));

        const u8 dpad = src->dpad & 0x0f;
        m_buttons.dpad_down  = (dpad == EightBitDoBleDPad_S)  || (dpad == EightBitDoBleDPad_SE) || (dpad == EightBitDoBleDPad_SW);
        m_buttons.dpad_up    = (dpad == EightBitDoBleDPad_N)  || (dpad == EightBitDoBleDPad_NE) || (dpad == EightBitDoBleDPad_NW);
        m_buttons.dpad_right = (dpad == EightBitDoBleDPad_E)  || (dpad == EightBitDoBleDPad_NE) || (dpad == EightBitDoBleDPad_SE);
        m_buttons.dpad_left  = (dpad == EightBitDoBleDPad_W)  || (dpad == EightBitDoBleDPad_NW) || (dpad == EightBitDoBleDPad_SW);

        // Positional mapping onto the Switch face buttons, as for the classic 8BitDo controllers
        m_buttons.A = src->buttons.B;
        m_buttons.B = src->buttons.A;
        m_buttons.X = src->buttons.Y;
        m_buttons.Y = src->buttons.X;

        m_buttons.L = src->buttons.L1;
        m_buttons.R = src->buttons.R1;

        m_buttons.ZL = src->left_trigger  > (m_trigger_threshold * TriggerMax);
        m_buttons.ZR = src->right_trigger > (m_trigger_threshold * TriggerMax);

        m_buttons.minus = src->buttons.select;
        m_buttons.plus  = src->buttons.start;

        m_buttons.lstick_press = src->buttons.L3;
        m_buttons.rstick_press = src->buttons.R3;

        m_buttons.home = src->buttons.home;

        // The back paddles have no counterpart on a Switch controller; they step the console volume
        this->HandleVolumeButtons(src->buttons.L4, src->buttons.R4);

        if (report->size >= sizeof(EightBitDoBleInputReport0x01) + 1) {
            m_battery = convert_battery_100(std::min<u8>(src->battery, 100));
            this->MapMotion(src);
        }
    }

    void EightBitDoBleController::MapMotion(const EightBitDoBleInputReport0x01 *src) {
        // The pad's axes are the Switch's: x forward, y left, z up (measured: 1 g on +x with the triggers up,
        // on -y with the right grip up, on +z lying face up; +z rotation turning it left on the table)
        float gyro[3];
        bool resting = true;
        for (int i = 0; i < 3; ++i) {
            gyro[i] = src->gyro[i] - m_gyro_bias[i];
            resting &= std::abs(gyro[i]) < GyroRestThreshold;
        }

        // The gyroscope reads a little off zero at rest (about 1 dps); follow that while the pad is still
        if (resting) {
            for (int i = 0; i < 3; ++i) {
                m_gyro_bias[i] += gyro[i] * GyroBiasRate;
            }
        }

        m_accel.x = src->accel[0] / AccelPerG;
        m_accel.y = src->accel[1] / AccelPerG;
        m_accel.z = src->accel[2] / AccelPerG;

        m_gyro.x = gyro[0] / GyroPerDps;
        m_gyro.y = gyro[1] / GyroPerDps;
        m_gyro.z = gyro[2] / GyroPerDps;
    }

    void EightBitDoBleController::HandleVolumeButtons(bool down, bool up) {
        const int step = (down && !m_volume_down) ? -1 : (up && !m_volume_up) ? 1 : 0;
        m_volume_down = down;
        m_volume_up = up;
        if (step == 0) {
            return;
        }

        static bool initialized = false;
        if (!initialized) {
            Result rc = audctlInitialize();
            initialized = R_SUCCEEDED(rc);
            if (!initialized) {
                ble::log::Write("audctlInitialize rc=0x%x", rc.GetValue());
                return;
            }
        }

        AudioTarget target = AudioTarget_Invalid;
        s32 volume = 0, min = 0, max = 0;
        Result rc = audctlGetActiveOutputTarget(&target);
        if (R_SUCCEEDED(rc)) rc = audctlGetTargetVolume(&volume, target);
        if (R_SUCCEEDED(rc)) rc = audctlGetTargetVolumeMin(&min);
        if (R_SUCCEEDED(rc)) rc = audctlGetTargetVolumeMax(&max);
        if (R_SUCCEEDED(rc)) rc = audctlSetTargetVolume(target, std::clamp<s32>(volume + step, min, max));
        ble::log::Write("volume %+d: target=%d volume=%d range=%d..%d rc=0x%x", step, target, volume, min, max, rc.GetValue());
    }

    Result EightBitDoBleController::SetVibration(const SwitchMotorData *motor_data) {
        // hid includes rumble data in every output report it sends, at around 60Hz. Turning each one into a GATT
        // write saturates the LE link and starves the input notifications, so only send changes.
        const u8 strong = static_cast<u8>(100 * std::max(motor_data->left_motor.low_band_amp, motor_data->right_motor.low_band_amp));
        const u8 weak   = static_cast<u8>(100 * std::max(motor_data->left_motor.high_band_amp, motor_data->right_motor.high_band_amp));
        R_RETURN(this->WriteRumble(strong, weak));
    }

    Result EightBitDoBleController::CancelVibration() {
        R_RETURN(this->WriteRumble(0, 0));
    }

    void EightBitDoBleController::StopRumble() {
        // Sent even if the last write already was a stop, in case that one was lost
        m_last_rumble_strong = UINT8_MAX;
        static_cast<void>(this->WriteRumble(0, 0));
    }

    Result EightBitDoBleController::WriteRumble(u8 strong, u8 weak) {
        if ((strong == m_last_rumble_strong) && (weak == m_last_rumble_weak)) {
            R_SUCCEED();
        }

        m_last_rumble_strong = strong;
        m_last_rumble_weak = weak;

        EightBitDoBleOutputReport0x05 report = {};
        report.magnitude[0] = strong;
        report.magnitude[1] = weak;

        // Over GATT the characteristic itself identifies the report
        R_RETURN(btdrvWriteGattCharacteristic(m_conn_id, m_service_primary, &m_service_id, &m_output_report_id, &report, sizeof(report), BtdrvGattAuthReqType_NoMitm, false));
    }

}
