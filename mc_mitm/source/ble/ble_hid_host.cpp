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
#include "ble_hid_host.hpp"
#include "ble_events.hpp"
#include "ble_gatt.hpp"
#include "ble_log.hpp"
#include "ble_results.hpp"
#include "../mcmitm_config.hpp"
#include "../mcmitm_initialization.hpp"
#include "../bluetooth_mitm/bluetooth/bluetooth_ble.hpp"
#include "../controllers/controller_management.hpp"
#include "../controllers/xbox_one_ble_controller.hpp"
#include "../controllers/8bitdo_ble_controller.hpp"

namespace ams::ble {

    namespace {

        constexpr s32 ThreadPriority = 9;
        constexpr size_t ThreadStackSize = 0x4000;
        alignas(os::ThreadStackAlignment) constinit u8 g_thread_stack[ThreadStackSize];
        constinit os::ThreadType g_thread;
        constinit bool g_launched = false;

        constexpr size_t LeHidThreadStackSize = 0x2000;
        alignas(os::ThreadStackAlignment) constinit u8 g_le_hid_thread_stack[LeHidThreadStackSize];
        constinit os::ThreadType g_le_hid_thread;
        constinit EventRecord g_le_hid_record;

        // Application UUID for our own GATT client, should we need to register one
        constexpr BtdrvGattAttributeUuid GattClientAppUuid = { 0x10, { 0x4d, 0x43, 0x42, 0x4c, 0x45, 0x48, 0x4f, 0x53, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 } };

        // Transport argument to btdrvCreateBond (BTA convention: 1 = BR/EDR, 2 = LE)
        constexpr u32 BondTransport_Le = 2;

        constinit u8 g_client_if = InvalidClientIf;

        // A refused pairing leaves the stack refusing the next ones for a while, so asking again straight away
        // only keeps that going
        constexpr TimeSpan PairingRetryBackoff = TimeSpan::FromSeconds(30);
        constinit s64 g_pairing_failed_tick = 0;

        /* The controller currently presented to the system */
        constinit os::SdkMutex g_controller_lock;
        std::shared_ptr<controller::BleHidController> g_controller;
        std::shared_ptr<controller::SwitchController> g_switch_controller;

        // Which driver the connected pad gets, told apart by its services once discovered
        enum ControllerKind {
            ControllerKind_Xbox,
            ControllerKind_EightBitDo,
        };
        constinit ControllerKind g_controller_kind = ControllerKind_Xbox;

        // Set while the HID service is registered as a managed data path (8BitDo pads)
        constinit bool g_managed_path = false;

        // 00010203-0405-0607-0809-0a0b0c0d1912, as the stack reports it
        constexpr u8 EightBitDoVendorService[0x10] = { 0x12, 0x19, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };

        void NoteReportMap(const u8 *map, size_t size) {
            const bool eightbitdo = controller::EightBitDoBleController::MatchesReportMap(map, size);
            log::Write("report map size=%u%s", static_cast<u32>(size), eightbitdo ? ", 8BitDo layout" : "");
        }

        // Withdrawn from hid while the Controllers screen is open, as the screen does with every controller;
        // the link stays up and the controller comes back on LB+RB (any button once the screen is closed)
        constinit bool g_parked = false;
        constinit s64 g_present_tick = 0;

        // hid registers a controller on the first input report it sees after re-reading btm's device list, which
        // it does some time after being signalled; a report sent straight after the signal is dropped and the
        // controller only appeared once a button was pressed. The pad's resting state is therefore delivered a
        // few times over the next seconds (ticks at which a copy is due, zero once sent).
        constexpr size_t NeutralReportCount = 4;
        constexpr s64 NeutralReportDelaysMs[NeutralReportCount] = { 0, 300, 1000, 2500 };
        constinit s64 g_neutral_due[NeutralReportCount] = {};

        void ScheduleNeutralReports() {
            const s64 now = os::GetSystemTick().GetInt64Value();
            std::scoped_lock lk(g_controller_lock);
            for (size_t i = 0; i < NeutralReportCount; ++i) {
                g_neutral_due[i] = now + os::ConvertToTick(TimeSpan::FromMilliSeconds(NeutralReportDelaysMs[i])).GetInt64Value();
            }
        }

        void ServiceNeutralReports() {
            std::shared_ptr<controller::BleHidController> controller;
            bool due = false;
            {
                std::scoped_lock lk(g_controller_lock);
                const s64 now = os::GetSystemTick().GetInt64Value();
                for (size_t i = 0; i < NeutralReportCount; ++i) {
                    if ((g_neutral_due[i] != 0) && (now >= g_neutral_due[i])) {
                        g_neutral_due[i] = 0;
                        due = true;
                    }
                }
                if (due && !g_parked) {
                    controller = g_controller;
                }
            }
            if (controller) {
                static_cast<void>(controller->HandleNeutralReport());
            }
        }

        void Unpark() {
            std::shared_ptr<controller::SwitchController> controller;
            {
                std::scoped_lock lk(g_controller_lock);
                if (!g_parked || !g_switch_controller) {
                    return;
                }
                g_parked = false;
                controller = g_switch_controller;
            }
            controller::ReattachVirtualController(controller);
            ScheduleNeutralReports();
            log::Write("controller presented to the system again");
        }

        /* Advertisement parsing */

        struct AdvertisementSummary {
            char name[0x20];
            bool has_appearance;
            u16 appearance;
        };

        void ParseAdvertisements(const BtdrvBleAdvertisement *ad_list, u8 count, AdvertisementSummary *out) {
            std::memset(out, 0, sizeof(*out));

            count = std::min<u8>(count, 10);
            for (u8 i = 0; i < count; ++i) {
                auto &ad = ad_list[i];
                size_t len = std::min<size_t>(ad.size, sizeof(ad.data));

                switch (ad.type) {
                    case BtdrvAdType_ShortenedLocalName:
                    case BtdrvAdType_CompleteLocalName: {
                        size_t n = std::min(len, sizeof(out->name) - 1);
                        std::memcpy(out->name, ad.data, n);
                        out->name[n] = '\0';
                        break;
                    }
                    case BtdrvAdType_Appearance:
                        if (len >= 2) {
                            out->has_appearance = true;
                            out->appearance = ad.data[0] | (ad.data[1] << 8);
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        // The name only travels in the scan response, which isn't always reported. Which driver a pad gets is
        // decided later from its HID Report Map, see NoteReportMap.
        bool IsSupportedController(const AdvertisementSummary &summary) {
            return (std::strncmp(summary.name, "Xbox", 4) == 0) || (std::strncmp(summary.name, "8BitDo", 6) == 0) || (summary.has_appearance && (summary.appearance == AppearanceGamepad));
        }

        // An input Report notification from the connected pad, from whichever stream it arrives on
        void DeliverInputReport(const u8 *data, size_t size) {
            std::shared_ptr<controller::BleHidController> controller;
            bool parked;
            {
                std::scoped_lock lk(g_controller_lock);
                controller = g_controller;
                parked = g_parked;
            }
            if (!controller) {
                return;
            }
            // Debug: the report's non-analog bytes whenever they change (hat, buttons, vendor bytes, tail)
            static u8 last_bytes[8];
            if (size >= 33) {
                const u8 bytes[8] = { data[0], data[7], data[8], data[9], data[10], data[11], data[12], data[32] };
                if (std::memcmp(last_bytes, bytes, sizeof(bytes)) != 0) {
                    std::memcpy(last_bytes, bytes, sizeof(bytes));
                    log::Write("buttons hat=%02x %02x %02x %02x | %02x %02x %02x | tail %02x", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]);
                }
            }
            if (parked) {
                // What brings a controller back on the Controllers screen is L+R; afterwards any button does
                if (controller->ReportHasShoulderButtons(data, size) || (!events::IsPairingWindowOpen() && controller->ReportHasAnyInput(data, size))) {
                    Unpark();
                } else {
                    return;
                }
            }
            static_cast<void>(controller->HandleInputReport(data, size));
        }

        /* LE HID event stream: notifications from services registered with RegisterGattHidDataPath, and the results
           of our GATT requests on them, arrive here rather than on the managed BLE event */

        void LeHidThreadFunc(void *) {
            Event event;
            Result rc = btdrvRegisterBleHidEvent(&event);
            log::Write("btdrvRegisterBleHidEvent rc=0x%x", rc.GetValue());
            if (R_FAILED(rc)) {
                return;
            }

            os::SystemEvent system_event;
            system_event.AttachReadableHandle(event.revent, false, os::EventClearMode_AutoClear);

            for (;;) {
                system_event.Wait();

                rc = btdrvGetLeHidEventInfo(&g_le_hid_record.info, sizeof(g_le_hid_record.info), &g_le_hid_record.type);
                if (R_FAILED(rc)) {
                    log::Write("btdrvGetLeHidEventInfo rc=0x%x", rc.GetValue());
                    continue;
                }

                auto &notify = g_le_hid_record.info.client_notify;
                if ((g_le_hid_record.type != BtdrvBleEventType_ClientNotify) || (notify.conn_id != events::GetConnection())) {
                    events::Log(g_le_hid_record);
                    continue;
                }

                if ((notify.type == GattNotifyType_Notification) && IsUuid16(notify.char_uuid, GattCharReport)) {
                    DeliverInputReport(notify.data, std::min<size_t>(notify.size, sizeof(notify.data)));
                } else {
                    if ((notify.result == 0) && IsUuid16(notify.char_uuid, GattCharReportMap)) {
                        NoteReportMap(notify.data, std::min<size_t>(notify.size, sizeof(notify.data)));
                    }
                    events::NoteGattResult(notify.result);
                    events::Log(g_le_hid_record);
                }
            }
        }

        void StartLeHidListener() {
            R_ABORT_UNLESS(os::CreateThread(&g_le_hid_thread,
                LeHidThreadFunc,
                nullptr,
                g_le_hid_thread_stack,
                LeHidThreadStackSize,
                ThreadPriority
            ));

            os::SetThreadNamePointer(&g_le_hid_thread, "mc::BleHidEventThread");
            os::StartThread(&g_le_hid_thread);
        }

        /* Bring-up */

        Result RegisterGattClient() {
            events::SetRegistering(true);
            ON_SCOPE_EXIT { events::SetRegistering(false); };

            Result rc = btdrvRegisterGattClient(&GattClientAppUuid);
            log::Write("btdrvRegisterGattClient rc=0x%x", rc.GetValue());
            R_TRY(rc);

            EventRecord e;
            if (!events::WaitFor(BtdrvBleEventType_ClientRegistration, TimeSpan::FromSeconds(5), &e, [](const EventRecord &r) { return r.info.client_registration.status == 1; })) {
                R_THROW(svc::ResultTimedOut());
            }

            g_client_if = e.info.client_registration.client_if;
            R_SUCCEED();
        }

        Result PrepareClient() {
            events::TakeOwnership();

            // btm leaves BLE disabled on a retail console. Enabling it here makes btdrv raise a BluetoothCrash
            // event, which btm answers by cycling the radio and bringing BLE up itself, registering four GATT
            // clients on the way; scans only return results after that, so the event is deliberately forwarded.
            Result rc = btdrvEnableBle();
            log::Write("btdrvEnableBle rc=0x%x", rc.GetValue());
            events::Pump(TimeSpan::FromSeconds(6));

            // btm's registrations take most of the stack's client slots; borrow the last one rather than risk
            // finding none free
            g_client_if = events::GetSpareClientIf();
            if (g_client_if != InvalidClientIf) {
                log::Write("using gatt client_if=%u", g_client_if);
                R_SUCCEED();
            }

            for (int attempt = 0; attempt < 3; ++attempt) {
                if (R_SUCCEEDED(RegisterGattClient())) {
                    log::Write("registered gatt client_if=%u", g_client_if);
                    R_SUCCEED();
                }
                os::SleepThread(TimeSpan::FromSeconds(2));
            }

            R_THROW(svc::ResultTimedOut());
        }

        // Blocks while the system has BLE switched off (sleep): any radio activity then wakes the console. A wake
        // re-registers btm's clients, so ours is picked up again afterwards.
        void RecycleRadio();

        void WaitForRadio() {
            bool woke = false;
            if (!events::IsRadioEnabled()) {
                log::Write("ble switched off by the system, waiting");
                events::WaitRadioEnabled();
                log::Write("ble switched back on");
                woke = true;
            }

            if (events::TakeRadioCycled()) {
                events::Pump(TimeSpan::FromSeconds(2));
                if (u8 spare = events::GetSpareClientIf(); spare != InvalidClientIf) {
                    g_client_if = spare;
                }
                log::Write("using gatt client_if=%u", g_client_if);
            }

            // The stack refuses the first pairing after a wake, and the controller only stays in pairing mode for
            // a few seconds after that refusal; cycling straight away lets the first attempt after a wake succeed
            if (woke) {
                log::Write("radio cycle after wake");
                RecycleRadio();
            }
        }

        /* Auto-connect: the remembered controller, reconnected by the stack when it comes back */

        constexpr const char LastDevicePath[] = "sdmc:/config/MissionControl/ble_last_device.bin";

        // Most recently connected first; a new controller pushes the oldest one out
        constexpr size_t MaxRememberedDevices = 4;

        constinit os::SdkMutex g_autoconnect_lock;
        constinit bool g_have_remembered = false;
        constinit size_t g_remembered_count = 0;
        constinit bluetooth::Address g_remembered[MaxRememberedDevices] = {};

        bool AutoConnectEnabled() {
            return mitm::GetGlobalConfig()->ble.auto_connect;
        }

        void LoadRememberedDevice() {
            if (!AutoConnectEnabled()) {
                return;
            }

            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), LastDevicePath, fs::OpenMode_Read))) {
                log::Write("auto-connect: no remembered controller");
                return;
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };

            bluetooth::Address addresses[MaxRememberedDevices];
            size_t read = 0;
            if (R_FAILED(fs::ReadFile(std::addressof(read), file, 0, addresses, sizeof(addresses))) || (read < sizeof(bluetooth::Address))) {
                return;
            }

            const size_t count = read / sizeof(bluetooth::Address);
            {
                std::scoped_lock lk(g_autoconnect_lock);
                std::memcpy(g_remembered, addresses, count * sizeof(bluetooth::Address));
                g_remembered_count = count;
                g_have_remembered = true;
            }
            events::RememberAttempted(&addresses[0]);
            for (size_t i = 0; i < count; ++i) {
                events::LogAddress("auto-connect: remembered controller", addresses[i]);
            }
        }

        void SaveRememberedDevice(const bluetooth::Address &address) {
            bluetooth::Address addresses[MaxRememberedDevices];
            size_t count;
            {
                std::scoped_lock lk(g_autoconnect_lock);
                if (g_have_remembered && AddressEqual(g_remembered[0], address)) {
                    return;
                }
                // Move it to the front, dropping its older entry or the oldest controller
                size_t n = 0;
                addresses[n++] = address;
                for (size_t i = 0; (i < g_remembered_count) && (n < MaxRememberedDevices); ++i) {
                    if (!AddressEqual(g_remembered[i], address)) {
                        addresses[n++] = g_remembered[i];
                    }
                }
                std::memcpy(g_remembered, addresses, n * sizeof(bluetooth::Address));
                g_remembered_count = count = n;
                g_have_remembered = true;
            }

            if (!AutoConnectEnabled()) {
                return;
            }

            const size_t size = count * sizeof(bluetooth::Address);
            static_cast<void>(fs::CreateFile(LastDevicePath, size));
            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), LastDevicePath, fs::OpenMode_Write))) {
                log::Write("auto-connect: could not save the controller");
                return;
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };
            static_cast<void>(fs::SetFileSize(file, size));
            static_cast<void>(fs::WriteFile(file, 0, addresses, size, fs::WriteOption::Flush));
            events::LogAddress("auto-connect: saved controller", address);
        }

        bool IsRememberedDevice(const bluetooth::Address &address) {
            std::scoped_lock lk(g_autoconnect_lock);
            for (size_t i = 0; i < g_remembered_count; ++i) {
                if (AddressEqual(g_remembered[i], address)) {
                    return true;
                }
            }
            return false;
        }

        // A background connection request (ConnectGattServer with is_direct=false) was tried first: the stack
        // accepted it but never connected when the controller was switched on by its Xbox button, and the
        // request could not be cancelled afterwards (0x29e71). Without a Secure Connections bond the controller
        // has no console to reconnect to and just switches itself off again, so the Xbox button alone cannot
        // bring it back. What works outside the Controllers screen is the sync button: the controller then
        // advertises to anyone, and a scan finds it at once since btm's inquiry only runs while the screen is
        // open. Scanning in bursts while no controller is connected keeps the radio mostly free.
        // No BLE scan runs within this long of classic activity (a controller being connected or paired): the
        // stack crashed, and btm cycled the radio, every time a scan was running while a classic controller
        // connected (seven times in a row in one session, 10-13 s after each scan started)
        constexpr TimeSpan ClassicActivityHoldOff = TimeSpan::FromSeconds(12);

        constexpr TimeSpan BackgroundScanDuration = TimeSpan::FromMilliSeconds(2500);
        constexpr TimeSpan BackgroundScanPause    = TimeSpan::FromMilliSeconds(2000);

        bool BackgroundScanWanted() {
            return AutoConnectEnabled() && g_have_remembered && events::IsRadioEnabled() && !events::IsPairingWindowOpen();
        }

        // Set when PrepareForSleep dropped the link: that end of link is no reason to cycle the radio on wake
        constinit bool g_dropped_for_sleep = false;

        // EXPERIMENT. The stack refuses every CreateBond for a controller that has been switched off until the
        // console restarts. A restart is not the only thing that resets the stack: btm cycles the radio whenever
        // btdrv reports a BluetoothCrash, which is how BLE came up in PrepareClient. Ask btdrv for one and pick up
        // the GATT client btm registers afterwards, as the bring-up does.
        void RecycleRadio() {
            if (g_dropped_for_sleep) {
                g_dropped_for_sleep = false;
                log::Write("link was dropped for sleep, no radio cycle");
                return;
            }

            if (!mitm::GetGlobalConfig()->ble.radio_cycle || !events::IsRadioEnabled()) {
                return;
            }

            events::ClearSpareClientIf();

            // Other controllers drop with the stack and reconnect on their own button; mc.mitm drops their stale
            // handlers when btm enables Bluetooth again (see BtdrvMitmService::EnableBluetooth)
            log::Write("radio cycle with %u other controller(s) connected", static_cast<u32>(controller::GetControllerCount()));

            Result rc = btdrvEmulateBluetoothCrash(BtdrvFatalReason_Unknown1);
            log::Write("radio cycle: btdrvEmulateBluetoothCrash rc=0x%x", rc.GetValue());
            if (R_FAILED(rc)) {
                return;
            }

            events::Pump(TimeSpan::FromSeconds(6));
            g_pairing_failed_tick = 0;

            if (u8 spare = events::GetSpareClientIf(); spare != InvalidClientIf) {
                g_client_if = spare;
                log::Write("radio cycle: btm brought BLE back up, using gatt client_if=%u", g_client_if);
                return;
            }

            // btm did not bring BLE back up on its own; do what the bring-up does
            rc = btdrvEnableBle();
            log::Write("radio cycle: btdrvEnableBle rc=0x%x", rc.GetValue());
            events::Pump(TimeSpan::FromSeconds(6));

            if (u8 spare = events::GetSpareClientIf(); spare != InvalidClientIf) {
                g_client_if = spare;
                log::Write("radio cycle: using gatt client_if=%u", g_client_if);
                return;
            }

            for (int attempt = 0; attempt < 3; ++attempt) {
                if (R_SUCCEEDED(RegisterGattClient())) {
                    log::Write("radio cycle: registered gatt client_if=%u", g_client_if);
                    return;
                }
                os::SleepThread(TimeSpan::FromSeconds(2));
            }

            log::Write("radio cycle: no gatt client available, keeping client_if=%u", g_client_if);
        }

        /* Finding and connecting the controller */

        // With `only` set, nothing but that controller is accepted (background scans outside the pairing window)
        Result Scan(bluetooth::Address *out_address, TimeSpan duration, bool only_remembered = false) {
            events::SetScanning(true);
            ON_SCOPE_EXIT { events::SetScanning(false); };

            // Nothing is reported while btm's scan filter conditions are installed; the system itself clears
            // them whenever the HOME menu is left, so nothing depends on them
            events::ExpectFilterEvent();
            static_cast<void>(btdrvClearBleScanFilters());
            events::ExpectFilterEvent();
            static_cast<void>(btdrvEnableBleScanFilter(false));

            // The two filter calls answer asynchronously; let them land before asking for the scan
            for (int i = 0; i < 2; ++i) {
                events::WaitFor(BtdrvBleEventType_ScanFilter, TimeSpan::FromMilliSeconds(400), nullptr, [](const EventRecord &) { return true; });
            }

            // The stack refuses the scan (0x2a871) for as long as btm's classic inquiry has the radio: 10.24 s at
            // a time from the moment the Controllers screen opens, repeated with hardly a gap while it stays
            // open. Keep asking while the screen is open; outside it the scan starts at once.
            Result rc;
            int attempts = 0;
            const bool in_window = events::IsPairingWindowOpen();
            for (;;) {
                if (events::ClassicActivityWithin(ClassicActivityHoldOff)) {
                    log::Write("scan not started: a classic controller is connecting");
                    events::Pump(TimeSpan::FromSeconds(1));
                    R_THROW(svc::ResultCancelled());
                }
                rc = btdrvStartBleScan();
                if (R_SUCCEEDED(rc) || (++attempts >= (in_window ? 150 : 3)) || (in_window && !events::IsPairingWindowOpen())) {
                    break;
                }
                if (attempts == 1) {
                    log::Write("btdrvStartBleScan rc=0x%x, retrying", rc.GetValue());
                }
                events::Pump(TimeSpan::FromMilliSeconds(200));
            }
            if (R_FAILED(rc)) {
                log::Write("btdrvStartBleScan rc=0x%x after %d attempts", rc.GetValue(), attempts);
                events::Pump(TimeSpan::FromSeconds(1));
                return rc;
            }
            if (attempts > 0) {
                log::Write("scanning (%d refused attempts first)", attempts);
            }

            EventRecord e;
            // Ends early if a link came up by itself or the console is going to sleep
            // Waited for in short slices so that the scan can be stopped as soon as a classic controller starts
            // connecting (see NoteClassicActivity), not only when an event happens to arrive
            const auto deadline = os::GetSystemTick().ToTimeSpan() + duration;
            bool found = false;
            bool aborted = false;
            for (;;) {
                const auto now = os::GetSystemTick().ToTimeSpan();
                if (now >= deadline) {
                    break;
                }
                found = events::WaitFor(BtdrvBleEventType_ScanResult, std::min(deadline - now, TimeSpan::FromMilliSeconds(250)), &e, [only_remembered](const EventRecord &r) {
                    if (r.info.scan_result.status != BleScanStatus_NewDevice) {
                        return false;
                    }
                    if (only_remembered) {
                        return IsRememberedDevice(r.info.scan_result.address);
                    }
                    AdvertisementSummary summary;
                    ParseAdvertisements(r.info.scan_result.ad_list, r.info.scan_result.count, &summary);
                    return IsSupportedController(summary);
                });
                if (found) {
                    break;
                }
                if (events::HasAdoptedConnection() || !events::IsRadioEnabled() || events::ClassicActivityWithin(ClassicActivityHoldOff)) {
                    aborted = true;
                    break;
                }
            }

            static_cast<void>(btdrvStopBleScan());
            events::ExpectFilterEvent();
            static_cast<void>(btdrvEnableBleScanFilter(true));

            // The scan-complete and filter events trail the calls above
            events::Pump(TimeSpan::FromSeconds(1));

            if (aborted && !events::HasAdoptedConnection() && events::IsRadioEnabled()) {
                log::Write("scan stopped: a classic controller is connecting");
            }
            R_UNLESS(!aborted, svc::ResultCancelled());
            R_UNLESS(found, svc::ResultTimedOut());

            *out_address = e.info.scan_result.address;
            events::LogAddress("found controller", *out_address);

            if (log::IsEnabled()) {
                SetSysBluetoothDevicesSettings settings;
                Result info_rc = btdrvGetPairedDeviceInfo(*out_address, &settings);
                log::Write("btdrvGetPairedDeviceInfo rc=0x%x key_present=%u key_type=%u device_type=%u",
                    info_rc.GetValue(), settings.link_key_present, settings.key_type, settings.device_type);
            }

            R_SUCCEED();
        }

        Result WaitForConnection(const bluetooth::Address &address, u32 *out_conn_id, TimeSpan timeout) {
            EventRecord e;
            if (!events::WaitFor(BtdrvBleEventType_ClientConnection, timeout, &e, [&](const EventRecord &r) {
                return AddressEqual(r.info.client_connection.address, address) || (r.info.client_connection.result != 0);
            })) {
                R_THROW(svc::ResultTimedOut());
            }

            if ((e.info.client_connection.result != 0) || (e.info.client_connection.status != BleConnectionStatus_Connected)) {
                log::Write("connection failed result=%u status=%u", e.info.client_connection.result, e.info.client_connection.status);
                R_THROW(ResultNoConnection());
            }

            *out_conn_id = e.info.client_connection.conn_id;
            events::SetConnection(*out_conn_id);
            log::Write("connected conn_id=%u", *out_conn_id);
            R_SUCCEED();
        }

        Result Connect(const bluetooth::Address &address, u32 *out_conn_id, TimeSpan timeout) {
            events::SetTarget(&address);

            // A record left for this address, including the one a classic firmware left behind, is encrypted
            // with the moment the link comes up, and the controller drops the link over a key it doesn't hold
            Result rc = btdrvRemoveBond(address);
            log::Write("btdrvRemoveBond rc=0x%x", rc.GetValue());

            rc = btdrvConnectGattServer(g_client_if, address, true, 0);
            log::Write("btdrvConnectGattServer rc=0x%x", rc.GetValue());
            R_TRY(rc);

            rc = WaitForConnection(address, out_conn_id, timeout);
            if (R_FAILED(rc)) {
                static_cast<void>(btdrvCancelConnectGattServer(g_client_if, address, true));
            }
            R_RETURN(rc);
        }

        bool PairingBackoffActive() {
            if (g_pairing_failed_tick == 0) {
                return false;
            }
            return (os::GetSystemTick() - os::Tick(g_pairing_failed_tick)).ToTimeSpan() < PairingRetryBackoff;
        }

        bool Adopt(bluetooth::Address *out_address, u32 *out_conn_id) {
            if (!events::TakeAdoptedConnection(out_address, out_conn_id)) {
                return false;
            }
            log::Write("controller connected on its own, conn_id=%u", *out_conn_id);
            return true;
        }

        // The stack never pairs on its own: an authenticated request on an unbonded link just fails. Pairing is
        // Just Works and needs no confirmation from the controller.
        bool Pair(const bluetooth::Address &address) {
            Result rc = btdrvCreateBond(address, BondTransport_Le);
            log::Write("btdrvCreateBond rc=0x%x", rc.GetValue());
            if (R_FAILED(rc)) {
                return false;
            }

            // The link-state event that reports the outcome is the same for success (~5 s) and refusal (at once,
            // followed by the stack dropping the link)
            const auto deadline = os::GetSystemTick().ToTimeSpan() + TimeSpan::FromSeconds(10);
            while (os::GetSystemTick().ToTimeSpan() < deadline) {
                events::Pump(TimeSpan::FromMilliSeconds(200));
                if (events::IsDisconnected()) {
                    break;
                }
                if (events::TakeBondComplete()) {
                    // A refusal drops the link within a few hundred milliseconds of this event
                    events::Pump(TimeSpan::FromMilliSeconds(500));
                    if (!events::IsDisconnected()) {
                        log::Write("paired");
                        return true;
                    }
                    break;
                }
            }
            log::Write("pairing failed");
            return false;
        }

        /* Presenting the controller to the system */

        void Present(u32 conn_id, const bluetooth::Address &address, const gatt::HidService &hid) {
            if (!hid.has_output_report) {
                log::Write("no output Report characteristic, rumble unavailable");
            }

            std::shared_ptr<controller::BleHidController> ble_controller;
            std::shared_ptr<controller::SwitchController> controller;
            if (g_controller_kind == ControllerKind_EightBitDo) {
                auto c = std::make_shared<controller::EightBitDoBleController>(address, conn_id, hid.service.id, hid.service.is_primary, hid.output_report.id);
                ble_controller = c;
                controller = c;
            } else {
                auto c = std::make_shared<controller::XboxOneBleController>(address, conn_id, hid.service.id, hid.service.is_primary, hid.output_report.id);
                ble_controller = c;
                controller = c;
            }
            {
                std::scoped_lock lk(g_controller_lock);
                g_controller = ble_controller;
                g_switch_controller = controller;
                g_parked = false;
                g_present_tick = os::GetSystemTick().GetInt64Value();
            }
            controller::AttachVirtualController(controller);
            g_pairing_failed_tick = 0;
            SaveRememberedDevice(address);

            // The pad only reports when something is touched, so its resting state (sticks centred, nothing
            // pressed) is delivered for it, now and again over the next seconds, see ScheduleNeutralReports
            ScheduleNeutralReports();
            ServiceNeutralReports();

            log::Write("controller presented to the system");
        }

        void Dismiss(const bluetooth::Address &address) {
            std::shared_ptr<controller::SwitchController> controller;
            bool parked;
            {
                std::scoped_lock lk(g_controller_lock);
                controller = std::move(g_switch_controller);
                g_switch_controller = nullptr;
                g_controller = nullptr;
                parked = g_parked;
                g_parked = false;
                for (auto &due : g_neutral_due) {
                    due = 0;
                }
            }

            if (controller && !parked) {
                controller::DetachVirtualController(address);
                log::Write("controller removed from the system");
            }
        }

        void Disconnect() {
            const u32 conn_id = events::GetConnection();

            bluetooth::Address address;
            if (events::GetTarget(&address)) {
                Dismiss(address);
            }

            // Nothing may touch the radio while the system has BLE switched off: it wakes the console
            if ((conn_id != InvalidConnId) && !events::IsDisconnected() && events::IsRadioEnabled()) {
                static_cast<void>(btdrvDisconnectGattServer(conn_id));
            }

            // Keep the target until the stack has finished tearing the link down
            events::Pump(TimeSpan::FromSeconds(1));

            events::SetConnection(InvalidConnId);
            events::SetTarget(nullptr);
            events::SetClaimedAddress(nullptr);
            events::SetInputSink(nullptr);
            events::SetCountManagedGattResults(false);
            if (g_managed_path) {
                auto hid_uuid = MakeUuid16(GattServiceHid);
                static_cast<void>(btdrvUnregisterGattManagedDataPath(&hid_uuid));
                g_managed_path = false;
            }
        }

        void RunConnected(u32 conn_id) {
            for (;;) {
                events::Pump(TimeSpan::FromMilliSeconds(250));
                ServiceNeutralReports();

                if (gatt::TakeRediscovered()) {
                    log::Write("services re-discovered, subscribing again");
                    gatt::HidService hid;
                    static_cast<void>(gatt::Subscribe(conn_id, &hid));
                }

                if (events::IsDisconnected()) {
                    log::Write("controller disconnected reason=0x%04x", events::GetDisconnectReason());
                    return;
                }

                if (!events::IsRadioEnabled()) {
                    return;
                }
            }
        }

        /*
         * A controller is paired while the Controllers screen has the system searching, the same window in which
         * a Pro Controller pairs, and used until its link drops. Every connection is a fresh pairing: the console's
         * stack (Broadcom BSA 1.6) only knows LE legacy pairing and these pads keep a bond only from LE Secure
         * Connections, so a pad that reconnected on its own would refuse the stored key anyway.
         */
        void HostThreadFunc(void *) {
            mitm::WaitInitialized();
            bluetooth::ble::WaitInitialized();

            // Bringing BLE up cycles the radio (see PrepareClient); let the system finish its own bring-up first
            os::SleepThread(TimeSpan::FromSeconds(20));
            log::Write("starting");

            if (R_FAILED(PrepareClient())) {
                log::Write("giving up: no gatt client available");
                return;
            }

            StartLeHidListener();
            events::SetSink(gatt::OnEvent);
            LoadRememberedDevice();

            for (;;) {
                WaitForRadio();

                bluetooth::Address address;
                u32 conn_id = InvalidConnId;

                bool connected = Adopt(&address, &conn_id);

                // Any gamepad in pairing mode while the Controllers screen is open; only the remembered one,
                // in short bursts, the rest of the time
                const bool classic_busy = events::ClassicActivityWithin(ClassicActivityHoldOff);
                const bool in_window = !classic_busy && events::IsPairingWindowOpen() && events::IsRadioEnabled() && !PairingBackoffActive();
                const bool background = !classic_busy && !in_window && !PairingBackoffActive() && BackgroundScanWanted();
                if (!connected && (in_window || background)) {
                    Result rc = in_window ? Scan(&address, TimeSpan::FromSeconds(10)) : Scan(&address, BackgroundScanDuration, true);
                    if (Adopt(&address, &conn_id)) {
                        connected = true;
                    } else if (R_SUCCEEDED(rc)) {
                        if (background) {
                            log::Write("remembered controller is in pairing mode, connecting");
                        }
                        connected = R_SUCCEEDED(Connect(address, &conn_id, TimeSpan::FromSeconds(15)));
                        if (!connected) {
                            Disconnect();
                        }
                    }
                }

                if (!connected) {
                    os::SleepThread(background ? BackgroundScanPause : TimeSpan::FromSeconds(2));
                    continue;
                }

                // Pairing traffic for this pad is answered on its behalf from here on
                events::SetClaimedAddress(&address);

                gatt::Discover(conn_id, address);

                // A link that drops as soon as it comes up (0x3e) leaves nothing to tell the pad by; just try again
                if (events::IsDisconnected()) {
                    log::Write("link lost during discovery reason=0x%04x", events::GetDisconnectReason());
                    Disconnect();
                    os::SleepThread(TimeSpan::FromSeconds(1));
                    continue;
                }

                // 8BitDo pads carry a vendor service of their own next to the HID one
                g_controller_kind = gatt::HasService128(EightBitDoVendorService) ? ControllerKind_EightBitDo : ControllerKind_Xbox;
                log::Write("%s controller", (g_controller_kind == ControllerKind_EightBitDo) ? "8BitDo" : "Xbox");

                // EXPERIMENT. An 8BitDo pad answers the console's LE legacy pairing at once and then serves nothing
                // that needs encryption; see whether it serves its HID service without, before pairing at all
                bool paired = false;
                if (g_controller_kind != ControllerKind_EightBitDo) {
                    if (!Pair(address)) {
                        g_pairing_failed_tick = os::GetSystemTick().GetInt64Value();
                        log::Write("backing off for %lds before trying to pair again", PairingRetryBackoff.GetSeconds());
                        Disconnect();
                        // A cycle clears the backoff: the next attempt is against a fresh stack. A connection asked for
                        // within a couple of seconds of the cycle fails to be established (0x3e), so let it settle.
                        RecycleRadio();
                        os::SleepThread(TimeSpan::FromSeconds(3));
                        continue;
                    }
                    paired = true;
                }

                // Hands the HID service to the stack's HID host layer for this link: its notifications, and the results
                // of our requests on it, then arrive on the LE HID event stream. That layer secures the link itself,
                // and an 8BitDo pad never completes the pairing it starts: the stack drops the link when it times
                // out 30 s later. Its service is used directly instead, over the managed BLE event stream.
                const bool managed = (g_controller_kind == ControllerKind_EightBitDo);
                events::SetCountManagedGattResults(managed);
                events::SetInputSink(managed ? DeliverInputReport : nullptr);
                auto hid_uuid = MakeUuid16(GattServiceHid);
                Result rc;
                if (managed) {
                    // Without a data path the stack reports nothing about the service at all
                    rc = btdrvRegisterGattManagedDataPath(&hid_uuid);
                    log::Write("btdrvRegisterGattManagedDataPath rc=0x%x", rc.GetValue());
                    g_managed_path = R_SUCCEEDED(rc);
                } else {
                    rc = btdrvRegisterGattHidDataPath(&hid_uuid);
                    log::Write("btdrvRegisterGattHidDataPath rc=0x%x", rc.GetValue());
                }

                // The subscription is judged by the CCCD write's result; the pad only reports on input
                events::ClearGattFailure();
                gatt::HidService hid;
                rc = gatt::Subscribe(conn_id, &hid, paired ? BtdrvGattAuthReqType_NoMitm : BtdrvGattAuthReqType_None);
                if (R_FAILED(rc) && !paired && !events::IsDisconnected()) {
                    log::Write("unencrypted subscription failed rc=0x%x gatt_result=%u, pairing", rc.GetValue(), events::GetGattFailure());
                    paired = Pair(address);
                    if (paired) {
                        events::ClearGattFailure();
                        rc = gatt::Subscribe(conn_id, &hid);
                    }
                }
                if (R_FAILED(rc)) {
                    log::Write("subscription failed rc=0x%x gatt_result=%u", rc.GetValue(), events::GetGattFailure());
                    Disconnect();
                    os::SleepThread(TimeSpan::FromSeconds(1));
                    continue;
                }

                Present(conn_id, address, hid);
                RunConnected(conn_id);

                // Nothing beyond the teardown is asked of the stack once the link is gone: removing the bond and
                // detaching the HID host layer here were both tried and changed nothing. A radio cycle is the one
                // thing left to try (skipped when the link ended because the console is going to sleep).
                Disconnect();
                RecycleRadio();
            }
        }

    }

    bool IsEnabled() {
        return mitm::GetGlobalConfig()->ble.enable && (hos::GetVersion() >= hos::Version_13_0_0);
    }

    void Launch() {
        if (!IsEnabled()) {
            return;
        }

        auto config = mitm::GetGlobalConfig();
        if (config->ble.log) {
            log::Initialize();
        }
        events::Activate();

        R_ABORT_UNLESS(os::CreateThread(&g_thread,
            HostThreadFunc,
            nullptr,
            g_thread_stack,
            ThreadStackSize,
            ThreadPriority
        ));

        os::SetThreadNamePointer(&g_thread, "mc::BleHidHostThread");
        os::StartThread(&g_thread);

        g_launched = true;
    }

    void WaitFinished() {
        if (g_launched) {
            os::WaitThread(&g_thread);
        }
    }

    void OnPairingWindow(bool open) {
        // btm restarts its inquiry every 10 s while the screen stays open, and stops and restarts it within a
        // few seconds when a controller connects; only a start after a quiet spell is the screen being opened.
        // A controller that was presented from within the screen stays, as any newly paired controller does.
        constexpr TimeSpan NewSessionGap = TimeSpan::FromSeconds(15);
        static constinit s64 s_last_close_tick = 0;

        const os::Tick now = os::GetSystemTick();
        if (!open) {
            // A parked controller comes back on its next button press, see LeHidThreadFunc
            s_last_close_tick = now.GetInt64Value();
            return;
        }

        // The screen is only inferred from btm's inquiry and was seen to withdraw the controller again right after
        // LB+RB brought it back; it read as the controller dropping off the console. Left connected instead.
        constexpr bool WithdrawOnControllersScreen = false;
        if (!WithdrawOnControllersScreen) {
            return;
        }

        if ((s_last_close_tick != 0) && ((now - os::Tick(s_last_close_tick)).ToTimeSpan() < NewSessionGap)) {
            return;
        }

        std::shared_ptr<controller::SwitchController> controller;
        std::shared_ptr<controller::BleHidController> ble_controller;
        {
            std::scoped_lock lk(g_controller_lock);
            if (!g_switch_controller || g_parked || ((now - os::Tick(g_present_tick)).ToTimeSpan() < NewSessionGap)) {
                return;
            }
            g_parked = true;
            controller = g_switch_controller;
            ble_controller = g_controller;
            for (auto &due : g_neutral_due) {
                due = 0;
            }
        }
        if (ble_controller) {
            ble_controller->StopRumble();
        }
        controller::DetachVirtualController(controller->Address());
        log::Write("controller withdrawn while the Controllers screen is open, LB+RB brings it back");
    }

    // EXPERIMENT. A controller left connected wakes the console a few seconds after it goes to sleep. The link
    // is dropped here, from the power module thread, while the bluetooth module is still awake (it is listed as
    // a dependency of ours), so that nothing is left on the air by the time the radio goes down. The host thread
    // sees the radio marked unavailable and stays off it until FullAwake.
    void PrepareForSleep() {
        const bool was_enabled = events::IsRadioEnabled();
        events::SetRadioEnabled(false);

        // MinimumAwake and SleepReady both come through here; only the first one has anything to do
        if (!g_launched || !was_enabled) {
            return;
        }

        const u32 conn_id = events::GetConnection();
        if ((conn_id == InvalidConnId) || events::IsDisconnected()) {
            log::Write("going to sleep, no link to drop");
            return;
        }

        g_dropped_for_sleep = true;
        Result rc = btdrvDisconnectGattServer(conn_id);
        log::Write("going to sleep: btdrvDisconnectGattServer conn_id=%u rc=0x%x", conn_id, rc.GetValue());

        // The disconnection is reported on btdrv's event thread, independently of the host thread
        for (int i = 0; (i < 30) && !events::IsDisconnected(); ++i) {
            os::SleepThread(TimeSpan::FromMilliSeconds(50));
        }
        log::Write(events::IsDisconnected() ? "link down before sleep" : "link still up as sleep begins");
    }

}
