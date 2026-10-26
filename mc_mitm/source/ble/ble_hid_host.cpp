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

        // EXPERIMENT. A second link at once was refused (0x29e71) on the client_if already holding a link; another
        // client is tried for it: one registered for us if the stack has room, else one of btm's (it never connects
        // with them on a retail console). Gone with every radio cycle.
        constinit u8 g_extra_client_if = InvalidClientIf;
        // The client the link being set up was connected with
        constinit u8 g_setup_client_if = InvalidClientIf;

        // A refused pairing leaves the stack refusing the next ones for a while, so asking again straight away
        // only keeps that going
        constexpr TimeSpan PairingRetryBackoff = TimeSpan::FromSeconds(30);
        constinit s64 g_pairing_failed_tick = 0;

        // Which driver a pad gets, told apart by its services once discovered
        enum ControllerKind {
            ControllerKind_Xbox,
            ControllerKind_EightBitDo,
        };

        // Kind of the pad being set up
        constinit ControllerKind g_controller_kind = ControllerKind_Xbox;

        // Set while the HID service is registered as a managed data path (8BitDo pads). The registration is by
        // service UUID, not per link, so it stays while any 8BitDo link needs it.
        constinit bool g_managed_path = false;

        // EXPERIMENT. Both data paths are registered by service UUID, so an Xbox pad on the HID data path and an
        // 8BitDo pad on the managed one exclude each other (the second registration fails with 0x2ac71). The
        // managed path was chosen for 8BitDo pads before the SMP timer patch, when the HID host layer dropped them
        // 30 s into a pairing they never finish; the HID data path is tried for them again. An 8BitDo link that
        // fails to subscribe there, or drops within LinkProbation of being presented, sends them back to the
        // managed path for the rest of the session.
        constinit bool g_eightbitdo_managed = false;
        constexpr TimeSpan LinkProbation = TimeSpan::FromSeconds(45);

        // 00010203-0405-0607-0809-0a0b0c0d1912, as the stack reports it
        constexpr u8 EightBitDoVendorService[0x10] = { 0x12, 0x19, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };

        void NoteReportMap(const u8 *map, size_t size) {
            const bool eightbitdo = controller::EightBitDoBleController::MatchesReportMap(map, size);
            log::Write("report map size=%u%s", static_cast<u32>(size), eightbitdo ? ", 8BitDo layout" : "");
        }

        // hid registers a controller on the first input report it sees after re-reading btm's device list, which
        // it does some time after being signalled; a report sent straight after the signal is dropped and the
        // controller only appeared once a button was pressed. The pad's resting state is therefore delivered a
        // few times over the next seconds (ticks at which a copy is due, zero once sent).
        constexpr size_t NeutralReportCount = 4;
        constexpr s64 NeutralReportDelaysMs[NeutralReportCount] = { 0, 300, 1000, 2500 };

        /* Controllers presented to the system, one per established link. Links are set up one at a time by the
           host thread; once presented, a link runs on its own (its input arrives on the btdrv event threads) while
           the host thread looks for the next controller. */
        constexpr size_t MaxLinks = 2;

        struct Link {
            bool used = false;
            u32 conn_id = InvalidConnId;
            u8 client_if = InvalidClientIf;
            bluetooth::Address address = {};
            ControllerKind kind = ControllerKind_Xbox;
            bool paired = false;    // bonded (Xbox): it pairs again only after a radio cycle
            bool managed = false;   // HID service on the managed data path (8BitDo)
            std::shared_ptr<controller::BleHidController> controller;
            std::shared_ptr<controller::SwitchController> switch_controller;
            s64 neutral_due[NeutralReportCount] = {};
            s64 present_tick = 0;
        };

        constinit os::SdkMutex g_controller_lock;
        Link g_links[MaxLinks];

        size_t LinkCount() {
            std::scoped_lock lk(g_controller_lock);
            size_t count = 0;
            for (auto &link : g_links) {
                count += link.used ? 1 : 0;
            }
            return count;
        }

        bool IsLinked(const bluetooth::Address &address) {
            std::scoped_lock lk(g_controller_lock);
            for (auto &link : g_links) {
                if (link.used && AddressEqual(link.address, address)) {
                    return true;
                }
            }
            return false;
        }

        bool ClientIfInUse(u8 client_if) {
            std::scoped_lock lk(g_controller_lock);
            for (auto &link : g_links) {
                if (link.used && (link.client_if == client_if)) {
                    return true;
                }
            }
            return false;
        }

        size_t GetEightBitDoLinks(bluetooth::Address *out_addresses, u32 *out_conn_ids) {
            std::scoped_lock lk(g_controller_lock);
            size_t count = 0;
            for (auto &link : g_links) {
                if (link.used && (link.kind == ControllerKind_EightBitDo)) {
                    out_addresses[count] = link.address;
                    out_conn_ids[count] = link.conn_id;
                    ++count;
                }
            }
            return count;
        }

        // While set, 8BitDo pads are not connected: the stack was restarted under one so that an Xbox pad could
        // pair first (see the host loop)
        constinit s64 g_eightbitdo_held_until = 0;
        constexpr TimeSpan EightBitDoHoldForPairing = TimeSpan::FromSeconds(25);

        void HoldEightBitDo() {
            g_eightbitdo_held_until = (os::GetSystemTick() + os::ConvertToTick(EightBitDoHoldForPairing)).GetInt64Value();
        }

        bool EightBitDoHeld() {
            return (g_eightbitdo_held_until != 0) && (os::GetSystemTick().GetInt64Value() < g_eightbitdo_held_until);
        }

        bool AnyManagedLink() {
            std::scoped_lock lk(g_controller_lock);
            for (auto &link : g_links) {
                if (link.used && link.managed) {
                    return true;
                }
            }
            return false;
        }

        void ScheduleNeutralReports(Link &link) {
            const s64 now = os::GetSystemTick().GetInt64Value();
            for (size_t i = 0; i < NeutralReportCount; ++i) {
                link.neutral_due[i] = now + os::ConvertToTick(TimeSpan::FromMilliSeconds(NeutralReportDelaysMs[i])).GetInt64Value();
            }
        }

        void ServiceNeutralReports() {
            std::shared_ptr<controller::BleHidController> due_controllers[MaxLinks];
            size_t count = 0;
            {
                std::scoped_lock lk(g_controller_lock);
                const s64 now = os::GetSystemTick().GetInt64Value();
                for (auto &link : g_links) {
                    if (!link.used) {
                        continue;
                    }
                    bool due = false;
                    for (auto &tick : link.neutral_due) {
                        if ((tick != 0) && (now >= tick)) {
                            tick = 0;
                            due = true;
                        }
                    }
                    if (due) {
                        due_controllers[count++] = link.controller;
                    }
                }
            }
            for (size_t i = 0; i < count; ++i) {
                static_cast<void>(due_controllers[i]->HandleNeutralReport());
            }
        }

        // The registration outlives a link only while another 8BitDo link still uses it
        void ReleaseManagedPath() {
            if (g_managed_path && !AnyManagedLink()) {
                auto hid_uuid = MakeUuid16(GattServiceHid);
                static_cast<void>(btdrvUnregisterGattManagedDataPath(&hid_uuid));
                g_managed_path = false;
            }
        }

        // Withdraws a link's controller from the system. The link is closed first unless it is already down.
        // Returns whether the pad was bonded (an Xbox controller), whose end calls for a radio cycle.
        bool TeardownLink(u32 conn_id, bool link_down, bool by_cycle = false) {
            Link removed;
            {
                std::scoped_lock lk(g_controller_lock);
                for (auto &link : g_links) {
                    if (link.used && (link.conn_id == conn_id)) {
                        removed = std::move(link);
                        link = Link{};
                        break;
                    }
                }
            }
            if (!removed.used) {
                return false;
            }

            events::RemoveLink(conn_id);
            // Nothing may touch the radio while the system has BLE switched off: it wakes the console
            if (!link_down && events::IsRadioEnabled()) {
                static_cast<void>(btdrvDisconnectGattServer(conn_id));
            }
            // A rumble write would touch the radio, which wakes a sleeping console
            if (removed.controller && events::IsRadioEnabled()) {
                removed.controller->StopRumble();
            }
            if (removed.switch_controller) {
                controller::DetachVirtualController(removed.address);
            }
            events::RemoveClaimedAddress(removed.address);
            ReleaseManagedPath();
            events::LogAddress("controller removed from the system", removed.address);

            if (link_down && !by_cycle && (removed.kind == ControllerKind_EightBitDo) && !removed.managed && !g_eightbitdo_managed &&
                ((os::GetSystemTick() - os::Tick(removed.present_tick)).ToTimeSpan() < LinkProbation)) {
                g_eightbitdo_managed = true;
                log::Write("8BitDo link on the HID data path ended early, back to the managed data path");
            }
            return removed.paired;
        }

        // A radio cycle takes every link down with it. The pads are not told: one the host disconnects takes it as
        // being let go and switches itself off (an 8BitDo pad, like a DualShock), one that loses the link looks
        // for the console again.
        void DropAllLinks() {
            u32 conn_ids[MaxLinks];
            size_t count = 0;
            {
                std::scoped_lock lk(g_controller_lock);
                for (auto &link : g_links) {
                    if (link.used) {
                        conn_ids[count++] = link.conn_id;
                    }
                }
            }
            for (size_t i = 0; i < count; ++i) {
                static_cast<void>(TeardownLink(conn_ids[i], true, true));
            }
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

        // An input Report notification from a connected pad, from whichever stream it arrives on
        void DeliverInputReport(u32 conn_id, const u8 *data, size_t size) {
            std::shared_ptr<controller::BleHidController> controller;
            bool eightbitdo = false;
            {
                std::scoped_lock lk(g_controller_lock);
                for (auto &link : g_links) {
                    if (link.used && (link.conn_id == conn_id)) {
                        controller = link.controller;
                        eightbitdo = link.kind == ControllerKind_EightBitDo;
                        break;
                    }
                }
            }
            if (!controller) {
                return;
            }
            // Debug: an 8BitDo report's non-analog bytes whenever they change (hat, buttons, vendor bytes, tail)
            static u8 last_bytes[8];
            if (eightbitdo && (size >= 33)) {
                const u8 bytes[8] = { data[0], data[7], data[8], data[9], data[10], data[11], data[12], data[32] };
                if (std::memcmp(last_bytes, bytes, sizeof(bytes)) != 0) {
                    std::memcpy(last_bytes, bytes, sizeof(bytes));
                    log::Write("buttons hat=%02x %02x %02x %02x | %02x %02x %02x | tail %02x", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]);
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
                if (g_le_hid_record.type != BtdrvBleEventType_ClientNotify) {
                    events::Log(g_le_hid_record);
                    continue;
                }

                // Input from any connected pad goes to its own controller; only the link being set up waits on
                // the results of requests
                if ((notify.type == GattNotifyType_Notification) && IsUuid16(notify.char_uuid, GattCharReport)) {
                    DeliverInputReport(notify.conn_id, notify.data, std::min<size_t>(notify.size, sizeof(notify.data)));
                } else if (notify.conn_id != events::GetConnection()) {
                    events::Log(g_le_hid_record);
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

        Result RegisterGattClient(u8 *out_client_if) {
            events::SetRegistering(true);
            ON_SCOPE_EXIT { events::SetRegistering(false); };

            Result rc = btdrvRegisterGattClient(&GattClientAppUuid);
            log::Write("btdrvRegisterGattClient rc=0x%x", rc.GetValue());
            R_TRY(rc);

            EventRecord e;
            if (!events::WaitFor(BtdrvBleEventType_ClientRegistration, TimeSpan::FromSeconds(5), &e, [](const EventRecord &r) { return r.info.client_registration.status == 1; })) {
                R_THROW(svc::ResultTimedOut());
            }

            *out_client_if = e.info.client_registration.client_if;
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
                if (R_SUCCEEDED(RegisterGattClient(&g_client_if))) {
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
                // Bluetooth switched off or the console going to sleep: the links are gone or about to be, and
                // nothing is asked of the radio until it is back
                if (const size_t links = LinkCount(); links > 0) {
                    log::Write("radio off, withdrawing %u controller(s)", static_cast<u32>(links));
                    u32 conn_ids[MaxLinks];
                    size_t count = 0;
                    {
                        std::scoped_lock lk(g_controller_lock);
                        for (auto &link : g_links) {
                            if (link.used) {
                                conn_ids[count++] = link.conn_id;
                            }
                        }
                    }
                    // The links were closed before the radio went off (see PrepareForSleep, OnBluetoothRadio)
                    for (size_t i = 0; i < count; ++i) {
                        static_cast<void>(TeardownLink(conn_ids[i], true, true));
                    }
                }
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

        /* 8BitDo pads, known by address before they connect: they are given a spare GATT client so that the
           first one, the only client on which an Xbox pad pairs (on any other the bond is refused within a second
           and the link never encrypts), stays free */

        constexpr const char EightBitDoDevicesPath[] = "sdmc:/config/MissionControl/ble_8bitdo_devices.bin";
        constexpr size_t MaxEightBitDoDevices = 8;
        constinit size_t g_eightbitdo_count = 0;
        constinit bluetooth::Address g_eightbitdo_devices[MaxEightBitDoDevices] = {};

        void LoadEightBitDoDevices() {
            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), EightBitDoDevicesPath, fs::OpenMode_Read))) {
                return;
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };

            size_t read = 0;
            if (R_SUCCEEDED(fs::ReadFile(std::addressof(read), file, 0, g_eightbitdo_devices, sizeof(g_eightbitdo_devices)))) {
                std::scoped_lock lk(g_autoconnect_lock);
                g_eightbitdo_count = read / sizeof(bluetooth::Address);
            }
        }

        bool IsKnownEightBitDo(const bluetooth::Address &address) {
            // 8BitDo's own address block (OUI E4:17:D8), for a pad seen before it was ever presented here
            if ((address.address[0] == 0xe4) && (address.address[1] == 0x17) && (address.address[2] == 0xd8)) {
                return true;
            }
            std::scoped_lock lk(g_autoconnect_lock);
            for (size_t i = 0; i < g_eightbitdo_count; ++i) {
                if (AddressEqual(g_eightbitdo_devices[i], address)) {
                    return true;
                }
            }
            return false;
        }

        void SaveEightBitDoDevice(const bluetooth::Address &address) {
            bluetooth::Address addresses[MaxEightBitDoDevices];
            size_t count;
            {
                std::scoped_lock lk(g_autoconnect_lock);
                for (size_t i = 0; i < g_eightbitdo_count; ++i) {
                    if (AddressEqual(g_eightbitdo_devices[i], address)) {
                        return;
                    }
                }
                // Most recent first; each pairing with the sync button gives the pad a new address
                size_t n = 0;
                addresses[n++] = address;
                for (size_t i = 0; (i < g_eightbitdo_count) && (n < MaxEightBitDoDevices); ++i) {
                    addresses[n++] = g_eightbitdo_devices[i];
                }
                std::memcpy(g_eightbitdo_devices, addresses, n * sizeof(bluetooth::Address));
                g_eightbitdo_count = count = n;
            }

            const size_t size = count * sizeof(bluetooth::Address);
            static_cast<void>(fs::CreateFile(EightBitDoDevicesPath, size));
            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), EightBitDoDevicesPath, fs::OpenMode_Write))) {
                return;
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };
            static_cast<void>(fs::SetFileSize(file, size));
            static_cast<void>(fs::WriteFile(file, 0, addresses, size, fs::WriteOption::Flush));
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
        // Scanning shares the radio with the pads already connected; look for another one less often
        constexpr TimeSpan BackgroundScanPauseLinked = TimeSpan::FromMilliSeconds(6000);

        // Covers one classic inquiry (10.24 s), during which the stack refuses connections
        constexpr TimeSpan ConnectRefusedRetryTime = TimeSpan::FromSeconds(12);

        // A remembered controller that is not connected yet
        bool HasUnlinkedRemembered() {
            bluetooth::Address remembered[MaxRememberedDevices];
            size_t count;
            {
                std::scoped_lock lk(g_autoconnect_lock);
                count = g_remembered_count;
                std::memcpy(remembered, g_remembered, sizeof(remembered));
            }
            for (size_t i = 0; i < count; ++i) {
                if (!IsLinked(remembered[i])) {
                    return true;
                }
            }
            return false;
        }

        bool BackgroundScanWanted() {
            return AutoConnectEnabled() && g_have_remembered && events::IsRadioEnabled() && !events::IsPairingWindowOpen() && HasUnlinkedRemembered();
        }

        // Set when PrepareForSleep dropped the link: that end of link is no reason to cycle the radio on wake
        constinit bool g_dropped_for_sleep = false;

        // Set when a bonded (Xbox) link ended while other controllers were connected: the cycle that lets that
        // pad pair again would drop them too (a DualShock/DualSense switches itself off when its link goes), so it
        // waits until the pad is back and about to pair
        constinit bool g_cycle_pending = false;

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

            // BLE links do not survive the cycle; their controllers are withdrawn first
            if (const size_t links = LinkCount(); links > 0) {
                log::Write("radio cycle drops %u Bluetooth LE controller(s)", static_cast<u32>(links));
                DropAllLinks();
            }

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
            g_extra_client_if = InvalidClientIf;
            g_cycle_pending = false;

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
                if (R_SUCCEEDED(RegisterGattClient(&g_client_if))) {
                    log::Write("radio cycle: registered gatt client_if=%u", g_client_if);
                    return;
                }
                os::SleepThread(TimeSpan::FromSeconds(2));
            }

            log::Write("radio cycle: no gatt client available, keeping client_if=%u", g_client_if);
        }

        /* Finding and connecting the controller */

        // With `only` set, nothing but that controller is accepted (background scans outside the pairing window)
        Result Scan(bluetooth::Address *out_address, bool *out_eightbitdo, TimeSpan duration, bool only_remembered = false) {
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
                    if (IsLinked(r.info.scan_result.address)) {
                        return false;
                    }
                    if (EightBitDoHeld() && IsKnownEightBitDo(r.info.scan_result.address)) {
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
            {
                AdvertisementSummary summary;
                ParseAdvertisements(e.info.scan_result.ad_list, e.info.scan_result.count, &summary);
                *out_eightbitdo = (std::strncmp(summary.name, "8BitDo", 6) == 0) || IsKnownEightBitDo(*out_address);
            }
            events::LogAddress(*out_eightbitdo ? "found controller (8BitDo)" : "found controller", *out_address);

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

        // Asks for a link on the first client that takes it: the main one for an Xbox pad, a spare one for an
        // 8BitDo pad (see LoadEightBitDoDevices); 1 and 2 refuse (0x300c71), btm's registrations from 3 up take it.
        Result RequestLink(const bluetooth::Address &address, bool eightbitdo, u8 *out_client_if) {
            u8 order[10];
            size_t n = 0;
            if (!eightbitdo) {
                order[n++] = g_client_if;
            }
            if (g_extra_client_if != InvalidClientIf) {
                order[n++] = g_extra_client_if;
            }
            for (u8 candidate = 3; candidate <= 8; ++candidate) {
                order[n++] = candidate;
            }
            if (eightbitdo) {
                order[n++] = g_client_if;
            }

            Result rc = ResultNoConnection();
            for (size_t i = 0; i < n; ++i) {
                const u8 client_if = order[i];
                if ((client_if == InvalidClientIf) || ClientIfInUse(client_if) || std::find(order, order + i, client_if) != order + i) {
                    continue;
                }
                rc = btdrvConnectGattServer(client_if, address, true, 0);
                log::Write("btdrvConnectGattServer client_if=%u rc=0x%x", client_if, rc.GetValue());
                if (R_SUCCEEDED(rc)) {
                    if (client_if != g_client_if) {
                        g_extra_client_if = client_if;
                    }
                    *out_client_if = client_if;
                    R_SUCCEED();
                }
            }
            R_RETURN(rc);
        }

        Result Connect(const bluetooth::Address &address, bool eightbitdo, u32 *out_conn_id, TimeSpan timeout) {
            events::SetTarget(&address);

            // A record left for this address, including the one a classic firmware left behind, is encrypted
            // with the moment the link comes up, and the controller drops the link over a key it doesn't hold
            Result rc = btdrvRemoveBond(address);
            log::Write("btdrvRemoveBond rc=0x%x", rc.GetValue());

            // While btm runs a classic inquiry (10.24 s, e.g. at boot or when the Controllers screen opens) the
            // stack turns a connection down at once (result 26). The pad keeps advertising for its reconnect, so
            // keep asking until the inquiry is over instead of letting the pad give up.
            const auto deadline = os::GetSystemTick().ToTimeSpan() + ConnectRefusedRetryTime;
            int refused = 0;
            u8 client_if = InvalidClientIf;
            for (;;) {
                if (client_if == InvalidClientIf) {
                    rc = RequestLink(address, eightbitdo, &client_if);
                } else {
                    rc = btdrvConnectGattServer(client_if, address, true, 0);
                }
                R_TRY(rc);
                g_setup_client_if = client_if;

                const auto asked = os::GetSystemTick().ToTimeSpan();
                rc = WaitForConnection(address, out_conn_id, timeout);
                if (R_SUCCEEDED(rc)) {
                    break;
                }
                static_cast<void>(btdrvCancelConnectGattServer(client_if, address, true));

                const auto now = os::GetSystemTick().ToTimeSpan();
                const bool refused_at_once = ResultNoConnection::Includes(rc) && (now - asked) < TimeSpan::FromSeconds(1);
                if (!refused_at_once || now >= deadline || !events::IsRadioEnabled()) {
                    break;
                }
                if (++refused == 1) {
                    log::Write("connection refused, retrying while the radio is busy");
                }
                events::Pump(TimeSpan::FromMilliSeconds(500));
            }
            if (refused > 0) {
                log::Write("%s after %d refused connection attempts", R_SUCCEEDED(rc) ? "connected" : "gave up", refused);
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
            g_setup_client_if = g_client_if;
            log::Write("controller connected on its own, conn_id=%u", *out_conn_id);
            return true;
        }

        constexpr s64 BondRefusedWithinMs = 2000;

        // The stack never pairs on its own: an authenticated request on an unbonded link just fails. Pairing is
        // Just Works and needs no confirmation from the controller.
        bool Pair(const bluetooth::Address &address) {
            const auto asked = os::GetSystemTick();
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
                    // A bond takes about 5.7 s; the same event within a second is the stack turning the pairing
                    // down (the link then never encrypts and every authenticated request fails)
                    const s64 elapsed_ms = (os::GetSystemTick() - asked).ToTimeSpan().GetMilliSeconds();
                    if (elapsed_ms < BondRefusedWithinMs) {
                        log::Write("bond refused, reported after %ld ms", elapsed_ms);
                        break;
                    }
                    if (!events::IsDisconnected()) {
                        log::Write("paired, bond reported after %ld ms", elapsed_ms);
                        return true;
                    }
                    break;
                }
            }
            log::Write("pairing failed");
            return false;
        }

        /* Presenting the controller to the system */

        // Hands the link being set up over to the established links. Returns false when there is no free slot.
        bool Present(u32 conn_id, const bluetooth::Address &address, const gatt::HidService &hid, bool paired, bool managed) {
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
                Link *slot = nullptr;
                for (auto &link : g_links) {
                    if (!link.used) {
                        slot = &link;
                        break;
                    }
                }
                if (!slot) {
                    log::Write("no free controller slot");
                    return false;
                }
                slot->used = true;
                slot->conn_id = conn_id;
                slot->client_if = g_setup_client_if;
                slot->address = address;
                slot->kind = g_controller_kind;
                slot->paired = paired;
                slot->managed = managed;
                slot->controller = ble_controller;
                slot->switch_controller = controller;
                slot->present_tick = os::GetSystemTick().GetInt64Value();
                ScheduleNeutralReports(*slot);
            }

            // Watched as an established link from here on; the link's end is no longer the setup's
            events::AddLink(address, conn_id);
            const bool lost = events::IsDisconnected();
            events::SetConnection(InvalidConnId);
            events::SetTarget(nullptr);
            events::SetCountManagedGattResults(false);

            controller::AttachVirtualController(controller);
            g_pairing_failed_tick = 0;
            SaveRememberedDevice(address);
            if (g_controller_kind == ControllerKind_EightBitDo) {
                SaveEightBitDoDevice(address);
            } else {
                g_eightbitdo_held_until = 0;
            }

            // The pad only reports when something is touched, so its resting state (sticks centred, nothing
            // pressed) is delivered for it, now and again over the next seconds, see ScheduleNeutralReports
            ServiceNeutralReports();

            log::Write("controller presented to the system, %u connected", static_cast<u32>(LinkCount()));

            // Dropped while being handed over: let the next service pass take it down
            if (lost) {
                static_cast<void>(TeardownLink(conn_id, true));
            }
            return true;
        }

        // Abandons the link being set up
        void Disconnect() {
            const u32 conn_id = events::GetConnection();

            bluetooth::Address address;
            const bool have_target = events::GetTarget(&address);

            // Nothing may touch the radio while the system has BLE switched off: it wakes the console
            if ((conn_id != InvalidConnId) && !events::IsDisconnected() && events::IsRadioEnabled()) {
                static_cast<void>(btdrvDisconnectGattServer(conn_id));
            }

            // Keep the target until the stack has finished tearing the link down
            events::Pump(TimeSpan::FromSeconds(1));

            events::SetConnection(InvalidConnId);
            events::SetTarget(nullptr);
            if (have_target) {
                events::RemoveClaimedAddress(address);
            }
            events::SetCountManagedGattResults(false);
            ReleaseManagedPath();
        }

        // Neutral reports for new controllers, and the ends of established links
        void ServiceLinks() {
            ServiceNeutralReports();

            u32 conn_id;
            u16 reason;
            while (events::TakeLinkDown(&conn_id, &reason)) {
                log::Write("controller disconnected conn_id=%u reason=0x%04x", conn_id, reason);
                const bool paired = TeardownLink(conn_id, true);

                // A radio cycle is the one thing that lets a bonded pad pair again (skipped when the link ended
                // because the console is going to sleep). An 8BitDo pad never bonds and needs none.
                if (!paired) {
                    continue;
                }
                if (g_dropped_for_sleep) {
                    log::Write("link was dropped for sleep, no radio cycle");
                } else if (controller::GetControllerCount() > 0) {
                    g_cycle_pending = true;
                    log::Write("radio cycle put off: %u other controller(s) connected", static_cast<u32>(controller::GetControllerCount()));
                } else {
                    RecycleRadio();
                }
            }

            if (LinkCount() == 0) {
                g_dropped_for_sleep = false;
            }
        }

        // Waits while keeping the established links serviced
        void Idle(TimeSpan duration) {
            const auto deadline = os::GetSystemTick().ToTimeSpan() + duration;
            for (;;) {
                ServiceLinks();
                const auto now = os::GetSystemTick().ToTimeSpan();
                if (now >= deadline) {
                    return;
                }
                events::Pump(std::min(deadline - now, TimeSpan::FromMilliSeconds(250)));
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
            events::SetInputSink(DeliverInputReport);
            LoadRememberedDevice();
            LoadEightBitDoDevices();

            for (;;) {
                WaitForRadio();
                ServiceLinks();

                if (LinkCount() >= MaxLinks) {
                    Idle(TimeSpan::FromMilliSeconds(500));
                    continue;
                }

                bluetooth::Address address;
                u32 conn_id = InvalidConnId;

                bool connected = Adopt(&address, &conn_id);

                // Any gamepad in pairing mode while the Controllers screen is open; only remembered ones that are
                // not connected yet, in short bursts, the rest of the time
                const bool classic_busy = events::ClassicActivityWithin(ClassicActivityHoldOff);
                const bool in_window = !classic_busy && events::IsPairingWindowOpen() && events::IsRadioEnabled() && !PairingBackoffActive();
                const bool background = !classic_busy && !in_window && !PairingBackoffActive() && BackgroundScanWanted();
                if (!connected && (in_window || background)) {
                    bool eightbitdo = false;
                    Result rc = in_window ? Scan(&address, &eightbitdo, TimeSpan::FromSeconds(10)) : Scan(&address, &eightbitdo, BackgroundScanDuration, true);
                    if (Adopt(&address, &conn_id)) {
                        connected = true;
                    } else if (R_SUCCEEDED(rc)) {
                        if (background) {
                            log::Write("remembered controller is in pairing mode, connecting");
                        }
                        connected = R_SUCCEEDED(Connect(address, eightbitdo, &conn_id, TimeSpan::FromSeconds(15)));
                        if (!connected) {
                            Disconnect();
                        }
                    }
                }

                if (!connected) {
                    Idle(!background ? TimeSpan::FromSeconds(2) : (LinkCount() > 0 ? BackgroundScanPauseLinked : BackgroundScanPause));
                    continue;
                }

                // Pairing traffic for this pad is answered on its behalf from here on
                events::AddClaimedAddress(address);

                gatt::Discover(conn_id, address);

                // A link that drops as soon as it comes up (0x3e) leaves nothing to tell the pad by; just try again
                if (events::IsDisconnected()) {
                    log::Write("link lost during discovery reason=0x%04x", events::GetDisconnectReason());
                    Disconnect();
                    Idle(TimeSpan::FromSeconds(1));
                    continue;
                }

                // 8BitDo pads carry a vendor service of their own next to the HID one
                g_controller_kind = gatt::HasService128(EightBitDoVendorService) ? ControllerKind_EightBitDo : ControllerKind_Xbox;
                log::Write("%s controller", (g_controller_kind == ControllerKind_EightBitDo) ? "8BitDo" : "Xbox");

                // EXPERIMENT. An 8BitDo pad answers the console's LE legacy pairing at once and then serves nothing
                // that needs encryption; see whether it serves its HID service without, before pairing at all
                bool paired = false;
                if (g_controller_kind != ControllerKind_EightBitDo) {
                    if (g_cycle_pending) {
                        g_cycle_pending = false;
                        log::Write("radio cycle deferred from the last link, doing it before pairing");
                        HoldEightBitDo();
                        Disconnect();
                        RecycleRadio();
                        Idle(TimeSpan::FromSeconds(3));
                        continue;
                    }
                    // The stack runs one LE pairing at a time, and an 8BitDo pad starts one it never finishes: with
                    // an 8BitDo pad connected every Xbox bond was refused within 0.7 s (cancelling the 8BitDo bond
                    // changed nothing), and a refused bond leaves the stack refusing the next ones until it is
                    // restarted. So no attempt is made: the stack is restarted at once and the Xbox pad, still in
                    // pairing mode, pairs before the 8BitDo pad is let back in.
                    bluetooth::Address eightbitdo_addresses[MaxLinks];
                    u32 eightbitdo_conn_ids[MaxLinks];
                    if (GetEightBitDoLinks(eightbitdo_addresses, eightbitdo_conn_ids) > 0) {
                        log::Write("8BitDo pad connected: restarting the stack so the Xbox pad can pair first");
                        HoldEightBitDo();
                        Disconnect();
                        RecycleRadio();
                        Idle(TimeSpan::FromSeconds(3));
                        continue;
                    }

                    const bool bonded = Pair(address);
                    if (!bonded) {
                        g_pairing_failed_tick = os::GetSystemTick().GetInt64Value();
                        log::Write("backing off for %lds before trying to pair again", PairingRetryBackoff.GetSeconds());
                        Disconnect();
                        // A cycle clears the backoff: the next attempt is against a fresh stack. A connection asked for
                        // within a couple of seconds of the cycle fails to be established (0x3e), so let it settle.
                        RecycleRadio();
                        Idle(TimeSpan::FromSeconds(3));
                        continue;
                    }
                    paired = true;
                }

                // Hands the HID service to the stack's HID host layer for this link: its notifications, and the results
                // of our requests on it, then arrive on the LE HID event stream. That layer secures the link itself,
                // and an 8BitDo pad never completes the pairing it starts: the stack drops the link when it times
                // out 30 s later. Its service is used directly instead, over the managed BLE event stream.
                // EXPERIMENT. Both registrations are by service UUID: with an 8BitDo and an Xbox pad connected at
                // once, input is routed by connection to whichever pad it belongs to on either stream.
                const bool managed = (g_controller_kind == ControllerKind_EightBitDo) && g_eightbitdo_managed;
                // Results come back on whichever stream the service's registration routes them to
                events::SetCountManagedGattResults(true);
                auto hid_uuid = MakeUuid16(GattServiceHid);
                Result rc;
                if (managed) {
                    // Without a data path the stack reports nothing about the service at all
                    if (!g_managed_path) {
                        rc = btdrvRegisterGattManagedDataPath(&hid_uuid);
                        log::Write("btdrvRegisterGattManagedDataPath rc=0x%x", rc.GetValue());
                        g_managed_path = R_SUCCEEDED(rc);
                    }
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
                    if ((g_controller_kind == ControllerKind_EightBitDo) && !managed) {
                        g_eightbitdo_managed = true;
                        log::Write("8BitDo pad goes back to the managed data path");
                    }
                    Disconnect();
                    Idle(TimeSpan::FromSeconds(1));
                    continue;
                }

                if (!Present(conn_id, address, hid, paired, managed)) {
                    Disconnect();
                }
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
        // Controllers used to be withdrawn while the Controllers screen was open, as the screen does with every
        // controller. The screen is only inferred from btm's inquiry and was seen to withdraw a controller again
        // right after LB+RB brought it back; it read as the controller dropping off the console. Left connected.
        AMS_UNUSED(open);
    }

    // EXPERIMENT. A controller left connected wakes the console a few seconds after it goes to sleep. The link
    // is dropped here, from the power module thread, while the bluetooth module is still awake (it is listed as
    // a dependency of ours), so that nothing is left on the air by the time the radio goes down. The host thread
    // sees the radio marked unavailable and stays off it until FullAwake.
    namespace {

        // Drops every link while the radio can still be used; the host thread stays off the radio until it is
        // marked usable again
        void DropLinksBeforeRadioOff() {
            u32 link_ids[MaxLinks];
            size_t link_count = 0;
            {
                std::scoped_lock lk(g_controller_lock);
                for (auto &link : g_links) {
                    if (link.used) {
                        link_ids[link_count++] = link.conn_id;
                    }
                }
            }
            u32 conn_ids[MaxLinks + 1];
            size_t count = 0;
            for (size_t i = 0; i < link_count; ++i) {
                conn_ids[count++] = link_ids[i];
            }
            if (const u32 setup = events::GetConnection(); (setup != InvalidConnId) && !events::IsDisconnected()) {
                conn_ids[count++] = setup;
            }
            if (count == 0) {
                log::Write("radio going off, no link to drop");
                return;
            }

            for (size_t i = 0; i < count; ++i) {
                Result rc = btdrvDisconnectGattServer(conn_ids[i]);
                log::Write("radio going off: btdrvDisconnectGattServer conn_id=%u rc=0x%x", conn_ids[i], rc.GetValue());
            }

            // The disconnections are reported on btdrv's event thread, independently of the host thread
            for (int i = 0; (i < 30) && (events::CountLinksUp() > 0); ++i) {
                os::SleepThread(TimeSpan::FromMilliSeconds(50));
            }
            log::Write((events::CountLinksUp() == 0) ? "links down" : "a link is still up as the radio goes off");
        }

    }

    void PrepareForSleep() {
        const bool was_enabled = events::IsRadioEnabled();
        events::SetRadioEnabled(false);

        // MinimumAwake and SleepReady both come through here; only the first one has anything to do
        if (!g_launched || !was_enabled) {
            return;
        }

        g_dropped_for_sleep = true;
        DropLinksBeforeRadioOff();
    }

    // Bluetooth switched off in the settings only stops classic Bluetooth: the BLE links went on working. Called
    // from btm's request before it is passed on: the links are closed while the radio still runs (a disconnection
    // asked for once it is off only happens when it comes back), which only waits on the bluetooth module. The
    // controllers are withdrawn by the host thread (see WaitForRadio): withdrawing one signals btm, which is
    // waiting on this very request, and doing it here hung the console.
    void OnBluetoothRadio(bool on) {
        if (!g_launched) {
            return;
        }
        if (!on && events::IsRadioEnabled()) {
            log::Write("bluetooth switched off in the settings, closing the links");
            DropLinksBeforeRadioOff();
        }
        events::SetBluetoothOn(on);
    }

}
