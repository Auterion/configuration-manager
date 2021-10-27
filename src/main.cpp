/****************************************************************************
 *
 *   Copyright (c) 2020-2021 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Auterion nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <configuration_manager.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <software_interface.h>
#include <software_interfaces/autopilot_manager_config.h>

#include <future>
#include <iostream>
#include <vector>

// These transaction ids are used to identity transaction message from AMC
// This ID should be used to identity a transaction of any type but for the
// moment we use it to identity the transaction type
constexpr uint8_t APMNG_TRANSACTION_ID = 126;
constexpr uint8_t GROUND_STATION_SYS_ID = 255;

using namespace mavsdk;
using namespace configuration_manager;
using namespace software_interface;
using namespace dbus_interface;
using namespace autopilot_manager_config;

std::shared_ptr<mavsdk::System> discover_ground_station(Mavsdk &mavsdk);

int main(int /* argc */, char ** /* argv */) {
    std::cout << "[configuration-manager] Starting..." << std::endl;

    // MAVSDK: for all mavlink communication with the ground station
    Mavsdk mavsdk;
    mavsdk.set_configuration(Mavsdk::Configuration(Mavsdk::Configuration::UsageType::CompanionComputer));

    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14542");
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "[configuration-manager] Error setting up UDP connection!" << std::endl;
        return 1;
    }

    auto system = discover_ground_station(mavsdk);
    std::shared_ptr<MavlinkPassthrough> mavlink_passthrough = std::make_shared<MavlinkPassthrough>(system);
    std::shared_ptr<DBusInterface> dbusInterface = std::make_shared<DBusInterface>();

    // Register software interfaces
    auto config_manager = std::make_shared<ConfigurationManager>(
        mavlink_passthrough, "/etc/configuration-manager/configuration-manager.conf");
    if (config_manager->getConfig().autopilot_manager_sw_interface_enabled) {
        config_manager->addSoftwareInterface(APMNG_TRANSACTION_ID,
                                             new AutopilotManagerConfig(config_manager, dbusInterface));
    }

    std::cout << "[configuration-manager] Run main loop" << std::endl;
    while (true) {
        config_manager->run();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    dbusInterface->release_dbus_connection();
    return 0;
}

std::shared_ptr<System> discover_ground_station(Mavsdk &mavsdk) {
    auto new_system_promise = std::promise<std::shared_ptr<System>>{};
    auto new_system_future = new_system_promise.get_future();

    std::cout << "[configuration-manager] Waiting to discover system..." << std::endl;

    // We wait for new systems to be discovered, and use the one that does not have an autopilot
    mavsdk.subscribe_on_new_system([&new_system_promise, &mavsdk]() {
        const auto systems = mavsdk.systems();
        auto system_it = std::find_if(systems.cbegin(), systems.cend(),
                                      [&](const auto &sys) { return sys->get_system_id() == GROUND_STATION_SYS_ID; });
        if (system_it != systems.cend()) {
            std::cout << "[configuration-manager] Discovered a ground station!\n";
            new_system_promise.set_value(*system_it);
            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
        }
    });

    return new_system_future.get();
}
