/****************************************************************************
 *
 *   Copyright (c) 2020 Auterion AG. All rights reserved.
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
#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <software_interface.h>

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>
#include <vector>

using namespace mavsdk;
namespace software_interface {
class SoftwareInterface;
}

using namespace software_interface;

namespace configuration_manager {

struct ConfigurationManagerConfig {
    bool ap_config_sw_interface_enabled;
    bool autopilot_manager_sw_interface_enabled;
    bool cot_sw_interface_enabled;
    bool modem_config_sw_interface_enabled;
};

class ConfigurationManager {
   public:
    ConfigurationManager(std::shared_ptr<MavlinkPassthrough> &mavlinkPassthrough, const std::string &config_path);
    void addSoftwareInterface(uint16_t id, SoftwareInterface *interface);
    void sendMessage(mavlink_message_t &msg) const;
    uint8_t getSysId() const;
    uint8_t getCompId() const;
    void run();

    const ConfigurationManagerConfig getConfig() const { return _config; }

   private:
    void initMavlinkHandlers();
    void handleParamExtRequestList(const mavlink_message_t &mavlink_message);
    void handleParamExtRequestRead(const mavlink_message_t &mavlink_message);
    void validParamExtSetMavlinkTarget(const mavlink_message_t &mavlink_message);
    void handleCommandLong(const mavlink_message_t &mavlink_message);
    bool loadConfigFromFile(const std::string &config_path);
    int getParamCount() const;

    std::thread _heartbeatThread;
    std::map<uint16_t, SoftwareInterface *> interfaces;
    std::shared_ptr<MavlinkPassthrough> _mavlinkPassthrough;

    ConfigurationManagerConfig _config{.ap_config_sw_interface_enabled = true,
                                       .autopilot_manager_sw_interface_enabled = true,
                                       .cot_sw_interface_enabled = true,
                                       .modem_config_sw_interface_enabled = true};
};
}  // namespace configuration_manager
