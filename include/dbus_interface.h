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

#include <dbus/dbus.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace dbus_interface {

// TO DO: check that all these addresses/paths are correct
const auto BUS_NAME = "com.auterion.configuration_manager";
const auto TARGET_BUS_NAME = "com.auterion.modem_configure";
const auto TARGET_OBJECT_PATH = "/com/auterion/modem_configure/interface";
const auto INTERFACE_NAME = "com.auterion.modem_configure.interface";
const auto METHOD_GET_CONFIG = "get_config";
const auto METHOD_SET_CONFIG = "set_config";
const auto METHOD_GET_STATUS = "get_status";

struct ConfigurationWrite {
    std::string apn;
    std::string pin;
    std::string puk;
    bool roaming;
    bool pin_enabled;
};

struct ConfigurationRead {
    std::string apn;
    bool roaming;
    uint32_t pin_tentative_left;
    uint32_t puk_tentative_left;
    bool pin_enabled;
    uint32_t response_code;
};

class Arguments {
   public:
    virtual bool appendArguments(DBusMessageIter *) const = 0;
};

class Response {
   public:
    virtual bool parseMessage(DBusMessage *) = 0;
};

class DBusInterface {
   public:
    DBusInterface();
    ~DBusInterface() = default;

    bool prepare_dbus_connection();
    bool send_message(const char *bus_name, const char *object_path, const char *interface_name, const char *method,
                      Arguments *args, Response *response);
    std::optional<ConfigurationRead> fetch_configuration();
    std::optional<ConfigurationRead> set_configuration(ConfigurationWrite config);
    bool release_dbus_connection();

   private:
    DBusConnection *_dbus_connection = nullptr;
    DBusError _dbus_error;
};

}  // namespace dbus_interface
