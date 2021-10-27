/****************************************************************************
 *
 *   Copyright (c) 2021 Auterion AG. All rights reserved.
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
#include <dbus_interface.h>
#include <software_interface.h>

#include <chrono>
#include <iostream>

using namespace software_interface;
using namespace dbus_interface;

namespace autopilot_manager_config {
enum DecisionMakerInputType : uint8_t { None = 0, SimpleCollisionAvoidance = 1, TargetDetection = 2, Other = 3 };

enum DecisionMakerOutputActions : uint8_t {
    KeepState = 0,
    Hold = 1,
    RTL = 2,
    GoToWaypoint = 3,
    IncreaseHorVel = 4,
    DecreaseHorVel = 5,
    Climb = 6,
    Descend = 7,
    Takeoff = 8,
    Land = 9,
    Custom = 10
};

class AutopilotManagerConfigSet : public Arguments {
   public:
    bool appendArguments(DBusMessageIter *iter) const;
    uint8_t autopilot_manager_enabled;
    DecisionMakerInputType decision_maker_input_type;
    uint8_t simple_collision_avoid_enabled;
    double simple_collision_avoid_distance_threshold;
    DecisionMakerOutputActions simple_collision_avoid_distance_on_condition_true;
    DecisionMakerOutputActions simple_collision_avoid_distance_on_condition_false;

   private:
    const char *appendSimpleCollAvoidOutputAction(const DecisionMakerOutputActions &action) const;
};

class AutopilotManagerConfigGet : public Response {
   public:
    bool parseMessage(DBusMessage *);
    uint8_t autopilot_manager_enabled = false;
    DecisionMakerInputType decision_maker_input_type = None;
    uint8_t simple_collision_avoid_enabled = false;
    double simple_collision_avoid_distance_threshold = 0.0;
    DecisionMakerOutputActions simple_collision_avoid_distance_on_condition_true = KeepState;
    DecisionMakerOutputActions simple_collision_avoid_distance_on_condition_false = KeepState;
    uint32_t response_code = 999;

   private:
    void parseSimpleCollAvoidOutputAction(DecisionMakerOutputActions &action, const std::string &cond_str);
};

class AutopilotManagerConfig : public SoftwareInterface {
   public:
    AutopilotManagerConfig(const std::shared_ptr<ConfigurationManager> &configManager,
                           const std::shared_ptr<DBusInterface> &dbusInterface);

   private:
    bool handleCommit();
    bool handleNotReady();
    bool handleStart();
    bool handleError();

    void setSimpleCollAvoidOutputActionArg(const MAVParameter &param, DecisionMakerOutputActions &action_arg);

    std::chrono::steady_clock::time_point _start_not_ready = std::chrono::steady_clock::now();
    const char *BUS_NAME = "com.auterion.autopilot_manager";
    const char *OBJECT_PATH = "/com/auterion/autopilot_manager/interface";
    const char *INTERFACE_NAME = "com.auterion.autopilot_manager.interface";
    const char *METHOD_GET_CONFIG = "get_config";
    const char *METHOD_SET_CONFIG = "set_config";
};
}  // namespace autopilot_manager_config
