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
#include <software_interfaces/autopilot_manager_config.h>

using namespace autopilot_manager_config;

const char *AutopilotManagerConfigSet::appendSimpleCollAvoidOutputAction(
    const DecisionMakerOutputActions &action) const {
    const char *cond_str;
    switch (action) {
        case KeepState:
            cond_str = "KEEP_STATE";
            break;
        case Hold:
            cond_str = "HOLD";
            break;
        case RTL:
            cond_str = "RTL";
            break;
        case GoToWaypoint:
            cond_str = "GO_TO_WAYPOINT";
            break;
        case IncreaseHorVel:
            cond_str = "INCREASE_HOR_VEL";
            break;
        case DecreaseHorVel:
            cond_str = "DECREASE_HOR_VEL";
            break;
        case Climb:
            cond_str = "CLIMB";
            break;
        case Descend:
            cond_str = "DESCEND";
            break;
        case Takeoff:
            cond_str = "TAKEOFF";
            break;
        case Land:
            cond_str = "LAND";
            break;
        case Custom:
            cond_str = "CUSTOM";
            break;
        default:
            cond_str = "KEEP_STATE";
            break;
    }
    return cond_str;
}

bool AutopilotManagerConfigSet::appendArguments(DBusMessageIter *iter) const {
    std::cout << "[AutopilotManagerConfig] Append argument to message" << std::endl;

    std::cout << "\tappending autopilot_manager_enabled... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_UINT32, &autopilot_manager_enabled)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'autopilot_manager_enabled'"
                  << std::endl;
        return false;
    }

    const char *input_type_str;
    switch (decision_maker_input_type) {
        case None:
            input_type_str = "NONE";
            break;
        case SimpleCollisionAvoidance:
            input_type_str = "SIMPLE_COLLISION_AVOIDANCE";
            break;
        case TargetDetection:
            input_type_str = "TARGET_DETECTION";
            break;
        case Other:
            input_type_str = "OTHER";
            break;
        default:
            input_type_str = "NONE";
            break;
    }
    std::cout << "\tappending decision_maker_input_type... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_STRING, &input_type_str)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'decision_maker_input_type'"
                  << std::endl;
        return false;
    }

    std::cout << "\tappending simple_collision_avoid_enabled... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_UINT32, &simple_collision_avoid_enabled)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'simple_collision_avoid_enabled'"
                  << std::endl;
        return false;
    }

    std::cout << "\tappending simple_collision_avoid_distance_threshold... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_DOUBLE, &simple_collision_avoid_distance_threshold)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'simple_collision_avoid_distance_threshold'"
                  << std::endl;
        return false;
    }

    const char *coll_avoid_on_true_str =
        appendSimpleCollAvoidOutputAction(simple_collision_avoid_distance_on_condition_true);
    std::cout << "\tappending simple_collision_avoid_distance_on_condition_true... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_STRING, &coll_avoid_on_true_str)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'simple_collision_avoid_distance_on_condition_true'"
                  << std::endl;
        return false;
    }

    const char *coll_avoid_on_false_str =
        appendSimpleCollAvoidOutputAction(simple_collision_avoid_distance_on_condition_false);
    std::cout << "\tappending simple_collision_avoid_distance_on_condition_false... ";
    if (dbus_message_iter_append_basic(iter, DBUS_TYPE_STRING, &coll_avoid_on_false_str)) {
        std::cout << "Done!" << std::endl;
    } else {
        std::cout << "Failed!" << std::endl;
        std::cerr << "[AutopilotManagerConfig] Error: failed to append argument "
                     "'simple_collision_avoid_distance_on_condition_false'"
                  << std::endl;
        return false;
    }

    return true;
}

void AutopilotManagerConfigGet::parseSimpleCollAvoidOutputAction(DecisionMakerOutputActions &action,
                                                                 const std::string &cond_str) {
    if (cond_str == "KEEP_STATE") {
        action = KeepState;
    } else if (cond_str == "HOLD") {
        action = Hold;
    } else if (cond_str == "RTL") {
        action = RTL;
    } else if (cond_str == "GO_TO_WAYPOINT") {
        action = GoToWaypoint;
    } else if (cond_str == "INCREASE_HOR_VEL") {
        action = IncreaseHorVel;
    } else if (cond_str == "DECREASE_HOR_VEL") {
        action = DecreaseHorVel;
    } else if (cond_str == "CLIMB") {
        action = Climb;
    } else if (cond_str == "DESCEND") {
        action = Descend;
    } else if (cond_str == "TAKEOFF") {
        action = Takeoff;
    } else if (cond_str == "LAND") {
        action = Land;
    } else if (cond_str == "CUSTOM") {
        action = Custom;
    }
}

bool AutopilotManagerConfigGet::parseMessage(DBusMessage *reply) {
    DBusError error;
    dbus_error_init(&error);
    char *returnedDecisionMakerType = nullptr;
    char *returnedCollAvoidOnCondTrue = nullptr;
    char *returnedCollAvoidOnCondFalse = nullptr;
    std::cout << "[AutopilotManagerConfig] Parse response..." << std::endl;
    dbus_message_get_args(reply, &error, DBUS_TYPE_UINT32, &autopilot_manager_enabled, DBUS_TYPE_STRING,
                          &returnedDecisionMakerType, DBUS_TYPE_UINT32, &simple_collision_avoid_enabled,
                          DBUS_TYPE_DOUBLE, &simple_collision_avoid_distance_threshold, DBUS_TYPE_STRING,
                          &returnedCollAvoidOnCondTrue, DBUS_TYPE_STRING, &returnedCollAvoidOnCondFalse,
                          DBUS_TYPE_UINT32, &response_code, DBUS_TYPE_INVALID);
    if (dbus_error_is_set(&error)) {
        std::cerr << "[AutopilotManagerConfig] Failed getting response arguments: " << error.message << std::endl;
        dbus_error_free(&error);
        return false;
    }

    // decision_maker_input_type response
    const std::string type_str(returnedDecisionMakerType);
    if (type_str == "NONE") {
        decision_maker_input_type = None;
    } else if (type_str == "SIMPLE_COLLISION_AVOIDANCE") {
        decision_maker_input_type = SimpleCollisionAvoidance;
    } else if (type_str == "TARGET_DETECTION") {
        decision_maker_input_type = TargetDetection;
    } else if (type_str == "OTHER") {
        decision_maker_input_type = Other;
    }

    // simple_collision_avoid_distance_on_condition_true response
    const std::string cond_true_str(returnedCollAvoidOnCondTrue);
    parseSimpleCollAvoidOutputAction(simple_collision_avoid_distance_on_condition_true, cond_true_str);

    // simple_collision_avoid_distance_on_condition_false response
    const std::string cond_false_str(returnedCollAvoidOnCondFalse);
    parseSimpleCollAvoidOutputAction(simple_collision_avoid_distance_on_condition_false, cond_false_str);

    std::cout << "[AutopilotManagerConfig] Received: [" << std::endl;
    std::cout << "    autopilot_manager_enabled: " << std::boolalpha << std::to_string(autopilot_manager_enabled)
              << std::endl;
    std::cout << "    decision_maker_input_type: " << decision_maker_input_type << std::endl;
    std::cout << "    simple_collision_avoid_enabled: " << std::boolalpha
              << std::to_string(simple_collision_avoid_enabled) << std::endl;
    std::cout << "    simple_collision_avoid_distance_threshold: " << simple_collision_avoid_distance_threshold
              << std::endl;
    std::cout << "    simple_collision_avoid_distance_on_condition_true: "
              << simple_collision_avoid_distance_on_condition_true << std::endl;
    std::cout << "    simple_collision_avoid_distance_on_condition_false: "
              << simple_collision_avoid_distance_on_condition_false << std::endl;
    std::cout << "    response_code: " << response_code << std::endl;
    std::cout << "]" << std::endl;
    return true;
}

AutopilotManagerConfig::AutopilotManagerConfig(const std::shared_ptr<ConfigurationManager> &configManager,
                                               const std::shared_ptr<DBusInterface> &dbusInterface)
    : SoftwareInterface("AutopilotManagerConfig", configManager, dbusInterface) {
    std::cout << "[AutopilotManagerConfig] Interface initialized!" << std::endl;
    _mavSettings.addParameter(MAVParameter("APMNG_ENABLED", MAV_PARAM_EXT_TYPE_UINT8, 0));
    _mavSettings.addParameter(MAVParameter("APMNG_DEC_IN_TP", MAV_PARAM_EXT_TYPE_UINT8, 1));
    _mavSettings.addParameter(MAVParameter("APMNG_AVD_EN", MAV_PARAM_EXT_TYPE_UINT8, 2));
    _mavSettings.addParameter(MAVParameter("APMNG_AVD_THRSH", MAV_PARAM_EXT_TYPE_REAL64, 3));
    _mavSettings.addParameter(MAVParameter("APMNG_AVD_TRUE", MAV_PARAM_EXT_TYPE_UINT8, 4));
    _mavSettings.addParameter(MAVParameter("APMNG_AVD_FALSE", MAV_PARAM_EXT_TYPE_UINT8, 5));
    _mavSettings.addParameter(MAVParameter("APMNG_STATUS", MAV_PARAM_EXT_TYPE_UINT32, 6));
}

void AutopilotManagerConfig::setSimpleCollAvoidOutputActionArg(const MAVParameter &param,
                                                               DecisionMakerOutputActions &action_arg) {
    switch (param.value.param_uint8) {
        case 0:
            action_arg = KeepState;
            break;
        case 1:
            action_arg = Hold;
            break;
        case 2:
            action_arg = RTL;
            break;
        case 3:
            action_arg = GoToWaypoint;
            break;
        case 4:
            action_arg = IncreaseHorVel;
            break;
        case 5:
            action_arg = DecreaseHorVel;
            break;
        case 6:
            action_arg = Climb;
            break;
        case 7:
            action_arg = Descend;
            break;
        case 8:
            action_arg = Takeoff;
            break;
        case 9:
            action_arg = Land;
            break;
        case 10:
            action_arg = Custom;
            break;
        default:
            action_arg = KeepState;
            break;
    }
}

bool AutopilotManagerConfig::handleCommit() {
    MAVParameter param{};
    AutopilotManagerConfigSet arguments{};
    AutopilotManagerConfigGet response{};

    // **** Get parameters **** //
    _mavSettings.getParameterById("APMNG_ENABLED", param);
    arguments.autopilot_manager_enabled = param.value.param_uint8;

    _mavSettings.getParameterById("APMNG_DEC_IN_TP", param);
    switch (param.value.param_uint8) {
        case 0:
            arguments.decision_maker_input_type = None;
            break;
        case 1:
            arguments.decision_maker_input_type = SimpleCollisionAvoidance;
            break;
        case 2:
            arguments.decision_maker_input_type = TargetDetection;
            break;
        default:
            arguments.decision_maker_input_type = None;
            break;
    }

    _mavSettings.getParameterById("APMNG_AVD_EN", param);
    arguments.simple_collision_avoid_enabled = param.value.param_uint8;

    _mavSettings.getParameterById("APMNG_AVD_THRSH", param);
    arguments.simple_collision_avoid_distance_threshold = param.value.param_double;

    _mavSettings.getParameterById("APMNG_AVD_TRUE", param);
    setSimpleCollAvoidOutputActionArg(param, arguments.simple_collision_avoid_distance_on_condition_true);

    _mavSettings.getParameterById("APMNG_AVD_FALSE", param);
    setSimpleCollAvoidOutputActionArg(param, arguments.simple_collision_avoid_distance_on_condition_false);

    if (!_dbusInterface->send_message(BUS_NAME, OBJECT_PATH, INTERFACE_NAME, METHOD_SET_CONFIG, &arguments,
                                      &response)) {
        _mavSettings.setParameterById("APMNG_STATUS", 2);  // FAILED
        return false;
    }

    _mavSettings.setParameterById("APMNG_ENABLED", static_cast<uint8_t>(response.autopilot_manager_enabled));
    _mavSettings.setParameterById("APMNG_DEC_IN_TP", static_cast<uint8_t>(response.decision_maker_input_type));
    _mavSettings.setParameterById("APMNG_AVD_EN", static_cast<uint8_t>(response.simple_collision_avoid_enabled));
    _mavSettings.setParameterById("APMNG_AVD_THRSH",
                                  static_cast<double>(response.simple_collision_avoid_distance_threshold));
    _mavSettings.setParameterById("APMNG_AVD_TRUE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_true));
    _mavSettings.setParameterById("APMNG_AVD_FALSE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_false));
    _mavSettings.setParameterById("APMNG_STATUS", static_cast<int>(response.response_code));

    return response.response_code != 2;  // not FAILED
}

bool AutopilotManagerConfig::handleNotReady() {
    AutopilotManagerConfigGet response{};

    if (!_dbusInterface->send_message(BUS_NAME, OBJECT_PATH, INTERFACE_NAME, METHOD_GET_CONFIG, nullptr, &response)) {
        return false;
    }

    _mavSettings.setParameterById("APMNG_ENABLED", static_cast<uint8_t>(response.autopilot_manager_enabled));
    _mavSettings.setParameterById("APMNG_DEC_IN_TP", static_cast<uint8_t>(response.decision_maker_input_type));
    _mavSettings.setParameterById("APMNG_AVD_EN", static_cast<uint8_t>(response.simple_collision_avoid_enabled));
    _mavSettings.setParameterById("APMNG_AVD_THRSH",
                                  static_cast<double>(response.simple_collision_avoid_distance_threshold));
    _mavSettings.setParameterById("APMNG_AVD_TRUE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_true));
    _mavSettings.setParameterById("APMNG_AVD_FALSE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_false));
    _mavSettings.setParameterById("APMNG_STATUS", static_cast<int>(response.response_code));

    return true;
}

bool AutopilotManagerConfig::handleStart() { return true; }

bool AutopilotManagerConfig::handleError() {
    // We just request the current configuration from autopilot-manager process
    AutopilotManagerConfigGet response{};

    const bool succeed =
        _dbusInterface->send_message(BUS_NAME, OBJECT_PATH, INTERFACE_NAME, METHOD_GET_CONFIG, nullptr, &response);
    if (!succeed) {
        std::cerr << "[AutopilotManagerConfig] Failed getting response arguments: " << response.response_code
                  << std::endl;
    }

    _mavSettings.setParameterById("APMNG_ENABLED", static_cast<uint8_t>(response.autopilot_manager_enabled));
    _mavSettings.setParameterById("APMNG_DEC_IN_TP", static_cast<uint8_t>(response.decision_maker_input_type));
    _mavSettings.setParameterById("APMNG_AVD_EN", static_cast<uint8_t>(response.simple_collision_avoid_enabled));
    _mavSettings.setParameterById("APMNG_AVD_THRSH",
                                  static_cast<double>(response.simple_collision_avoid_distance_threshold));
    _mavSettings.setParameterById("APMNG_AVD_TRUE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_true));
    _mavSettings.setParameterById("APMNG_AVD_FALSE",
                                  static_cast<uint8_t>(response.simple_collision_avoid_distance_on_condition_false));
    _mavSettings.setParameterById("APMNG_STATUS", static_cast<int>(response.response_code));

    return true;
}
