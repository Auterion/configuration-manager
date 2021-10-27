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
#include <configuration_manager.h>

using namespace configuration_manager;

ConfigurationManager::ConfigurationManager(std::shared_ptr<MavlinkPassthrough> &mavlinkPassthrough,
                                           const std::string &config_path)
    : _mavlinkPassthrough(mavlinkPassthrough) {
    initMavlinkHandlers();

    if (!loadConfigFromFile(config_path)) {
        std::cerr << "[configuration-manager] Failed to load configuration from file [" << config_path << "]";
        exit(1);
    }
}

void ConfigurationManager::initMavlinkHandlers() {
    _mavlinkPassthrough->subscribe_message_async(
        MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST,
        [this](const mavlink_message_t &mavlink_message) { handleParamExtRequestList(mavlink_message); });
    _mavlinkPassthrough->subscribe_message_async(
        MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ,
        [this](const mavlink_message_t &mavlink_message) { handleParamExtRequestRead(mavlink_message); });
    _mavlinkPassthrough->subscribe_message_async(
        MAVLINK_MSG_ID_PARAM_EXT_SET,
        [this](const mavlink_message_t &mavlink_message) { validParamExtSetMavlinkTarget(mavlink_message); });
    _mavlinkPassthrough->subscribe_message_async(
        MAVLINK_MSG_ID_COMMAND_LONG,
        [this](const mavlink_message_t &mavlink_message) { handleCommandLong(mavlink_message); });
}

void ConfigurationManager::addSoftwareInterface(uint16_t id, SoftwareInterface *interface) {
    if (interface != nullptr) {
        interfaces[id] = interface;
    }
}

void ConfigurationManager::sendMessage(mavlink_message_t &msg) const { _mavlinkPassthrough->send_message(msg); }

uint8_t ConfigurationManager::getSysId() const { return _mavlinkPassthrough->get_our_sysid(); }

uint8_t ConfigurationManager::getCompId() const { return _mavlinkPassthrough->get_our_compid(); }

int ConfigurationManager::getParamCount() const {
    int size = 0;
    for (auto element = interfaces.begin(); element != interfaces.end(); ++element) {
        size += element->second->getParamCount();
    }
    return size;
}

void ConfigurationManager::run() {
    for (auto element = interfaces.begin(); element != interfaces.end(); ++element) {
        element->second->run();
    }
}

void ConfigurationManager::handleParamExtRequestList(const mavlink_message_t &mavlink_message) {
    std::cout << "[configuration-manager] Received "
                 "MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST"
              << std::endl;
    const uint8_t sysid = _mavlinkPassthrough->get_our_sysid();
    const uint8_t compid = _mavlinkPassthrough->get_our_compid();

    mavlink_param_ext_request_list_t param_request;
    mavlink_msg_param_ext_request_list_decode(&mavlink_message, &param_request);
    if (param_request.target_system != sysid ||
        (param_request.target_component != compid && param_request.target_component != MAV_COMP_ID_ALL)) {
        // The command is not for us, ignore it.
        return;
    }

    std::vector<MAVParameter> param_list;
    for (auto element = interfaces.begin(); element != interfaces.end(); ++element) {
        element->second->handleParamExtRequestList(mavlink_message, param_list);
    }

    for (unsigned long index = 0; index < param_list.size(); index++) {
        if (param_list[index].isValid()) {
            std::cout << "[configuration-manager] send_all " << param_list[index].param_id << std::endl;

            // Prepare PARAM_EXT_VALUE
            mavlink_param_ext_value_t p;
            memset(&p, 0, sizeof(p));
            memcpy(&p.param_value[0], param_list[index].value.bytes, MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN);
            strncpy(p.param_id, param_list[index].param_id.c_str(), param_list[index].param_id.size());
            mavlink_message_t msg;
            mavlink_msg_param_ext_value_pack(sysid, compid, &msg, p.param_id, p.param_value,
                                             param_list[index].param_type, static_cast<uint16_t>(getParamCount()),
                                             static_cast<uint16_t>(index));

            // Send PARAM_EXT_VALUE
            _mavlinkPassthrough->send_message(msg);
        }
    }
}

void ConfigurationManager::handleParamExtRequestRead(const mavlink_message_t &mavlink_message) {
    std::cout << "Received MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ" << std::endl;
    const uint8_t sysid = _mavlinkPassthrough->get_our_sysid();
    const uint8_t compid = _mavlinkPassthrough->get_our_compid();

    mavlink_param_ext_request_read_t req;
    mavlink_msg_param_ext_request_read_decode(&mavlink_message, &req);
    if (req.target_system != sysid || (req.target_component != compid && req.target_component != MAV_COMP_ID_ALL)) {
        // The command is not for us, ignore it.
        return;
    }

    MAVParameter param;
    for (auto element = interfaces.begin(); element != interfaces.end(); ++element) {
        SoftwareInterface *interface = element->second;
        if (interface->handleParamExtRequestRead(mavlink_message, param)) {
            // A parameter is valid if it has been set since boot. A parameter is
            // either automatically set at boot after talking via mavlink with the
            // ground station or via dbus with other processes.
            if (param.isValid()) {
                std::cout << "[configuration-manager] - " << interface->getInterfaceName() << " - send "
                          << param.param_id << std::endl;

                // Prepare PARAM_EXT_VALUE
                mavlink_param_ext_value_t p;
                memset(&p, 0, sizeof(p));
                memcpy(&p.param_value[0], param.value.bytes, MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN);
                strncpy(p.param_id, param.param_id.c_str(), param.param_id.size());
                mavlink_message_t msg;
                mavlink_msg_param_ext_value_pack(sysid, compid, &msg, p.param_id, p.param_value, param.param_type,
                                                 static_cast<uint16_t>(getParamCount()),
                                                 static_cast<uint16_t>(param.param_index));

                // Send PARAM_EXT_VALUE
                _mavlinkPassthrough->send_message(msg);
            } else {
                // XXX: Do we want to handle that?
            }
            return;
        }
    }
    // XXX: We need to handle that
    // TODO: from mavlink docs: There is no formal way for the drone to signal
    // when an invalid parameter is requested (i.e. for a parameter name or id
    // that does not exist). In this case the drone should emit STATUS_TEXT. The
    // GCS may monitor for the specific notification, but will otherwise fail the
    // request after any timeout/resend cycle completes.
}

void ConfigurationManager::validParamExtSetMavlinkTarget(const mavlink_message_t &mavlink_message) {
    std::cout << "Received MAVLINK_MSG_ID_PARAM_EXT_SET" << std::endl;
    const uint8_t sysid = _mavlinkPassthrough->get_our_sysid();
    const uint8_t compid = _mavlinkPassthrough->get_our_compid();
    mavlink_param_ext_set_t param_new;
    mavlink_msg_param_ext_set_decode(&mavlink_message, &param_new);
    if (param_new.target_system != sysid ||
        (param_new.target_component != compid && param_new.target_component != MAV_COMP_ID_ALL)) {
        // The command is not for us, ignore it.
        return;
    }

    MAVParameter param_old;
    PARAM_ACK res = PARAM_ACK_VALUE_UNSUPPORTED;

    for (auto element = interfaces.begin(); element != interfaces.end(); ++element) {
        // The PARAM_EXT_SET is different from the read operations. We don't
        // restrict read operations based on the states, while we need to allow a
        // PARAM_EXT_SET only when in the TRANSACTION mode. Moreover we cannot just
        // return if we are in the wrong state because we still need to send
        // PARAM_ACK_FAILED. This is the reason why the validity check of the
        // mavlink message had to be decoupled from the actual parameter set.
        SoftwareInterface *interface = element->second;
        if (!interface->validParamExtSetMavlinkTarget(mavlink_message, param_new, param_old)) {
            continue;
        } else {
            // We accept the new parameter only if a transaction is open
            if (interface->getState() == TRANSACTION) {
                // The handleParamExtSet can only fail if the parameter doesn't exist
                if (interface->handleParamExtSet(param_new)) {
                    std::cout << "[configuration-manager] - " << interface->getInterfaceName() << " - set success "
                              << param_new.param_id << std::endl;
                    res = PARAM_ACK_ACCEPTED;
                } else {
                    std::cout << "[configuration-manager] - " << interface->getInterfaceName() << " - set fail "
                              << param_new.param_id << std::endl;
                    res = PARAM_ACK_FAILED;
                }
            } else {
                // No open transaction
                res = PARAM_ACK_FAILED;
                std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                          << " - no open transaction. set fail " << param_new.param_id << std::endl;
            }

            // Prepare PARAM_EXT_ACK
            mavlink_param_ext_ack_t p;
            memset(&p, 0, sizeof(p));
            memcpy(&p.param_value[0],
                   res == PARAM_ACK_ACCEPTED ? param_new.param_value : reinterpret_cast<char *>(param_old.value.bytes),
                   MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN);
            strncpy(p.param_id, param_new.param_id, MAVLINK_MSG_PARAM_EXT_ACK_FIELD_PARAM_ID_LEN);
            mavlink_message_t msg;
            mavlink_msg_param_ext_ack_pack(sysid, compid, &msg, p.param_id, p.param_value, param_new.param_type, res);

            // Send PARAM_EXT_ACK
            _mavlinkPassthrough->send_message(msg);
            return;
        }
    }
}

void ConfigurationManager::handleCommandLong(const mavlink_message_t &mavlink_message) {
    std::cout << "Received MAVLINK_MSG_ID_COMMAND_LONG: " << mavlink_message.msgid << std::endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&mavlink_message, &cmd);
    std::cout << "Received COMMAND_LONG with ID: " << cmd.command << std::endl;

    switch (cmd.command) {
        case MAV_CMD_PARAM_TRANSACTION:
            mavlink_message_t msg;
            uint16_t transactionId = static_cast<uint16_t>(cmd.param3);
            uint8_t action = static_cast<uint8_t>(cmd.param1);
            MAV_RESULT result = MAV_RESULT_UNSUPPORTED;
            std::cout << "[configuration-manager] Received transaction ID " << transactionId << " with action "
                      << action << std::endl;
            // For now each software component has a transaction ID.
            if (interfaces.find(transactionId) != interfaces.end()) {
                result = MAV_RESULT_ACCEPTED;
                SoftwareInterface *interface = interfaces.find(transactionId)->second;
                switch (action) {
                    case PARAM_TRANSACTION_ACTION_START:
                        if (interface->getState() != IDLE) {
                            std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Already in transaction, cancel current transaction" << std::endl;
                            interface->cancelTransaction();
                            interface->run();
                        }
                        std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                  << " - Start transaction" << std::endl;
                        interface->reset();
                        interface->startTransaction();
                        break;
                    case PARAM_TRANSACTION_ACTION_COMMIT:
                        if (interface->getState() == TRANSACTION) {
                            result = MAV_RESULT_IN_PROGRESS;
                            std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Commit transaction" << std::endl;
                            interface->commitTransaction();
                        } else if (interface->getState() == COMMIT_INPROGRESS) {
                            result = MAV_RESULT_IN_PROGRESS;
                            std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Commit in progress" << std::endl;
                        } else if (interface->getState() == DONE) {
                            result = MAV_RESULT_ACCEPTED;
                            std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Commit in progress" << std::endl;
                            interface->reset();
                        } else {
                            std::cerr << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Failed to commit transaction" << std::endl;
                            result = MAV_RESULT_FAILED;
                            interface->reset();
                        }
                        break;
                    case PARAM_TRANSACTION_ACTION_CANCEL:
                        if (interface->getState() == TRANSACTION) {
                            std::cout << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " - Cancel transaction " << std::endl;
                            interface->cancelTransaction();
                        } else {
                            std::cerr << "[configuration-manager] - " << interface->getInterfaceName()
                                      << " Failed to cancel transaction " << std::endl;
                            result = MAV_RESULT_FAILED;
                        }
                        break;
                }
            }
            mavlink_msg_command_ack_pack(_mavlinkPassthrough->get_our_sysid(), _mavlinkPassthrough->get_our_compid(),
                                         &msg, cmd.command, result, 255, -1, _mavlinkPassthrough->get_target_sysid(),
                                         _mavlinkPassthrough->get_target_compid());
            _mavlinkPassthrough->send_message(msg);
    }
}

bool ConfigurationManager::loadConfigFromFile(const std::string &config_path) {
    std::ifstream file(config_path);
    if (file.is_open()) {
        std::istringstream sin;
        std::string line;
        std::cout << "[configuration-manager] Loaded config from file:" << std::endl;
        while (std::getline(file, line)) {
            sin.str(line.substr(line.find("=") + 1));
            if (line.find("ap_config_sw_interface_enabled") != std::string::npos) {
                std::cout << "\tap_config_sw_interface_enabled: " << std::boolalpha << sin.str() << std::endl;
                sin >> std::boolalpha >> _config.ap_config_sw_interface_enabled;
            }
            if (line.find("autopilot_manager_sw_interface_enabled") != std::string::npos) {
                std::cout << "\tautopilot_manager_sw_interface_enabled: " << std::boolalpha << sin.str() << std::endl;
                sin >> std::boolalpha >> _config.autopilot_manager_sw_interface_enabled;
            }
            if (line.find("cot_sw_interface_enabled") != std::string::npos) {
                std::cout << "\tcot_sw_interface_enabled: " << std::boolalpha << sin.str() << std::endl;
                sin >> std::boolalpha >> _config.cot_sw_interface_enabled;
            }
            if (line.find("modem_config_sw_interface_enabled") != std::string::npos) {
                std::cout << "\tmodem_config_sw_interface_enabled: " << std::boolalpha << sin.str() << std::endl;
                sin >> std::boolalpha >> _config.modem_config_sw_interface_enabled;
            }
            sin.clear();
        }
        file.close();
        return true;
    } else {
        std::cerr << "[configuration-manager] Unable to open file: " << config_path << std::endl;
        return false;
    }
}
