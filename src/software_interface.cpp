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
#include <software_interface.h>

using namespace configuration_manager;
using namespace software_interface;

SoftwareInterface::SoftwareInterface(const std::string &interfaceName,
                                     const std::shared_ptr<ConfigurationManager> &configManager,
                                     const std::shared_ptr<DBusInterface> &dbusInterface)
    : StateMachine<TransactionState>(NOT_READY),
      _configManager(configManager),
      _dbusInterface(dbusInterface),
      _interfaceName(interfaceName) {}

void SoftwareInterface::run() { iterateOnce(); }

void SoftwareInterface::reset() {
    _new_commit_transaction = false;
    _transaction_cancelled = false;
    _transaction_done = false;
    _transaction_failed = false;
}

void SoftwareInterface::handleParamExtRequestList(const mavlink_message_t &msg, std::vector<MAVParameter> &param_list) {
    std::cout << "[" << _interfaceName << "]"
              << " handleParamExtRequestList: " << msg.msgid << std::endl;
    std::vector<MAVParameter> internal_param_list = _mavSettings.getParameterList();
    param_list.insert(param_list.end(), std::make_move_iterator(internal_param_list.begin()),
                      std::make_move_iterator(internal_param_list.end()));
}

bool SoftwareInterface::handleParamExtRequestRead(const mavlink_message_t &msg, MAVParameter &param) {
    std::cout << "[" << _interfaceName << "]"
              << " handleParamExtRequestRead: " << msg.msgid << std::endl;
    mavlink_param_ext_request_read_t req;
    mavlink_msg_param_ext_request_read_decode(&msg, &req);
    return _mavSettings.getParameterById(req.param_id, param);
}

bool SoftwareInterface::handleParamExtSet(mavlink_param_ext_set_t param_new) {
    std::cout << "[" << _interfaceName << "]"
              << " handleParamExtSet" << std::endl;
    if (param_new.param_type == MAV_PARAM_EXT_TYPE_UINT8) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<uint8_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_INT8) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<int8_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_UINT16) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<uint16_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_INT16) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<int16_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_UINT32) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<uint32_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_INT32) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<int32_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_UINT64) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<uint64_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_INT64) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<int64_t *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_REAL32) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<float *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_REAL64) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             (*reinterpret_cast<double *>(&param_new.param_value[0])));
    } else if (param_new.param_type == MAV_PARAM_EXT_TYPE_CUSTOM) {
        return _mavSettings.setParameterById(param_new.param_id,
                                             reinterpret_cast<const unsigned char *>(param_new.param_value));
    }
    return false;
}

bool SoftwareInterface::validParamExtSetMavlinkTarget(const mavlink_message_t &msg, mavlink_param_ext_set_t &param,
                                                      MAVParameter &param_old) {
    std::cout << "[" << _interfaceName << "]"
              << " validMavlinkTarget?: " << msg.msgid << std::endl;
    return _mavSettings.getParameterById(param.param_id, param_old);
}

void SoftwareInterface::printTransition(TransactionState currentState, TransactionState newState, Transition t) const {
    const std::string currentStateStr = transactionStateToString(currentState);
    const std::string newStateStr = transactionStateToString(newState);
    const std::string transitionStr = transitionToString(t);

    std::cout << "[" << _interfaceName << "] " << currentStateStr << " -> " << newStateStr << " [" << transitionStr
              << "]" << std::endl;
}

void SoftwareInterface::publishParameters() const {
    std::vector<MAVParameter> param_list = _mavSettings.getParameterList();
    for (unsigned long index = 0; index < param_list.size(); index++) {
        if (param_list[index].isValid()) {
            std::cout << "[" << _interfaceName << "] send_all " << param_list[index].param_id << std::endl;
            // Prepare PARAM_EXT_VALUE
            mavlink_param_ext_value_t p;
            memset(&p, 0, sizeof(p));
            memcpy(&p.param_value[0], param_list[index].value.bytes, MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN);
            strncpy(p.param_id, param_list[index].param_id.c_str(), param_list[index].param_id.size());
            mavlink_message_t msg;
            mavlink_msg_param_ext_value_pack(_configManager->getSysId(), _configManager->getCompId(), &msg, p.param_id,
                                             p.param_value, param_list[index].param_type,
                                             static_cast<uint16_t>(param_list.size()), static_cast<uint16_t>(index));

            // Send PARAM_EXT_VALUE
            _configManager->sendMessage(msg);
        }
    }
}

std::string SoftwareInterface::transactionStateToString(TransactionState state) const {
    switch (state) {
        case NOT_READY:
            return "NOT_READY";
        case IDLE:
            return "IDLE";
        case TRANSACTION:
            return "TRANSACTION";
        case COMMIT_INPROGRESS:
            return "COMMIT_INPROGRESS";
        case DONE:
            return "DONE";
        case FAILED_TRANSACTION:
            return "FAILED_TRANSACTION";
        case CANCEL:
            return "CANCEL";
        default:
            return "UNKNOWN";
    }
}

TransactionState SoftwareInterface::chooseNextState(TransactionState currentState, Transition transition) {
    // clang-format off
    USM_TABLE(
        currentState,           NOT_READY,
        USM_STATE(transition,   NOT_READY,              USM_MAP(NEXT1, IDLE);
                                                        USM_MAP(NEXT2, DISABLED));
        USM_STATE(transition,   IDLE,                   USM_MAP(NEXT1, TRANSACTION));
        USM_STATE(transition,   TRANSACTION,            USM_MAP(NEXT1, COMMIT_INPROGRESS);
                                                        USM_MAP(NEXT2, CANCEL);
                                                        USM_MAP(ERROR, FAILED_TRANSACTION));
        USM_STATE(transition,   COMMIT_INPROGRESS,      USM_MAP(NEXT1, DONE);
                                                        USM_MAP(ERROR, FAILED_TRANSACTION));
        USM_STATE(transition,   DONE,                   USM_MAP(NEXT1, IDLE));
        USM_STATE(transition,   CANCEL,                 USM_MAP(NEXT1, IDLE));
        USM_STATE(transition,   FAILED_TRANSACTION,     USM_MAP(NEXT1, IDLE));
    );
    // clang-format on
}

Transition SoftwareInterface::runCurrentState(TransactionState currentState) {
    // clang-format off
    switch(currentState) {
        case NOT_READY: return runNotReady();
        case IDLE: return runIdle();
        case TRANSACTION: return runTransaction();
        case COMMIT_INPROGRESS: return runCommitInProcess();
        case DONE: return runDone();
        case CANCEL: return runCancel();
        case FAILED_TRANSACTION: return runFailedTransaction();
        case DISABLED: return REPEAT;
    }
    // clang-format on
    return Transition::ERROR;
}

Transition SoftwareInterface::runNotReady() {
    if (!handleNotReady()) {
        return REPEAT;
    }
    if (_is_disabled) return NEXT2;
    return NEXT1;
}

Transition SoftwareInterface::runIdle() {
    _transaction_done = false;
    _transaction_cancelled = false;
    _transaction_failed = false;
    if (!_new_start_transaction) {
        return REPEAT;
    }
    publishParameters();
    return NEXT1;
}

Transition SoftwareInterface::runTransaction() {
    if (_new_start_transaction) {
        _new_start_transaction = false;
        _mavSettingsSave = _mavSettings;
        if (!handleStart()) {
            _transaction_failed = true;
            return ERROR;
        }
    }
    if (_transaction_cancelled) {
        return NEXT2;
    }
    if (!_new_commit_transaction) {
        return REPEAT;
    }
    return NEXT1;
}

Transition SoftwareInterface::runCommitInProcess() {
    if (_new_commit_transaction) {
        _new_commit_transaction = false;
        if (!handleCommit()) {
            _transaction_failed = true;
            return ERROR;
        }
        _transaction_done = true;
        return NEXT1;
    }
    return REPEAT;
    // For the moment we run that on main thread, so the commit will block the
    // main thread
}

Transition SoftwareInterface::runDone() {
    _mavSettingsSave = _mavSettings;
    while (_transaction_done) {
        return REPEAT;
    }
    return NEXT1;
}

Transition SoftwareInterface::runCancel() {
    _mavSettings = _mavSettingsSave;
    publishParameters();
    return NEXT1;
}

Transition SoftwareInterface::runFailedTransaction() {
    while (_transaction_failed) {
        return REPEAT;
    }
    return NEXT1;
}
