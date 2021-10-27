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

#include <configuration_manager.h>
#include <dbus_interface.h>
#include <mav_settings.h>
#include <usm.h>

#include <atomic>
#include <chrono>
#include <iostream>

using namespace dbus_interface;
using namespace settings;
using namespace usm;

namespace configuration_manager {
class ConfigurationManager;
}

using namespace configuration_manager;

namespace software_interface {

enum TransactionState { NOT_READY, IDLE, TRANSACTION, COMMIT_INPROGRESS, DONE, CANCEL, FAILED_TRANSACTION, DISABLED };

class SoftwareInterface : public StateMachine<TransactionState> {
   public:
    SoftwareInterface(const std::string &interfaceName, const std::shared_ptr<ConfigurationManager> &configManager,
                      const std::shared_ptr<DBusInterface> &dbusInterface);

    void handleParamExtRequestList(const mavlink_message_t &msg, std::vector<MAVParameter> &param_list);
    bool handleParamExtRequestRead(const mavlink_message_t &msg, MAVParameter &param);

    bool validParamExtSetMavlinkTarget(const mavlink_message_t &msg, mavlink_param_ext_set_t &param,
                                       MAVParameter &param_old);
    bool handleParamExtSet(mavlink_param_ext_set_t param_new);

    const std::string getInterfaceName() const { return _interfaceName; }
    int getParamCount() const { return _mavSettings.getParameterCount(); }

    void startTransaction() { _new_start_transaction = true; }
    void commitTransaction() { _new_commit_transaction = true; }
    void cancelTransaction() { _transaction_cancelled = true; }
    void reset();

    void run();

   private:
    std::shared_ptr<ConfigurationManager> _configManager;

   protected:
    std::shared_ptr<DBusInterface> _dbusInterface;
    MAVSettings _mavSettings;
    MAVSettings _mavSettingsSave;
    Transition runCurrentState(TransactionState currentState) override;
    TransactionState chooseNextState(TransactionState currentState, Transition transition) override;
    void printTransition(TransactionState currentState, TransactionState newState, Transition t) const override;
    void publishParameters() const;

    Transition runNotReady();
    Transition runIdle();
    Transition runTransaction();
    Transition runCommitInProcess();
    Transition runDone();
    Transition runCancel();
    Transition runFailedTransaction();

    virtual bool handleCommit() = 0;
    virtual bool handleNotReady() = 0;
    virtual bool handleStart() = 0;
    virtual bool handleError() = 0;

    std::atomic<bool> _is_disabled = false;

   private:
    std::string transactionStateToString(TransactionState state) const;
    std::string _interfaceName;
    std::atomic<bool> _new_start_transaction = false;
    std::atomic<bool> _new_commit_transaction = false;
    std::atomic<bool> _transaction_done = false;
    std::atomic<bool> _transaction_cancelled = false;
    std::atomic<bool> _transaction_failed = false;
};

}  // namespace software_interface
