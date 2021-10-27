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
#include <mavlink_handler.h>
#include <mavlink_parameters.h>

#define LTE_TRANSACTION_ID 123
#define APMNG_TRANSACTION_ID 126

// modem_config parameter transaction tester
void startModemConfigTransaction(MAVLinkHandler *mav_handler);
void cancelModemConfigTransaction(MAVLinkHandler *mav_handler);
void commitModemConfigTransaction(MAVLinkHandler *mav_handler);
void sendParamExtSetPIN(MAVLinkHandler *mav_handler);
void sendParamExtSetAPN(MAVLinkHandler *mav_handler);
void sendParamExtSetEnableRoaming(MAVLinkHandler *mav_handler);
void sendParamExtSeEnablePin(MAVLinkHandler *mav_handler);
void sendGoodModemConfigTransaction(MAVLinkHandler *mav_handler, MAVLinkParameters *mavlink_parameters);

// autopilot_manager parameter transaction tester
void startAPMTransaction(MAVLinkHandler *mav_handler);
void cancelAPMTransaction(MAVLinkHandler *mav_handler);
void commitAPMTransaction(MAVLinkHandler *mav_handler);
void sendParamSetAPMEnabled(MAVLinkHandler *mav_handler);
void sendParamSetAPMDecisionMakerInputType(MAVLinkHandler *mav_handler);
void sendParamSetAPMSimpleCollAvoidEnabled(MAVLinkHandler *mav_handler);
void sendParamSetAPMSimpleCollAvoidDistanceThreshold(MAVLinkHandler *mav_handler);
void sendParamSetAPMSimpleCollAvoidActionOnCondTrue(MAVLinkHandler *mav_handler);
void sendParamSetAPMSimpleCollAvoidActionOnCondFalse(MAVLinkHandler *mav_handler);
void sendGoodAutopilotManagerTransaction(MAVLinkHandler *mav_handler, MAVLinkParameters *mavlink_parameters);

// XXX: the mavlink handler was modified to not take the system id from
//      the autopilot but to "pretend" to be a ground station.
// XXX: the reason why I'm not using the mavlink passthrough is that this
//      was recycled from an old integration test. Would make sense to change
//      this if it's useful for testing. I am already using the mavsdk for the
//      mavlink headers since the previous example had mavlink as a submodule.
int main(int argc, char **argv) {
    MAVLinkParameters mavlink_parameters;
    MAVLinkHandler mavlink_handler(14510, 255);

    // Unused functions
    (void)cancelModemConfigTransaction;
    (void)cancelAPMTransaction;

    if (argc == 2) {
        // Starting MAVLink handler
        mavlink_parameters.set_mavlink_handler(&mavlink_handler);
        mavlink_handler.Start();

        if (std::string(argv[1]) == "modem_config") {
            sendGoodModemConfigTransaction(&mavlink_handler, &mavlink_parameters);
            std::cout << "modem_config param transaction done" << std::endl;
        } else if (std::string(argv[1]) == "autopilot_manager") {
            sendGoodAutopilotManagerTransaction(&mavlink_handler, &mavlink_parameters);
            std::cout << "autopilot_manager param transaction done" << std::endl;
        } else {
            std::cerr << "-- Invalid argument! Use 'modem_config' or "
                         "'autopilot_manager'.\nExiting..."
                      << std::endl;
            exit(1);
        }

    } else {
        std::cerr << "-- No test selected. Please pass as an argument 'modem_config' or "
                     "'autopilot_manager' \n\t\tto test parameter transactions with "
                     "either one or the other.\n-- Exiting..."
                  << std::endl;
    }

    exit(0);
}

// ***** modem_config parameter transaction tester ******//
void startModemConfigTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Start transaction to modem_config" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_START, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  LTE_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void cancelModemConfigTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Cancel transaction to modem_config" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_CANCEL, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  LTE_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void commitModemConfigTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Commit transaction to modem_config" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_COMMIT, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  LTE_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamExtSetPIN(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamV3Set" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "LTE_PIN", "",
                                   MAV_PARAM_EXT_TYPE_CUSTOM);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamExtSetAPN(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "LTE_APN", "internet",
                                   MAV_PARAM_EXT_TYPE_CUSTOM);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamExtSetEnableRoaming(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "LTE_ROAMING", "1",
                                   MAV_PARAM_EXT_TYPE_CUSTOM);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamExtSeEnablePin(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "LTE_PIN_ENABLED", "0",
                                   MAV_PARAM_EXT_TYPE_CUSTOM);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendGoodModemConfigTransaction(MAVLinkHandler *mav_handler, MAVLinkParameters *mavlink_parameters) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Starting transaction to the modem_config" << std::endl;

    // while(!mavlink_parameters->_startTransactionAck){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    startModemConfigTransaction(mav_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     if(mavlink_parameters->_failure){
    //         std::cout << "Failed to start transaction" << std::endl;
    //         return;
    //     }
    // }
    // mavlink_parameters->_startTransactionAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamExtSetPIN(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamExtSetAPN(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamExtSetEnableRoaming(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamExtSeEnablePin(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    // while(!mavlink_parameters->_commitTransactionAck){
    commitModemConfigTransaction(mav_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     if(mavlink_parameters->_failure){
    //         std::cout << "Failed to commit transaction" << std::endl;
    //         return;
    //     }
    // }
    // mavlink_parameters->_commitTransactionAck = false;
}
// ******************************************************//

// *** autopilot_manager parameter transaction tester ***//
void startAPMTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Start transaction to autopilot_manager" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_START, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  APMNG_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void cancelAPMTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Cancel transaction to autopilot_manager" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_CANCEL, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  APMNG_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void commitAPMTransaction(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "Commit transaction to autopilot_manager" << std::endl;
    mavlink_msg_command_long_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_CMD_PARAM_TRANSACTION, 0,
                                  PARAM_TRANSACTION_ACTION_COMMIT, PARAM_TRANSACTION_TRANSPORT_PARAM_EXT,
                                  APMNG_TRANSACTION_ID, 0, 0, 0, 0);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMEnabled(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_ENABLED" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_ENABLED", "1",
                                   MAV_PARAM_EXT_TYPE_UINT8);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMDecisionMakerInputType(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_DEC_IN_TP" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_DEC_IN_TP", "2",
                                   MAV_PARAM_EXT_TYPE_UINT8);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMSimpleCollAvoidEnabled(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_AVD_EN" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_AVD_EN", "1",
                                   MAV_PARAM_EXT_TYPE_UINT8);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMSimpleCollAvoidDistanceThreshold(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_AVD_THRSH" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_AVD_THRSH", "4.79",
                                   MAV_PARAM_EXT_TYPE_REAL64);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMSimpleCollAvoidActionOnCondTrue(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_AVD_TRUE" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_AVD_TRUE", "2",
                                   MAV_PARAM_EXT_TYPE_UINT8);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendParamSetAPMSimpleCollAvoidActionOnCondFalse(MAVLinkHandler *mav_handler) {
    mavlink_message_t msg;

    std::cout << "sendParamExtSet: APMNG_AVD_FALSE" << std::endl;
    mavlink_msg_param_ext_set_pack(255, 255, &msg, 1, MAV_COMP_ID_ONBOARD_COMPUTER, "APMNG_AVD_FALSE", "0",
                                   MAV_PARAM_EXT_TYPE_UINT8);
    mav_handler->send_mavlink_message(&msg, nullptr);
}

void sendGoodAutopilotManagerTransaction(MAVLinkHandler *mav_handler, MAVLinkParameters *mavlink_parameters) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Starting transaction to the autopilot_manager" << std::endl;

    // while(!mavlink_parameters->_startTransactionAck){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    startAPMTransaction(mav_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     if(mavlink_parameters->_failure){
    //         std::cout << "Failed to start transaction" << std::endl;
    //         return;
    //     }
    // }
    // mavlink_parameters->_startTransactionAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMEnabled(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMDecisionMakerInputType(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMSimpleCollAvoidEnabled(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMSimpleCollAvoidDistanceThreshold(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMSimpleCollAvoidActionOnCondTrue(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    while (!mavlink_parameters->_paramExtAck) {
        sendParamSetAPMSimpleCollAvoidActionOnCondFalse(mav_handler);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (mavlink_parameters->_failure) {
            std::cout << "Transaction Failed" << std::endl;
            return;
        }
    }
    mavlink_parameters->_paramExtAck = false;

    // while(!mavlink_parameters->_commitTransactionAck){
    commitAPMTransaction(mav_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     if(mavlink_parameters->_failure){
    //         std::cout << "Failed to commit transaction" << std::endl;
    //         return;
    //     }
    // }
    // mavlink_parameters->_commitTransactionAck = false;
}
// ******************************************************//
