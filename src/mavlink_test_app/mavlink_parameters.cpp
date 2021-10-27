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
#include <mavlink_parameters.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

void MAVLinkParameters::handleMavLinkMessage(mavlink_message_t *msg, struct sockaddr *srcaddr) {
    if (msg->compid != MAV_COMP_ID_ONBOARD_COMPUTER) return;

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
            handleParamExtValue(msg, srcaddr);
            break;
        case MAVLINK_MSG_ID_PARAM_EXT_ACK:
            handleParamExtAck(msg, srcaddr);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg, srcaddr);
        default:
            break;
    }
}

void MAVLinkParameters::handleParamExtValue(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/) {
    mavlink_param_ext_value_t param;
    mavlink_msg_param_ext_value_decode(pMsg, &param);

    _paramExtValue = true;

    std::cout << "_handleParamExtValue: ID: " << param.param_id << " Value: " << param.param_value << std::endl;
    std::cout << "Count: " << param.param_count << " Index: " << param.param_index << std::endl;
}

void MAVLinkParameters::handleParamExtAck(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/) {
    mavlink_param_ext_ack_t param;
    mavlink_msg_param_ext_ack_decode(pMsg, &param);

    if (param.param_result == PARAM_ACK_ACCEPTED) {
        _paramExtAck = true;
    } else if (param.param_result != PARAM_ACK_IN_PROGRESS) {
        _failure = true;
    }

    std::cout << "_handleParamExtAck: ID: " << param.param_id << " Value: " << param.param_value << std::endl;
}

void MAVLinkParameters::handleCommandLong(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/) const {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    std::cout << "Received COMMAND_LONG with ID: " << cmd.command << std::endl;

    // switch (cmd.command) {
    //         case MAV_CMD_PARAM_TRANSACTION:
    // }
}
