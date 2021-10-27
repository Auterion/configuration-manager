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
#pragma once

#include <mavlink_handler.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>

class MAVLinkParameters {
   public:
    MAVLinkParameters() = default;
    ~MAVLinkParameters() = default;

    void set_mavlink_handler(MAVLinkHandler *mav_handler) {
        _mav_handler = mav_handler;
        //-- Connect to MAVLink handler
        _mav_handler->RegisterMessageHandler(
            [this](mavlink_message_t *msg, struct sockaddr *srcaddr) { handleMavLinkMessage(msg, srcaddr); });
    };

    std::atomic<bool> _paramExtValue = false;
    std::atomic<bool> _paramExtAck = false;
    std::atomic<bool> _startTransactionAck = false;
    std::atomic<bool> _commitTransactionAck = false;
    std::atomic<bool> _failure = false;

   private:
    void handleMavLinkMessage(mavlink_message_t *msg, struct sockaddr *srcaddr);
    void handleParamExtValue(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/);
    void handleParamExtAck(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/);
    void handleCommandLong(const mavlink_message_t *pMsg, struct sockaddr * /*srcaddr*/) const;

    MAVLinkHandler *_mav_handler;
};
