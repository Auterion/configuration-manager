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

#include <arpa/inet.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

class MAVLinkHandler {
   public:
    // Type of callback function to be invoked on all messages.
    using HandlerFunction = std::function<void(mavlink_message_t *msg, struct sockaddr *srcaddr)>;

    // Instantiates MAVLinkHandler to communicate using given local UDP port.
    // Also take care of heartbeats.
    MAVLinkHandler(uint16_t local_port, uint8_t comp_id);
    ~MAVLinkHandler();

    // Registers given handler function to be called on every mavlink message.
    // This must be called before starting processing, see below.
    void RegisterMessageHandler(HandlerFunction handler);

    // Start mavlink processing. All handlers previously registered may be
    // called, no new handler can be registered anymore.
    void Start();

    void send_mavlink_message(const mavlink_message_t *message, struct sockaddr *srcaddr = nullptr);
    void send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result,
                      struct sockaddr *srcaddr);
    void run();
    uint8_t sysID() const { return _sysID; }

   private:
    void _send_heartbeat();

   private:
    uint8_t _compID = 255;
    uint8_t _sysID = 255;
    int _fd = -1;
    struct sockaddr_in _myaddr {};      ///< The locally bound address
    struct sockaddr_in _targetAddr {};  ///< Target address (router)
    struct pollfd _fds[1]{};
    std::atomic<bool> _threadRunning{false};
    bool _hasTarget = false;
    bool _hasSysID = true;
    std::thread _udpThread;

    std::vector<HandlerFunction> _handlers;
    std::chrono::steady_clock::time_point _lastHeartbeat;
};
