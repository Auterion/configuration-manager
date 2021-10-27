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

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

//-----------------------------------------------------------------------------
MAVLinkHandler::MAVLinkHandler(uint16_t local_port, uint8_t comp_id)
    : _compID(comp_id), _lastHeartbeat(std::chrono::steady_clock::now()) {
    memset(&_targetAddr, 0, sizeof(_targetAddr));

    // Unused
    (void)&MAVLinkHandler::send_cmd_ack;

    _fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (_fd < 0) {
        throw std::runtime_error(std::string("Create camera UDP socket failed: ") + ::strerror(errno));
    }
    memset(&_myaddr, 0, sizeof(_myaddr));
    _myaddr.sin_family = AF_INET;
    _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    _myaddr.sin_port = htons(local_port);
    if (::bind(_fd, reinterpret_cast<struct sockaddr *>(&_myaddr), sizeof(_myaddr)) < 0) {
        throw std::runtime_error(std::string("Bind failed for UDP port ") + std::to_string(local_port) + ": " +
                                 ::strerror(errno));
    }
    _fds[0].fd = _fd;
    _fds[0].events = POLLIN;
    mavlink_status_t *chan_state = mavlink_get_channel_status(MAVLINK_COMM_1);
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
}

//-----------------------------------------------------------------------------
MAVLinkHandler::~MAVLinkHandler() {
    _threadRunning.store(false, std::memory_order_relaxed);
    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }
    if (_udpThread.joinable()) {
        _udpThread.join();
    }
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::RegisterMessageHandler(HandlerFunction handler) {
    if (_threadRunning.load(std::memory_order_relaxed)) {
        throw std::logic_error("Registration must be complete before starting MAVLinkHandler");
    }
    _handlers.push_back(std::move(handler));
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::Start() {
    _threadRunning.store(true, std::memory_order_relaxed);
    _udpThread = std::thread(&MAVLinkHandler::run, this);
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::send_mavlink_message(const mavlink_message_t *message, struct sockaddr *srcaddr) {
    struct sockaddr *target = srcaddr ? srcaddr : reinterpret_cast<struct sockaddr *>(&_targetAddr);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    size_t packetlen = mavlink_msg_to_send_buffer(buffer, message);
    ssize_t len = sendto(_fd, buffer, packetlen, 0, target, sizeof(_targetAddr));
    if (len <= 0) {
        std::cerr << "Failed sending mavlink message: " << message->msgid << std::endl;
    }
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result,
                                  struct sockaddr *srcaddr) {
    if (_hasSysID) {
        mavlink_message_t msg;
        mavlink_msg_command_ack_pack_chan(_sysID, _compID, MAVLINK_COMM_1, &msg, cmd, result, 100, 0, target_sysid,
                                          target_compid);
        send_mavlink_message(&msg, srcaddr);
    }
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::_send_heartbeat() {
    if (_hasSysID) {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack_chan(_sysID, _compID, MAVLINK_COMM_1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                                        0, 0, 0);
        send_mavlink_message(&msg);
    }
    fflush(stdout);
    fflush(stderr);
}

//-----------------------------------------------------------------------------
void MAVLinkHandler::run() {
    std::cout << "MAVLink thread started" << std::endl;
    mavlink_status_t status;
    mavlink_message_t msg;
    unsigned char buffer[16 * 1024];
    while (_threadRunning.load(std::memory_order_relaxed)) {
        ::poll(&_fds[0], (sizeof(_fds[0]) / sizeof(_fds[0])), 1000);
        if (_fd < 0) {
            break;
        }
        if (_fds[0].revents & POLLIN) {
            struct sockaddr srcaddr {};
            socklen_t addrlen = sizeof(srcaddr);
            ssize_t len =
                recvfrom(_fd, buffer, sizeof(buffer), 0, reinterpret_cast<struct sockaddr *>(&srcaddr), &addrlen);

            if (len > 0) {
                for (ssize_t i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status)) {
                        if (!_hasTarget) {
                            _hasTarget = true;
                            memcpy(&_targetAddr, &srcaddr, sizeof(_targetAddr));
                            char addr[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &(_targetAddr.sin_addr), addr, INET_ADDRSTRLEN);
                            std::cout << "Got UDP target: " << addr << ":" << std::dec << ntohs(_targetAddr.sin_port)
                                      << std::endl;
                        }
                        // XXX: this check is usually used to take the same systemid as the
                        // autopilot but for this application we want
                        //      to simulate a ground station so this was hardcoded to 255
                        // if (!_hasSysID && msg.compid == MAV_COMP_ID_AUTOPILOT1 &&
                        // msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        //     _sysID = msg.sysid;
                        //     _hasSysID = true;
                        // }
                        for (const auto &handler : _handlers) {
                            handler(&msg, &srcaddr);
                        }
                        memset(&status, 0, sizeof(status));
                        memset(&msg, 0, sizeof(msg));
                    }
                }
            }
        }
        // Our heartbeat
        if (_hasTarget) {
            auto current = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - _lastHeartbeat).count() > 1000) {
                _lastHeartbeat = current;
                _send_heartbeat();
            }
        }
    }
    std::cout << "MAVLink thread stopped" << std::endl;
}
