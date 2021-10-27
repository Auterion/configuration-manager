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
#include <dbus_interface.h>

#include <thread>

using namespace dbus_interface;

DBusInterface::DBusInterface() {
    std::cout << "Preparing dbus..." << std::endl;
    dbus_error_init(&_dbus_error);

    if (!prepare_dbus_connection()) {
        std::cerr << "Failed to prepare dbus connection." << std::endl;
        exit(1);
    }
}

bool DBusInterface::prepare_dbus_connection() {
    DBusConnection *dbus_connection = dbus_bus_get(DBUS_BUS_SYSTEM, &_dbus_error);
    if (dbus_error_is_set(&_dbus_error)) {
        std::cerr << "dbus_bus_get:" << _dbus_error.message << std::endl;
        dbus_error_free(&_dbus_error);
    }

    if (!dbus_connection) {
        return false;
    }

    std::cout << "Getting a well known name...";
    bool is_well_known_name_set = false;
    do {
        const int return_code = dbus_bus_request_name(dbus_connection, BUS_NAME, 0, &_dbus_error);

        switch (return_code) {
            case DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER:
                std::cout << " Done!" << std::endl;
                is_well_known_name_set = true;
                break;
            case DBUS_REQUEST_NAME_REPLY_IN_QUEUE:
                std::cout << ".";
                std::this_thread::sleep_for(std::chrono::seconds(1));
                break;
        }

        if (dbus_error_is_set(&_dbus_error)) {
            std::cerr << std::endl << "dbus_bus_request_name: " << _dbus_error.message << std::endl;
            return false;
        }
    } while (!is_well_known_name_set);

    _dbus_connection = dbus_connection;
    return true;
}

bool DBusInterface::release_dbus_connection() {
    if (dbus_bus_release_name(_dbus_connection, BUS_NAME, &_dbus_error) == -1) {
        std::cerr << "dbus_bus_release_name: " << _dbus_error.message << std::endl;
        dbus_error_free(&_dbus_error);
        return false;
    }

    return true;
}

bool DBusInterface::send_message(const char *bus_name, const char *object_path, const char *interface_name,
                                 const char *method, Arguments *args, Response *response) {
    if (_dbus_connection != nullptr) {
        std::cout << "[DBusInterface] Preparing request..." << std::endl;
        DBusMessage *request = dbus_message_new_method_call(bus_name, object_path, interface_name, method);
        if (request == nullptr) {
            std::cerr << "[DBusInterface] Failed to prepare request!" << std::endl;
            return false;
        }

        std::cout << "[DBusInterface] Appending request arguments" << std::endl;
        DBusMessageIter iter;
        dbus_message_iter_init_append(request, &iter);
        if (args != nullptr) {
            args->appendArguments(&iter);
        }

        std::cout << "[DBusInterface] Sending request..." << std::endl;
        DBusPendingCall *pending_return;
        if (!dbus_connection_send_with_reply(_dbus_connection, request, &pending_return, 600000)) {
            std::cerr << "[DBusInterface] Failed to send request: no memory" << std::endl;
            return false;
        }
        if (pending_return == nullptr) {
            std::cerr << "[DBusInterface] Failed to send request" << std::endl;
            return false;
        }

        dbus_connection_flush(_dbus_connection);
        dbus_pending_call_block(pending_return);

        DBusMessage *reply = dbus_pending_call_steal_reply(pending_return);
        if (reply == nullptr) {
            std::cerr << "[DBusInterface] Failed to receive request" << std::endl;
            return false;
        }

        dbus_pending_call_unref(pending_return);

        if (dbus_message_get_type(reply) == DBUS_MESSAGE_TYPE_ERROR) {
            std::cerr << "[DBusInterface] Response error: " << dbus_message_get_error_name(reply) << std::endl;
            return false;
        } else {
            std::cout << "[DBusInterface] Response received" << std::endl;
            bool parsed = true;
            if (response != nullptr) {
                parsed = response->parseMessage(reply);
            }
            dbus_message_unref(reply);
            return parsed;
        }
    } else {
        return false;
    }
}
