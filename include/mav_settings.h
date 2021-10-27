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

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <string>
#include <vector>

using namespace mavsdk;

namespace settings {

MAVPACKED(typedef struct {
    union {
        float param_float;
        double param_double;
        int64_t param_int64;
        uint64_t param_uint64;
        int32_t param_int32;
        uint32_t param_uint32;
        int16_t param_int16;
        uint16_t param_uint16;
        int8_t param_int8;
        uint8_t param_uint8;
        uint8_t bytes[MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN];
    };
})
param_ext_union_t;

class MAVParameter {
   public:
    MAVParameter(const char *pid = "NO_NAME", MAV_PARAM_EXT_TYPE type = MAV_PARAM_EXT_TYPE_CUSTOM,
                 uint16_t idx = 0)  // XXX: idx might be global here
        : param_id(pid), param_type(type), param_index(idx) {}
    ~MAVParameter() = default;

    void setValue(const unsigned char *v) {
        memset(&value.bytes[0], 0, MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN);
        memcpy(&value.bytes[0], v,
               std::min(MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN,
                        static_cast<int>(strlen(reinterpret_cast<const char *>(v)))));
        valid = true;
    }

    void setValue(uint8_t v) {
        value.param_uint8 = v;
        valid = true;
    }

    void setValue(int8_t v) {
        value.param_int8 = v;
        valid = true;
    }

    void setValue(uint16_t v) {
        value.param_uint16 = v;
        valid = true;
    }

    void setValue(int16_t v) {
        value.param_int16 = v;
        valid = true;
    }

    void setValue(uint32_t v) {
        value.param_uint32 = v;
        valid = true;
    }

    void setValue(int32_t v) {
        value.param_int32 = v;
        valid = true;
    }

    void setValue(uint64_t v) {
        value.param_uint64 = v;
        valid = true;
    }

    void setValue(int64_t v) {
        value.param_int64 = v;
        valid = true;
    }

    void setValue(float v) {
        value.param_float = v;
        valid = true;
    }

    void setValue(double v) {
        value.param_double = v;
        valid = true;
    }

    bool isValid() { return valid; }

    std::string param_id;
    MAV_PARAM_EXT_TYPE param_type;
    param_ext_union_t value;
    uint16_t param_index;

   private:
    bool valid = false;
};

class MAVSettings {
   public:
    MAVSettings() = default;
    ~MAVSettings() = default;

    std::vector<MAVParameter> getParameterList() const { return _parameter_list; }

    void addParameter(MAVParameter param) {
        bool exist = false;
        for (auto param_it = _parameter_list.begin(); param_it != _parameter_list.end(); ++param_it) {
            if (param.param_id == (*param_it).param_id) {
                exist = true;
                break;
            }
        }
        if (!exist) {
            _parameter_list.push_back(param);
        }
    }

    bool getParameterById(const char *param_id, MAVParameter &param) const {
        for (auto param_it = _parameter_list.begin(); param_it != _parameter_list.end(); ++param_it) {
            if (param_id == (*param_it).param_id) {
                param = *param_it;
                return true;
            }
        }
        return false;
    }

    bool setParameterById(const char *param_id, const unsigned char *value) {
        for (auto param_it = _parameter_list.begin(); param_it != _parameter_list.end(); ++param_it) {
            if (param_id == (*param_it).param_id) {
                std::cout << "MAVSettings::setParameterById: " << value << std::endl;
                (*param_it).setValue(value);
                return true;
            }
        }
        return false;
    }

    template <typename T>
    bool setParameterById(const char *param_id, T value) {
        for (auto param_it = _parameter_list.begin(); param_it != _parameter_list.end(); ++param_it) {
            if (param_id == (*param_it).param_id) {
                std::cout << "MAVSettings::setParameterById: " << std::to_string(value) << std::endl;
                (*param_it).setValue(value);
                return true;
            }
        }
        return false;
    }

    int getParameterCount() const { return _parameter_list.size(); }

   private:
    std::vector<MAVParameter> _parameter_list;
};

}  // namespace settings
