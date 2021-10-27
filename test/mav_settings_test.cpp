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
#include <gtest/gtest.h>
#include <mav_settings.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>

using namespace settings;

TEST(mav_settings_test, mav_settings_set_and_get_single) {
    // GIVEN: the MAVSettings class constructor
    MAVSettings mav_settings;
    MAVParameter mav_parameter;

    // WHEN: we add the MAVParameters for this application
    mav_settings.addParameter(MAVParameter("LTE_PIN", MAV_PARAM_EXT_TYPE_CUSTOM, 0));
    mav_settings.addParameter(MAVParameter("LTE_APN", MAV_PARAM_EXT_TYPE_CUSTOM, 1));
    mav_settings.addParameter(MAVParameter("LTE_PUK", MAV_PARAM_EXT_TYPE_CUSTOM, 2));

    // AND WHEN: we set the same MAVParameters available for this application
    mav_settings.setParameterById("LTE_PIN", reinterpret_cast<const unsigned char *>("1234"));
    mav_settings.setParameterById("LTE_APN", reinterpret_cast<const unsigned char *>("auterion.rules.com"));
    mav_settings.setParameterById("LTE_PUK", reinterpret_cast<const unsigned char *>("123456"));

    // THEN: we expect the getParameterById method to return the same values that
    // was previously set
    mav_settings.getParameterById("LTE_PIN", mav_parameter);
    EXPECT_STREQ("1234", reinterpret_cast<const char *>(mav_parameter.value.bytes));

    mav_settings.getParameterById("LTE_APN", mav_parameter);
    EXPECT_STREQ("auterion.rules.com", reinterpret_cast<const char *>(mav_parameter.value.bytes));

    mav_settings.getParameterById("LTE_PUK", mav_parameter);
    EXPECT_STREQ("123456", reinterpret_cast<const char *>(mav_parameter.value.bytes));
}
