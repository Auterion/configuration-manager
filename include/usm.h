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

/**
 * @author Julian Kent <julian@auterion.com>
 */

#pragma once

namespace usm {

enum Transition { REPEAT, NEXT1, NEXT2, NEXT3, NEXT4, ERROR };

template <typename StateEnum>
class StateMachine {
   public:
    StateMachine(StateEnum startingState);
    void iterateOnce();

    StateEnum getState();

   protected:
    virtual Transition runCurrentState(StateEnum currentState) = 0;  // a big switch
    virtual StateEnum chooseNextState(StateEnum currentState,
                                      Transition transition) = 0;  // nested switches
    virtual void printTransition(StateEnum currentState, StateEnum newState, Transition t) const = 0;

    std::string transitionToString(Transition) const;

   private:
    StateEnum m_currentState;
};

/*---------------IMPLEMENTATION------------------*/

template <typename StateEnum>
StateMachine<StateEnum>::StateMachine(StateEnum startingState) : m_currentState(startingState) {}

template <typename StateEnum>
void StateMachine<StateEnum>::iterateOnce() {
    Transition t = runCurrentState(m_currentState);
    if (t != REPEAT) {
        const StateEnum newState = chooseNextState(m_currentState, t);
        printTransition(m_currentState, newState, t);
        m_currentState = newState;
    }
}

template <typename StateEnum>
StateEnum StateMachine<StateEnum>::getState() {
    return m_currentState;
}

template <typename StateEnum>
std::string StateMachine<StateEnum>::transitionToString(Transition t) const {
    switch (t) {
        case REPEAT:
            return "REPEAT";
        case NEXT1:
            return "NEXT1";
        case NEXT2:
            return "NEXT2";
        case NEXT3:
            return "NEXT3";
        case NEXT4:
            return "NEXT4";
        case ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}
}  // namespace usm

/*---------------MACROS TO MAKE TRANSITION TABLES EASY------------------*/

// clang-format off
#define USM_TABLE(current_state, error, ...) \
switch (current_state) { \
    __VA_ARGS__; \
    default: break; \
} \
return error

#define USM_STATE(transition, start_state, ...) \
    case start_state: \
        switch (transition) { \
            __VA_ARGS__; \
            default: break; \
        } \
    break

#define USM_MAP(transition, next_state) \
            case transition: return next_state

// clang-format on
