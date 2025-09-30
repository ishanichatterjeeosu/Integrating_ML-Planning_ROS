////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <leatherman/profile.h>

#include <chrono>

#include <ros/console.h>

#ifndef SBPL_PROFILE
#define SBPL_PROFILE 0
#endif

namespace sbpl {

class stopwatch_store
{
public:

    stopwatch_store(const std::string& name, int throttle) :
        m_name(name),
        m_throttle(throttle),
        m_then(),
        m_elapsed(0.0),
        m_times(0)
    {
    }

    void start()
    {
        m_then = std::chrono::high_resolution_clock::now();
    }

    void reset()
    {
        m_elapsed = 0.0;
        m_times = 0;
    }

    void stop()
    {
        auto now = std::chrono::high_resolution_clock::now();
        m_elapsed += std::chrono::duration<double>(now - m_then).count();
        ++m_times;
        log();
    }

    void lap()
    {
        ++m_times;
        if (m_elapsed != 0.0 && (m_times % m_throttle) == 0) {
            ROS_INFO("%s: \t%0.3f Hz", m_name.c_str(), m_times / m_elapsed);
        }
        log();
    }

    void log()
    {
        if (m_elapsed != 0.0 && (m_times % m_throttle) == 0) {
            ROS_INFO("%s: \t%0.3f Hz", m_name.c_str(), m_times / m_elapsed);
        }
    }

private:

    const std::string m_name;
    const int m_throttle;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_then;
    double m_elapsed;
    int m_times;
};

/// \brief Construct a stopwatch
stopwatch::stopwatch(const std::string& name, int throttle) :
#if SBPL_PROFILE
    m_sw(std::make_shared<stopwatch_store>(name, throttle))
#else
    m_sw()
#endif
{ }

/// \brief Start a stopwatch and log the call frequency at the throttled rate
void stopwatch::start()
{
#if SBPL_PROFILE
    m_sw->start();
#endif
}

/// \brief Reset the stopwatch
void stopwatch::reset()
{
#if SBPL_PROFILE
    m_sw->reset();
#endif
}

/// \brief Stop a stopwatch
void stopwatch::stop()
{
#if SBPL_PROFILE
    m_sw->stop();
#endif
}

/// \brief Tick a stopwatch and log the call frequency at the throttled rate
void stopwatch::lap()
{
#if SBPL_PROFILE
    m_sw->lap();
#endif
}

timer::timer(const stopwatch& watch) :
    m_sw(watch.m_sw)
{
#if SBPL_PROFILE
    m_sw->start();
#endif
}

timer::~timer()
{
#if SBPL_PROFILE
    m_sw->stop();
#endif
}


} // namespace sbpl

