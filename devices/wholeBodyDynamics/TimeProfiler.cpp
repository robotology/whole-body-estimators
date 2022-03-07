/**
 * @file TimeProfiler.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>

#include <yarp/os/LogStream.h>

#include "TimeProfiler.h"

using namespace WholeBodyDynamics;

std::string
printTimerDescription(const std::string& name, const TimerHandler::TimerDescription& description)
{
    std::stringstream ss;
    ss << "|" << std::setw(30) << name << "|" << std::setw(15) << std::setprecision(13)
       << description.averageDuration.count() << "|" << std::setw(10) << std::setprecision(8)
       << description.timer.getInfo().deadlineMiss << "|" << std::setw(15) << std::setprecision(13)
       << description.timer.getInfo().latestDeadlineMissDuration.count() << "|" << std::endl;

    return ss.str();
};

Timer::Timer(const std::string& name, const std::chrono::duration<double>& expectedDuration)
{
    this->setName(name);
    this->setExpectedDuration(expectedDuration);
}

void Timer::tic()
{
    m_initTime = std::chrono::system_clock::now();
    m_info.dealineMissDetected = false;
}

void Timer::toc()
{
    m_info.duration = std::chrono::system_clock::now() - m_initTime;
    if (m_info.duration > m_exprectedDuration)
    {
        m_info.dealineMissDetected = true;
        m_info.deadlineMiss++;
        m_info.latestDeadlineMissDuration = m_info.duration;
    }
}

void Timer::setName(const std::string& name)
{
    m_info.name = name;
}

void Timer::setExpectedDuration(const std::chrono::duration<double>& expectedDuration)
{
    m_exprectedDuration = expectedDuration;
}

const Timer::Info& Timer::getInfo() const
{
    return m_info;
}

void TimerHandler::setHorizon(unsigned int horizon)
{
    m_horizon = horizon;
}

bool TimerHandler::addTimer(const std::string& name, const Timer& timer)
{
    auto it = m_timers.find(name);
    if (it != m_timers.end())
    {
        return false;
    }

    m_timers[name].timer = timer;

    return true;
}

void TimerHandler::tic(const std::string& key)
{
    // if the timer does not exist we add it
    m_timers[key].timer.tic();
}

bool TimerHandler::toc(const std::string& key)
{
    auto it = m_timers.find(key);
    if (it == m_timers.end())
    {
        return false;
    }

    it->second.timer.toc();
    return true;
}

void TimerHandler::setVerbosity(bool verbosity)
{
    m_verbosity = verbosity;
}

void TimerHandler::profiling()
{
    bool deadlineMissDetected = false;
    for (auto& [name, timerDescription] : m_timers)
    {
        const auto& duration = timerDescription.timer.getInfo().duration;

        deadlineMissDetected = deadlineMissDetected || timerDescription.timer.getInfo().dealineMissDetected;

        // this automatically create the element if does bot exist
        auto& queue = m_durations[name];
        queue.push_back(duration);

        // keep the queue size equal to horizon
        if (queue.size() > m_horizon)
        {
            queue.pop_front();
        }

        // this should never happen
        assert(queue.size() <= m_horizon);

        timerDescription.averageDuration
            = std::accumulate(std::next(queue.begin()), queue.end(), queue.front())
              / double(queue.size());
    }

    if (m_verbosity)
    {
        if (m_verbosityCounter == 0)
        {
            std::stringstream ss;
            std::string output;

            ss << "|" << std::setw(30) << "name            |" << std::setw(15) << "tavarg (s)  |"
               << std::setw(10) << "dm    |" << std::setw(15) << "tdm (s)   |" << std::endl
               << "        " << std::setfill('-') << "|" << std::setw(30) << "|" << std::setw(15)
               << "|" << std::setw(10) << "|" << std::setw(15) << "|" << std::endl;

            output = ss.str();

            for (const auto& [name, timerDescription] : m_timers)
            {
                output += "        " + printTimerDescription(name, timerDescription);
            }

            yDebug() << output;
        }

        if (deadlineMissDetected)
        {
            std::stringstream ss;
            std::string output;

            ss << "---------------------------------- Deadline miss detected ----------------------------------" << std::endl;
            ss << "|" << std::setw(30) << "name            |" << std::setw(15) << "t (s)  |" << std::endl
               << "        " << std::setfill('-') << "|" << std::setw(30) << "|" << std::setw(15) << std::endl;

            for (const auto& [name, timerDescription] : m_timers)
            {
                ss << "|" << std::setw(30) << name << "|" << std::setw(15) << std::setprecision(13)
                   << timerDescription.timer.getInfo().duration.count() << std::endl;
            }

            yDebug() << ss.str();
        }

        m_verbosityCounter++;
        if (m_verbosityCounter == m_horizon)
            m_verbosityCounter = 0;
    }
}
