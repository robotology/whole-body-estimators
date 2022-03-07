#ifndef TIME_PROFILER_H
#define TIME_PROFILER_H

// std
#include <chrono>
#include <deque>
#include <unordered_map>

namespace WholeBodyDynamics
{

/**
 * Simple timer.
 */
class Timer
{

public:
    struct Info
    {
        unsigned int deadlineMiss{0}; /**< Number of deadline miss */
        std::string name{"Timer"}; /**< Name associated to the timer */
        bool dealineMissDetected{false};
      std::chrono::duration<double> latestDeadlineMissDuration{0}; /**< Average duration. */
      std::chrono::duration<double> duration{0}; /**< Latest duration. */
    };

    Timer(const std::string& name, const std::chrono::duration<double>& expectedDuration);

    Timer() = default;

    void setName(const std::string& name);

    void setExpectedDuration(const std::chrono::duration<double>& expectedDuration);

    /**
     * Set initial time.
     */
    inline void tic();

    /**
     * Set final time.
     */
    inline void toc();

    /**
     * Get the average duration.
     * @return average duration.
     */
    const Info& getInfo() const;

private:
    std::chrono::time_point<std::chrono::system_clock> m_initTime; /**< Init time. */
    std::chrono::duration<double> m_exprectedDuration{std::chrono::duration<double>::max()};

    Info m_info;
};

/**
 * Simple Time profiler class
 */
class TimerHandler
{
public:

    struct TimerDescription
    {
        ::WholeBodyDynamics::Timer timer;
        std::chrono::duration<double> averageDuration{0};
    };

    /**
     * Set the output period.
     * @param maxCounter is the period (expressed in cycles).
     */
    void setHorizon(unsigned int horizon);

    /**
     * Add a new timer
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    void tic(const std::string& key);

    /**
     * Set the init time for the timer named "key"
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool toc(const std::string& key);

    /**
     * Print the profiling quantities.
     */
    void profiling();

    void setVerbosity(bool verbosity);

    bool addTimer(const std::string& name, const Timer& timer);

private:
    unsigned int m_horizon; /**< Counter useful to print the profiling quantities only every
                  m_maxCounter times. */

    std::unordered_map<std::string, TimerHandler::TimerDescription> m_timers; /**< Dictionary that contains all
                                                               the timers. */

    std::unordered_map<std::string, std::deque<std::chrono::duration<double>>> m_durations;

    bool m_verbosity{false};
    unsigned int m_verbosityCounter{0};
};
}; // namespace WholeBodyDynamics

#endif // TIME_PROFILER_H
