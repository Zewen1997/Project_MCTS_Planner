/**
 * @file    mcts_timer.h
 * @brief   A timer implementation for control the Monte Carlo Tree Search
 */

#ifndef MCTS_TIMER_H
#define MCTS_TIMER_H

// #include "think_utils/base.h"
#include <chrono>
#include <iostream>

class LoopTimer
{
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::microseconds          Units;

 private:
  int iterations_;

 public:
  bool verbose_;

  Clock::time_point start_time_;
  Clock::time_point loop_start_time_;

  Units avg_loop_duration;
  Units run_duration;

  LoopTimer();

  // initialize timer. Call before the loop starts
  void init();

  // indicate start of loop
  void loopStart();

  // indicate end of loop
  void loopEnd();

  // check if current total run duration (since init) exceeds max_millis
  bool checkDuration(int max_millis) const;

  // return average loop duration
  int avgLoopDurationMicros() const;

  // return current total run duration (since init)
  int runDurationMicros() const;
};

#endif  // MCTS_TIMER_H
