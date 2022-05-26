#include "mcts_timer.h"

LoopTimer::LoopTimer() : verbose_(false) {}

void LoopTimer::init()
{
  start_time_ = Clock::now();
  iterations_ = 0;
}

void LoopTimer::loopStart()
{
  loop_start_time_ = Clock::now();
  iterations_++;
}

void LoopTimer::loopEnd()
{
  auto loop_end_time         = Clock::now();
  auto current_loop_duration = std::chrono::duration_cast<Units>(loop_end_time - loop_start_time_);

  run_duration      = std::chrono::duration_cast<Units>(loop_end_time - start_time_);
  avg_loop_duration = std::chrono::duration_cast<Units>(run_duration / iterations_);

  if (verbose_)
  {
    std::cout << iterations_ << ": ";
    std::cout << "run_duration: " << run_duration.count() << ", ";
    std::cout << "current_loop_duration: " << current_loop_duration.count() << ", ";
    std::cout << "avg_loop_duration: " << avg_loop_duration.count() << ", ";
    std::cout << std::endl;
  }
}

bool LoopTimer::checkDuration(int max_millis) const
{
  // estimate when the next loop will end
  auto next_loop_end_time = Clock::now() + avg_loop_duration;
  return next_loop_end_time > start_time_ + std::chrono::milliseconds(max_millis);
}

int LoopTimer::avgLoopDurationMicros() const
{
  return std::chrono::duration_cast<Units>(avg_loop_duration).count();
}

int LoopTimer::runDurationMicros() const
{
  return std::chrono::duration_cast<Units>(run_duration).count();
}

