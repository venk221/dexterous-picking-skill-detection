#ifndef SCRAP_BURNING_TIMER_HPP
#define SCRAP_BURNING_TIMER_HPP

#include <chrono>
#include <ratio>

namespace scrap_burning {
  template <typename Clock=std::chrono::steady_clock>
  class Timer {
  public:
    Timer() :
      _times{Clock::now()} {}

    void addLap() {
      _times.push_back(Clock::now());
    }

    std::chrono::time_point<Clock> operator[](std::size_t idx) const {return _times[idx];}

    template <typename Res=std::ratio<1, 1> >
    void output(std::ostream &os) {
      if(_times.size() == 1) return; // Nothing to output
      
      for(std::size_t idx = 1; idx < _times.size(); ++idx)
	os << idx - 1 << " -> " << idx << ": " << _getTimeElapsed<Res>(idx, idx - 1) << '\n';
    }
  private:
    std::vector<std::chrono::time_point<Clock> > _times;

    template <typename Res>
    double _getTimeElapsed(std::size_t id0, std::size_t id1) const {
      return std::chrono::duration<double, Res>(_times[id0] - _times[id1]).count();
    }
  };
}

#endif
