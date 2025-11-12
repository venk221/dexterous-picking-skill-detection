#ifndef TIMING_READER_HPP
#define TIMING_READER_HPP

#include <string>
#include <vector>
#include <fstream>

class TimingEntry {
public:
  TimingEntry(int from, double duration)
    : _from(from), _duration(duration) {}

  int getFrom() const {return _from;}
  void setFrom(int from) {_from = from;}

  int getTo() const {return _from + 1;}

  double getDuration() const {return _duration;}
  void setDuration(double duration) {_duration = duration;}
private:
  double _duration;
  int _from;
};

TimingEntry readTimingEntry(std::istream &is) {
  static constexpr std::size_t LINE_SIZE=512;

  // Read line
  char line[LINE_SIZE];
  is.getline(line, LINE_SIZE);

  // Copy into string
  std::string sLine(line);

  // Get first number
  int from = std::stoi(sLine);
  // Skip ahead till after the : and get the duration
  double duration = std::stod(std::string(sLine).substr(sLine.find(":") + 1));

  // Return the TimingEntry
  return TimingEntry(from, duration);
}

std::vector<TimingEntry> readTimingFile(const std::string &filepath) {

  std::ifstream ifs(filepath);

  std::vector<TimingEntry> ret;

  int prevTo = 0;
  while(ifs.good()) {
    TimingEntry entry = readTimingEntry(ifs);

    if(entry.getFrom() != prevTo)
      throw "Failed to parse timing file";

    ret.push_back(entry);

    prevTo = entry.getTo();

    // Peek to check if we have any more characters
    ifs.peek();
  }

  return ret;
}

#endif
