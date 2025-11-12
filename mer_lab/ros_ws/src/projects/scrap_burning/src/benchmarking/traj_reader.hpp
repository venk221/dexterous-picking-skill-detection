#ifndef TRAJ_READER_HPP
#define TRAJ_READER_HPP

#include <fstream>
#include <string>

#include <Eigen/Geometry>

Eigen::Isometry3d readPoint(const std::string &line) {
  Eigen::Isometry3d ret(Eigen::Isometry3d::Identity());
  std::array<double, 7> data;

  // Split the string at commas
  std::size_t searchStart = 0;
  std::size_t prevSearchStart = 0;
  for(int i = 0; i < data.size(); ++i) {
    data[i] = std::stod(line.substr(searchStart + 1));
    searchStart = line.find(",", searchStart + 1);
  }

  ret.translation() = Eigen::Vector3d(data[0], data[1], data[2]);
  ret.linear() = Eigen::Quaterniond(data[6], data[3], data[4], data[5]).normalized().toRotationMatrix();

  return ret;
}

std::vector<Eigen::Isometry3d> getPoses(std::istream &is) {
  static constexpr std::size_t LINE_SIZE=512;

  std::vector<Eigen::Isometry3d> ret;
  char line[LINE_SIZE];

  is.getline(line, LINE_SIZE);
  while(is.good()) {
    try {
      ret.push_back(readPoint(line));
      is.getline(line, LINE_SIZE);
    } catch(const std::invalid_argument &ia) {
      // Ran into an unknown line, return
      return ret;
    }
  }

  return ret;
}

std::pair<std::vector<Eigen::Isometry3d>, std::vector<Eigen::Isometry3d> > readTrajFile(const std::string &filename) {
  static constexpr std::size_t LINE_SIZE=512;

  std::ifstream ifs(filename);

  // Construct return values
  std::pair<std::vector<Eigen::Isometry3d>, std::vector<Eigen::Isometry3d> > ret;

  // Read first text line
  char lineBuffer[LINE_SIZE];
  ifs.getline(lineBuffer, LINE_SIZE);
  // Read points
  ret.first = getPoses(ifs);
  // Read waypoints
  ret.second = getPoses(ifs);

  // Done, return the pair
  return ret;
}

#endif
