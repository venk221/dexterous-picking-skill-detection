#include <string>
#include <random>
#include <vector>
#include <fstream>
#include <sstream>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/SpawnModel.h>

#include "conveyor_sorting/utils.hpp"
#include "conveyor_sorting/config.hpp"
#include "conveyor_sorting/pt_utils.hpp"

#include <qnd/geom.hpp>
#include <qnd/comm.hpp>

static constexpr char packageName[]{"conveyor_sorting"};
static constexpr char modelFolder[]{"pile_models"};

static constexpr char spawnTopic[]{"/gazebo/spawn_urdf_model"};

// Random number facilities
static std::random_device rd{};
static std::mt19937 randomGen{rd()};

template <typename T>
const T& getRandElem(const std::vector<T>& list) {
  std::uniform_int_distribution<std::size_t> dist(0, list.size() - 1);
  return list[dist(randomGen)];
}

std::string randomURDF(const geometry_msgs::Point& minSize, const geometry_msgs::Point& maxSize,
		       const double minMass, const double maxMass) {
  static std::vector<std::string> colorOptions{"Gazebo/Red", "Gazebo/Green", "Gazebo/Blue"};
  std::uniform_real_distribution<double> xDist(minSize.x, maxSize.x);
  std::uniform_real_distribution<double> yDist(minSize.y, maxSize.y);
  std::uniform_real_distribution<double> zDist(minSize.z, maxSize.z);
  std::uniform_real_distribution<double> mDist(minMass,   maxMass);

  geometry_msgs::Point randShape;
  randShape.x = xDist(randomGen);
  randShape.y = yDist(randomGen);
  randShape.z = zDist(randomGen);

  double randMass{mDist(randomGen)};

  return createRectangleURDF(randShape, randMass, "floating", getRandElem(colorOptions));
}

// Staggered random pose generator, generates random poses each higher than the last
class StaggeredRPG {
public:
  StaggeredRPG(const geometry_msgs::Point& center, double w, double l, double heightDiff)
    : _rX{center.x - w / 2, center.x + w / 2},
      _rY{center.y - l / 2, center.y + l / 2},
      _heightDiff(heightDiff), _nextHeight{center.z + _heightDiff} {}
  geometry_msgs::Pose operator()() {
    geometry_msgs::Pose ret;

    ret.orientation.w = 1.0;

    ret.position.x = _rX(randomGen);
    ret.position.y = _rY(randomGen);
    ret.position.z = _nextHeight;

    // Increment the height
    _nextHeight += _heightDiff;

    return ret;
  }
private:
  geometry_msgs::Point _center;
  std::uniform_real_distribution<double> _rX;
  std::uniform_real_distribution<double> _rY;
  double _heightDiff;
  double _nextHeight;
};

class ObjectNameGenerator {
public:
  ObjectNameGenerator(const std::string& prefix)
    : _prefix{prefix}, _spawnIdx{0} {}

  std::string operator()() {
    std::ostringstream oss;

    oss << _prefix << _spawnIdx;

    _spawnIdx++;

    return oss.str();
  }
private:
  std::string _prefix;
  std::size_t _spawnIdx;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pile_spawner_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    std::cerr << "Failed to load all params\n";
    return 1;
  }

  // Parse params
  const TableDescConfig& table{cfgLoader.getConfig().table_desc};
  const PileSpawnerConfig& pile{cfgLoader.getConfig().pile_spawner};
  geometry_msgs::Point spawnCenter{qnd::createPt(table.center.x,
						 table.center.y,
						 table.center.z + table.size.z / 2)};
  double width{pile.pile_width};
  double length{pile.pile_length};
  double heightDiff{pile.pile_height_diff};
  int itemCountMin{pile.pile_items.min};
  int itemCountMax{pile.pile_items.max};

  // Minimum and maximum cube sizes
  geometry_msgs::Point minSize{qnd::createPt(0.05)}, maxSize{qnd::createPt(0.1)};
  // Minimum and maximum masses
  double minMass{0.05}, maxMass{0.07};

  // Spawn random objects in the target location
  // Setup services
  ros::ServiceClient spawnClient{*qnd::serviceClient<gazebo_msgs::SpawnModel>(nh, spawnTopic)};

  // Create walls
  std::cout << "Spawning temporary walls\n";
  static constexpr double wallHeight{0.7};
  static constexpr double wallThickness{0.01};
  // To avoid clipping into the table
  static constexpr double wallHeightAllowance{0.03};
  std::string nsWall{createRectangleURDF(qnd::createPt(wallThickness, length + maxSize.y - wallThickness * 2, wallHeight),
					 1000.0, "fixed")};
  std::string weWall{createRectangleURDF(qnd::createPt(width + maxSize.x - wallThickness * 2, wallThickness, wallHeight),
					 1000.0, "fixed")};
  geometry_msgs::Pose nWallPose{}, sWallPose{}, wWallPose{}, eWallPose{};
  nWallPose.orientation.w = 1.0;
  nWallPose.position.x = spawnCenter.x + length / 2 + wallThickness / 2 + maxSize.x / 2;
  nWallPose.position.y = spawnCenter.y;
  nWallPose.position.z = spawnCenter.z + wallHeight / 2 + wallHeightAllowance;
  sWallPose = nWallPose;
  sWallPose.position.x -= (length + maxSize.y);
  wWallPose = nWallPose;
  wWallPose.position.x = spawnCenter.x;
  wWallPose.position.y = spawnCenter.y + width / 2 + wallThickness / 2 + maxSize.y / 2;
  eWallPose = wWallPose;
  eWallPose.position.y -= (width + maxSize.x);
  if(!spawnMesh(spawnClient, "North Wall", nsWall, nWallPose) ||
     !spawnMesh(spawnClient, "South Wall", nsWall, sWallPose) ||
     !spawnMesh(spawnClient, "West Wall",  weWall, wWallPose) ||
     !spawnMesh(spawnClient, "East Wall",  weWall, eWallPose)) {
    std::cerr << "Failed to spawn walls\n";
    return 2;
  }

  // Create random pose generator
  StaggeredRPG poseGenerator{spawnCenter, width, length, heightDiff};
  // Create random name generator
  ObjectNameGenerator nameGenerator{"Pile Object "};
  // Pick a random number of objects to spawn
  int spawnCount{getRandElem(std::vector<int>{itemCountMin, itemCountMax})};
  for(int i = 0; i < spawnCount; ++i) {
    std::cout << "Spawning object " << i << '\n';
    if(!spawnMesh(spawnClient,
		  nameGenerator(),
		  randomURDF(minSize, maxSize, minMass, maxMass),
		  poseGenerator())) {
      std::cerr << "Failed to spawn object in gazebo\n";
      return 3;
    }
  }

  return 0;
}
