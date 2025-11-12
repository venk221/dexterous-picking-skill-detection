// System includes
#include <string>
#include <vector>
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// scrap_burning includes
#include "cloud_processing.hpp"
#include "scrap_burning/pcl.hpp"
#include "scrap_burning/CurveFitting.h"
#include "scrap_burning/config/config.hpp"

// ncurses
#include <curses.h>

// alias declarations
using scrap_burning::pcl::PointType;
using scrap_burning::pcl::PointNormalType;
using scrap_burning::pcl::CloudType;
using scrap_burning::pcl::CloudPtr;

pcl::PointXYZ convertPointToPCL(const geometry_msgs::Point &pt) {
  return pcl::PointXYZ(pt.x, pt.y, pt.z);
}

void addCentered(const std::string& txt, int posY, int posX, bool highlight) {
  if(highlight)
    attron(A_STANDOUT);

  mvaddstr(posY, posX - txt.size() / 2, txt.c_str());

  attroff(A_STANDOUT);
}

void drawBounds(int posY, int posX, int w, int h) {
  mvvline(posY + 1, posX, '|', h);
  mvvline(posY + 1, posX + w + 1, '|', h);
  mvhline(posY, posX + 1, '-', w);
  mvhline(posY + h + 1, posX + 1, '-', w);
  mvaddch(posY, posX, '+');
  mvaddch(posY + h + 1, posX, '+');
  mvaddch(posY, posX + w + 1, '+');
  mvaddch(posY + h + 1, posX + w + 1, '+');
}

void drawBarUp(int posY, int posX, int h, int w) {
  attron(A_STANDOUT);
  for(int i = 0; i < w; ++i) {
    mvvline(posY - h + 1, posX - w / 2 + i, ' ', h);
  }
  attroff(A_STANDOUT);
}

static constexpr int BOX_WIDTH{3};

int main(int argc, char **argv) {
  ros::init(argc, argv, "skeleton_vis_node");
  ros::NodeHandle nh{};

  // We need the full cloud, leaf size, and interpolation count to fit a curve to it, check if we have the correct # of args
  if(argc != 2) {
    std::cerr << "Usage: " << argv[0] << " full_cloud\n";
    return 1;
  }
  const std::string FULL_CLOUD_PATH(argv[1]);

  // Load clouds
  CloudPtr fullCloud(new CloudType());
  if(pcl::io::loadPCDFile(FULL_CLOUD_PATH, *fullCloud) == -1) {
    std::cerr << "Failed to load cloud at " << FULL_CLOUD_PATH << '\n';
    return 2;
  }

  // Load config
  scrap_burning::ScrapBurningConfig sbc(nh);
  sbc.loadParams();
  // Config's color filters will be overriden by user

  // Setup ncurses
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);

  // These represent pointers to the data being modified
  std::array<double*, 3> dataPtrs{&sbc.filter_lower, &sbc.filter_upper, &sbc.filter_secondary_thresh};
  // There are the labels for each item
  std::array<std::string, 3> labels{"Lower", "Upper", "Secondary"};

  // Setup publisher
  ros::Publisher filteredCloudPub{nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1)};

  // Draw + filter loop
  // Store terminal sizes here
  int w, h;
  // This is the current cursor position
  std::size_t cursorPos = 0;
  // This is the character that was hit
  int ch{};
  // Whether the filter parameters changed or not
  bool changed{false};
  // When to stop execution
  bool stop{false};
  while(!stop) {
    // Reset screen
    erase();

    // Setup sizes
    getmaxyx(stdscr, h, w);

    // Reset changed flag
    changed = false;

    // This is the center of each item in the terminal
    std::array<int, 3> boxCenters{w / 4, 2 * w / 4, 3 * w / 4};

    // This is the height of the box containing the bar
    const double boxHeight(h - 4);
    // This is how many pixels/unit each bar is
    const double scale{boxHeight / 255};

    // Draw each item
    for(std::size_t i = 0; i < 3; ++i) {
      addCentered(labels[i], h - 1, boxCenters[i], cursorPos == i);
      drawBounds(1, boxCenters[i] - BOX_WIDTH / 2 - 1, BOX_WIDTH, boxHeight);

      // Draw bar representing the value of each variable
      drawBarUp(h - 3, boxCenters[i], static_cast<int>((*dataPtrs[i]) * scale), BOX_WIDTH);

      // Draw the actual value of the variable
      // They are stored as double, but cast them to an unsigned char here to better display (no decimal places)
      addCentered(std::to_string(static_cast<unsigned char>(*dataPtrs[i])), 0, boxCenters[i], false);
    }

    ch = getch();
    switch(ch) {
    case KEY_LEFT:
      cursorPos = (cursorPos + 2) % 3;
      break;
    case KEY_RIGHT:
      cursorPos = (cursorPos + 1) % 3;
      break;
    case KEY_UP:
      if(*dataPtrs[cursorPos] < 255) {
	++(*dataPtrs[cursorPos]);
	changed = true;
      }
      break;
    case KEY_DOWN:
      if(*dataPtrs[cursorPos] > 0) {
	--(*dataPtrs[cursorPos]);
	changed = true;
      }
      break;
    case 'q':			// Stop the program
      stop = true;
      break;
    case 's':			// Save the parameters to the param server
      nh.setParam("/scrap/path_follow/filter/lower", sbc.filter_lower);
      nh.setParam("/scrap/path_follow/filter/upper", sbc.filter_upper);
      nh.setParam("/scrap/path_follow/filter/secondary_threshold", sbc.filter_secondary_thresh);
      ros::spinOnce();
      break;
    default:			// Other key, ignore
      break;
    }

    if(changed) {
      CloudPtr filteredCloud = filterCloud(nh, sbc, fullCloud);
      sensor_msgs::PointCloud2 pc2 = scrap_burning::pcl::to_pc2(filteredCloud);
      pc2.header.frame_id = "world";
      filteredCloudPub.publish(pc2);
      ros::spinOnce();
    }
  }

  endwin();

  return 0;
}
