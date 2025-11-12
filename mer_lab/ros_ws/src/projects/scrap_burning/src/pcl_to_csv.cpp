#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cerr << "Please provide filepath";
    return 1;
  }

  // Load file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    std::cerr << "Could not load file";
    return 2;
  }

  // Output to csv
  std::ofstream ofs{"/tmp/pclData.csv"};
  for(unsigned long i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ pt = (*cloud)[i];
    ofs << pt.x << ',' << pt.y << ',' << pt.z << '\n';
  }

  ofs.close();

  std::cout << "Wrote data to file \"/tmp/pclData.csv\"\n";

  return 0;
}
