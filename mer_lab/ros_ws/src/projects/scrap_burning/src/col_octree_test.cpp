#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"

int main() {
  octomap::ColorOcTree cot(0.1);

  cot.insertRay({0, 0, 0}, {0.3, 0.3, 0.3}, 255, 0, 0);
  cot.insertRay({0, 0, 0}, {0.4, 0.3, 0.3}, 0, 255, 0);
  cot.insertRay({0, 0, 0}, {0.5, 0.3, 0.3}, 0, 0, 255);

  cot.write("/home/fadi/scrap_ws/test_color.ot");

  return 0;
}
