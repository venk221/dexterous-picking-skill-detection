#include <array>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>

// Test samplers
#include "conveyor_sorting/samplers.hpp"

// A reference image we will use for testing later
static cv::Mat refImg(10, 10, CV_32FC1, 1.0);

TEST(Samplers, HammersleySampler) {
  static constexpr std::size_t sampleCount{10};
  static constexpr double height{0.05};

  static const std::array<std::array<double, 3>, sampleCount>
    expectedValues{
		   std::array<double, 3>{0, 0, height},
		   std::array<double, 3>{1, 5, height},
		   std::array<double, 3>{2, 2.5, height},
		   std::array<double, 3>{3, 7.5, height},
		   std::array<double, 3>{4, 1.25, height},
		   std::array<double, 3>{5, 6.25, height},
		   std::array<double, 3>{6, 3.75, height},
		   std::array<double, 3>{7, 8.75, height},
		   std::array<double, 3>{8, 0.625, height},
		   std::array<double, 3>{9, 5.625, height},
  };

  conveyor_sorting::HammersleySampler sampler{height};

  // Make sure the height is correctly set
  EXPECT_EQ(sampler.getHeight(), height);

  // Hammersley is independant of the region vector, so we can make an empty one
  std::vector<conveyor_sorting::Region> regionVector{};

  // Test that all points are on the specified height and within the image bounds
  std::vector<geometry_msgs::Point> sampledPoints{sampler(refImg, regionVector, sampleCount)};
  // Check we got the right number of points
  EXPECT_EQ(sampledPoints.size(), sampleCount);
  for(std::size_t i = 0; i < sampledPoints.size(); ++i) {
    const geometry_msgs::Point& pt(sampledPoints[i]);

    EXPECT_FLOAT_EQ(pt.x, expectedValues[i][0]);
    EXPECT_FLOAT_EQ(pt.y, expectedValues[i][1]);
    EXPECT_FLOAT_EQ(pt.z, expectedValues[i][2]);
  }
}
TEST(Samplers, AreaWeightedRandomSampler) {
  // Create a bunch of random regions
  std::vector<conveyor_sorting::Region> regions{};
  regions.push_back({0.0,
		      {
		       {0, 0},
		       {0, 1},
		       {0, 2},
		       {1, 0}
		      }});
    regions.push_back({0.1,
		       {
			{0, 1}
		       }});

  // With 50 sampled points, we expect 40 in the first region, and 10 in the second
  static constexpr std::size_t sampleCount{50};
  static constexpr std::size_t unevenSampleCount{11}; // This is not divisible by the region sizes
  // Keep the seed constant
  static constexpr std::size_t seed{0};

  conveyor_sorting::AreaWeightedRandomSampler sampler(seed);
  std::vector<geometry_msgs::Point> sampledPoints(sampler(refImg, regions, sampleCount));

  // Filter out all the points into two bins based on height
  std::vector<geometry_msgs::Point> lowerHeight{};
  std::vector<geometry_msgs::Point> upperHeight{};
  for(std::size_t i = 0; i < sampledPoints.size(); ++i) {
    if(sampledPoints[i].z == regions[0].first)
      lowerHeight.push_back(sampledPoints[i]);
    else if(sampledPoints[i].z == regions[1].first)
      upperHeight.push_back(sampledPoints[i]);
    else			// A point is not at any valid height
      ADD_FAILURE();
  }
  // Make sure the sizes of the bins are valid
  EXPECT_EQ(lowerHeight.size(), 40);
  EXPECT_EQ(upperHeight.size(), 10);

  // Running it again after resetting the seed, we expect to get the exact same points since the seed is unchanged
  sampler.setSeed(sampler.getSeed());
  std::vector<geometry_msgs::Point> moreSampledPoints(sampler(refImg, regions, sampleCount));
  for(std::size_t i = 0; i < moreSampledPoints.size(); ++i) {
    EXPECT_FLOAT_EQ(sampledPoints[i].x, moreSampledPoints[i].x);
    EXPECT_FLOAT_EQ(sampledPoints[i].y, moreSampledPoints[i].y);
    EXPECT_FLOAT_EQ(sampledPoints[i].z, moreSampledPoints[i].z);
  }

  // Running it for a different sample count that is non-divisible produces less points
  std::vector<geometry_msgs::Point> unevenSampledPoints(sampler(refImg, regions, unevenSampleCount));
  EXPECT_LT(unevenSampledPoints.size(), unevenSampleCount);
}

// Testing sweepers
#include "conveyor_sorting/sweeping.hpp"

TEST(SweepingHelpers, expandTillBorder) {
  // Try this out with many different centers and orientations
  
}
