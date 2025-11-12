#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <sstream>
#include <fstream>

pcl::visualization::PCLVisualizer viewer("Curve Fitting 3D");

// WHY IS IT CALLED PointCloud2Vector2d IF IT CONVERTS TO A VECTOR3D!?!?!?
void
PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for(const auto &p : *cloud)
    {
      if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
	data.emplace_back(p.x, p.y, p.z);
    }
}

void
VisualizeCurve(ON_NurbsCurve &curve, double r, double g, double b, bool show_cps, const auto& data)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);

  std::vector<bool> oobIndex;
  for(std::size_t i = 0; i < cloud->size() - 1; i++)
    {
      pcl::PointXYZRGB &p1 = cloud->at(i);
      cloud_to_save->push_back(pcl::PointXYZ(p1.x, p1.y, p1.z));
      pcl::PointXYZRGB &p2 = cloud->at(i + 1);
      std::ostringstream os;
      os << "line_" << r << "_" << g << "_" << b << "_" << i;
      double dx, dy, dz;
      double min_dist = 0.01;
      for(int i = 0; i < data.interior.size(); ++i) {
	dx = data.interior[i][0] - p1.x;
	dy = data.interior[i][1] - p1.y;
	dz = data.interior[i][2] - p1.z;
	oobIndex.push_back(dx*dx+dy*dy+dz*dz <= min_dist*min_dist);
	if(dx*dx+dy*dy+dz*dz <= min_dist*min_dist) {
	  // std::cout << "Found " << t << " inside the bounds\n";
	  // std::cout << "Point " << p1.x << ", " << p1.y << ", " << p1.z << "\n";
	  
	  viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
	  break;
	}
      }
      // viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
    }

  if(show_cps)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
      for(int i = 0; i < curve.CVCount(); i++)
	{
	  ON_3dPoint cp;
	  curve.GetCV(i, cp);

	  pcl::PointXYZ p;
	  p.x = float(cp.x);
	  p.y = float(cp.y);
	  p.z = float(cp.z);
	  cps->push_back(p);
	}
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 5 * g, 5 * b);
      viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
    }
  // Save cloud
  pcl::io::savePCDFileASCII("/home/fadi/Desktop/cheat_file.pcd", *cloud_to_save);
}

inline ON_3dPoint vecToOn(const Eigen::Vector3d& in_vec) {
  return ON_3dPoint(in_vec[0], in_vec[1], in_vec[2]);
}

int main(int argc, char *argv[]) {
  std::string pcd_file;
  int ctrl_pts = 20;		// Default is the one from the examples file
  int order = 4;		// Default is the one from the examples file
  double smoothness = 1.0;	// Not default, that one kinda sucks
  double r = 100.0;		// Default is the one from the examples file
  double min_cutoff_dist = 0.01;

  if(argc > 1)
    {
      pcd_file = argv[1];
    }
  else
    {
      printf("\nUsage: pcl_example_nurbs_fitting_curve3d pcd-file [control-points] [order]\n\n");
      printf("  pcd-file         point-cloud file\n");
      printf("  control-points   number of control points(OPTIONAL)\n");
      printf("  order            curve order(OPTIONAL)\n");
      printf("  smoothness       curve smoothness(OPTIONAL)\n");
      printf("  r                curve initial radius(OPTIONAL)\n");
      exit(0);
    }

  // Keep it semantically similar to above conditionals
  if(argc > 2) {
    std::istringstream iss(argv[2]);
    iss >> ctrl_pts;
  }
  if(argc > 3) {
    std::istringstream iss(argv[3]);
    iss >> order;
  }
  if(argc > 4) {
    std::istringstream iss(argv[4]);
    iss >> smoothness;
  }
  if(argc > 5) {
    std::istringstream iss(argv[5]);
    iss >> r;
  }

  // #################### LOAD FILE #########################
  printf("  loading %s\n", pcd_file.c_str());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud2;

  if(pcl::io::loadPCDFile(pcd_file, cloud2) == -1)
    throw std::runtime_error("  PCD file not found.");

  fromPCLPointCloud2(cloud2, *cloud);

  // convert to NURBS data structure
  pcl::on_nurbs::NurbsDataCurve data;
  PointCloud2Vector2d(cloud, data.interior);
  std::cout << "Cloud size is " << data.interior.size() << "\n";

  viewer.setSize(800, 600);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloudbase");

  // #################### CURVE PARAMETERS #########################
  unsigned n_control_points(ctrl_pts);
  std::cout << "Fitting with " << n_control_points << " control pts\n";

  pcl::on_nurbs::FittingCurve::Parameter curve_params;
  curve_params.smoothness = smoothness;

  // #################### CURVE FITTING #########################
  // printf("Fitting curve");
  ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA(order, data.interior, n_control_points, r);
  std::cout << "Created curve with order " << order << "\n";

  // curve = ON_NurbsCurve(3, false, order, ctrl_pts);
  // curve = ON_MakeClampedUniformKnotVector
  // if(curve.MakePeriodicUniformKnotVector(1.0 /(ctrl_pts - order + 1)))
  //   std::cout << "Made periodic knot vector\n";
  // curve.MakeClampedUniformKnotVector(0.0);
  // std::cout << 1.0 /(ctrl_pts - order + 1) << " clamped arg\n";
  std::cout << curve.IsClosed() << std::endl;

  pcl::on_nurbs::FittingCurve fit(&data, curve);
  // fit.refine();
  // fit.refine();
  // fit.refine();
  fit.assemble(curve_params);
  fit.solve();

  // visualize
  VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, true, data);
  viewer.spin();

  return 0;
}

