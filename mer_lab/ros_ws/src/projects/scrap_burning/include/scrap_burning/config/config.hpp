#ifndef SCRAP_BURNING_CONFIG_HPP
#define SCRAP_BURNING_CONFIG_HPP

#include <vector>
#include <memory>
#include <string>
#include <string_view>

#include <ros/ros.h>

namespace scrap_burning {

  template <typename T>
  class LabeledEnum {
  public:
    // String conversion functions
    template <typename E>
    std::string toString() {
      return T::labels[static_cast<std::size_t>(T::type)];
    }
    static T fromString(const std::string &str) {
      auto iter = std::find(T::labels.begin(), T::labels.end(), str);
      if(iter != T::labels.end())
	return static_cast<typename T::Type>(std::distance(T::labels.begin(), iter));

      else
	return T(T::UNKNOWN);
    }

    // Comparison functions
    bool operator==(T other) {
      return static_cast<T*>(this)->type == other.type;
    }
    bool operator!=(T other) {
      return this->operator==(other);
    }
  };

  class CurveFollowMethod : public LabeledEnum<CurveFollowMethod> {
  public:
    enum Type {PCL_NURBS, SKELETON, ACTIVE_VISION, UNKNOWN};
    inline static const std::array<std::string, 3> labels {"pcl_nurbs", "skeleton", "active"};

    CurveFollowMethod(Type t) : type(t) {}
    Type type;
  };

  class NoiseType : public LabeledEnum<NoiseType> {
  public:
    enum Type {NONE, GAUSSIAN, UNIFORM, UNKNOWN};
    inline static const std::array<std::string, 3> labels{"none", "gaussian", "uniform"};

    NoiseType(Type t) : type(t) {}
    Type type;
  };

  class ScrapBurningConfig {
  public:
    ScrapBurningConfig(ros::NodeHandle &nh)
      : _nh(nh) {}

    bool loadParams() {
      bool ret = true;

      // Meta config
      ret = ret & _load_param("/scrap/path_follow/service_wait_timeout", service_wait_timeout);

      // Noise
      ret = ret & _loadNoiseConfig();

      // Initial joints
      ret = ret & _loadInitJointValsConfig();

      // Misc camera stuff
      ret = ret & _load_param("/scrap/path_follow/depth_topic", depth_topic);
      ret = ret & _load_param("/scrap/path_follow/normal_nn_count", normal_nn_count);

      // Outlier removal
      ret = ret & _loadOutlierRemovalConfig();

      // Path following
      ret = ret & _loadPathFollowingConfig();

      // Filter config
      ret = ret & _loadFilterConfig();

      // Downsampling
      ret = ret & _load_param("/scrap/path_follow/downsample_leaf_size", downsample_leaf_size);

      // Curve fitting params
      ret = ret & _loadCurveFittingConfig();

      // Visualization params
      ret = ret & _loadVisualizationConfig();

      // Interactive execution
      ret = ret & _load_param("/scrap/path_follow/interactive_execution", interactive_execution);

      // Saving
      ret = ret & _loadSavingConfig();

      // Other
      ret = ret & _load_param("/scrap/path_follow/reverse_normal", reverse_normal);
      ret = ret & _load_param("/scrap/path_follow/sample_execution", sample_execution);

      return ret;
    }
    
    // Config variables
    double service_wait_timeout{-1}; // Default is to wait indefinitely
    std::vector<double> initial_joint_vals{0.0, -0.78, 0.0, -0.78, 0.0, 0.78, 0.785398};
    std::string depth_topic{"/panda_camera/depth/points"};
    std::string image_topic{"/panda_camera/depth/image_raw"};
    std::string noise_depth_topic{"/noisy/depth"};
    std::string noise_rgb_topic{"/noisy/color"};
    std::string noise_type{"none"};
    bool is_relative{false};
    double mean{0.0};
    double stddev{0.005};
    double cutoff{1.0};
    double left{0.0};
    double right{1.0};
    int normal_nn_count{50};
    int mean_k{50};
    double stddev_thresh{1.0};
    std::string filter_field{"r"};
    double filter_lower{10};
    double filter_upper{255};
    double filter_secondary_thresh{20};
    double downsample_leaf_size{0.0}; // Do not downsample by default
    CurveFollowMethod followMethod{CurveFollowMethod::PCL_NURBS};
    // Active vision stuff
    double octomap_res{0.05};
    int camera_y{320};
    int camera_x{240};
    double camera_focal_length{554.25469};
    double ray_length{0.7};
    int frontier_bounding_box{10};
    int threads{2};
    bool optimized{true};
    // Active vision stuff
    int control_pts{20};
    double smoothness{100.0};
    double radius{10.0};
    double min_cutoff_dist{0.01};
    double skel_leaf_size{0.005};
    bool view_curve{true};
    bool view_target{true};
    bool view_target_norm{true};
    bool view_visited{true};
    bool view_segmented_curve{true};
    bool interactive_execution{true};
    bool save_filtered{false};
    std::string save_filtered_path{"/tmp/filtered.pcd"};
    double distance_from_scrap{0.15};
    double path_end_threshold{0.001};
    double planning_time{3.0};
    bool reverse_normal{false};
    bool sample_execution{false};
    bool save_points{false};
    std::string save_points_path{"/tmp/points.csv"};
    std::string save_global_skel_path{"/tmp/skel_pts.csv"};
    std::string save_octomap_path{"/home/fadi/scrap_ws/latest.ot"};
  private:
    ros::NodeHandle &_nh;

    template <typename ParamType>
    bool _load_param(const std::string &path, ParamType &pt) {
      if(!_nh.getParam(path, pt)) {
	ROS_WARN_STREAM("Could not retrieve " << path);
	return false;
      }
      return true;
    }

    bool _loadNoiseConfig() {
      bool ret = true;

      // If no noise is specified, stop here
      if(!_load_param("/scrap/add_noise/noise_type", noise_type))
	return false;

      // If a noise is specified as none, stop here
      if(noise_type == "none")
	return true;

      // If a noise is specified but not a legal value, stop here
      if(!(noise_type == "gaussian" || noise_type == "uniform")) {
	ROS_WARN_STREAM("Invalid noise type " << noise_type);
	return false;
      }

      // Load all other parameters
      ret = ret & _load_param("/scrap/add_noise/image_topic", image_topic);
      ret = ret & _load_param("/scrap/add_noise/noise_depth_topic", noise_depth_topic);
      ret = ret & _load_param("/scrap/add_noise/noise_rgb_topic", noise_rgb_topic);
      ret = ret & _load_param("/scrap/add_noise/is_relative", is_relative);

      // Load noise_type-specific parameters
      if(noise_type == "gaussian") {
	ret = ret & _load_param("/scrap/add_noise/gaussian/mean", mean);
	ret = ret & _load_param("/scrap/add_noise/gaussian/stddev", stddev);
	ret = ret & _load_param("/scrap/add_noise/gaussian/cutoff", cutoff);
      }
      else if(noise_type == "uniform") {
	ret = ret & _load_param("/scrap/add_noise/uniform/left", left);
	ret = ret & _load_param("/scrap/add_noise/uniform/right", right);
      }

      return ret;
    }

    bool _loadInitJointValsConfig() {
      std::vector<double> temp_joint_vals;
      if(!_nh.getParam("/scrap/path_follow/initial_joint_config", temp_joint_vals)) { // None found
	ROS_WARN_STREAM("Could not retrieve /scrap/path_follow/initial_joint_config, loading defaults");
	return false;
      }

      if(temp_joint_vals.size() != 7) { // Incorrect size
	ROS_WARN_STREAM("Incorrect size of joint values, expected 7, got " << temp_joint_vals.size() <<
			", loading defaults");
	return false;
      }

      // Worked fine, load those values
      initial_joint_vals.swap(temp_joint_vals);
      return true;
    }

    bool _loadOutlierRemovalConfig() {
      bool ret = true;

      ret = ret & _load_param("/scrap/path_follow/outlier_removal/mean_k", mean_k);
      ret = ret & _load_param("/scrap/path_follow/outlier_removal/stddev_thresh", stddev_thresh);

      return ret;
    }

    bool _loadPathFollowingConfig() {
      bool ret = true;

      ret = ret & _load_param("/scrap/path_follow/distance_from_scrap", distance_from_scrap);
      ret = ret & _load_param("/scrap/path_follow/path_end_threshold", path_end_threshold);
      ret = ret & _load_param("/scrap/path_follow/planning_time", planning_time);

      return ret;
    }

    bool _loadFilterConfig() {
      bool ret = true;

      // TODO: Should check if this is in [r|g|b]
      ret = ret & _load_param("/scrap/path_follow/filter/color", filter_field);
      // TODO: For both lower, upper, and threshold, we should probably check if it is between 0 and 255
      ret = ret & _load_param("/scrap/path_follow/filter/lower", filter_lower);
      ret = ret & _load_param("/scrap/path_follow/filter/upper", filter_upper);
      ret = ret & _load_param("/scrap/path_follow/filter/secondary_threshold", filter_secondary_thresh);

      return ret;
    }

    bool _loadCurveFittingConfig() {
      // Load method
      std::string strMethod;
      _load_param("/scrap/path_follow/use_method", strMethod);
      CurveFollowMethod method = CurveFollowMethod::fromString(strMethod);

      // Invalid method
      if(method == CurveFollowMethod::UNKNOWN) {
	ROS_WARN_STREAM("Invalid use_method " << strMethod);
	return false;
      }
      
      // Set new method
      followMethod = method;

      bool ret = true;
      switch(followMethod.type) {
      case CurveFollowMethod::PCL_NURBS:
	ret = ret & _load_param("/scrap/path_follow/curve_fitting/control_pts", control_pts);
	ret = ret & _load_param("/scrap/path_follow/curve_fitting/smoothness", smoothness);
	ret = ret & _load_param("/scrap/path_follow/curve_fitting/radius", radius);
	ret = ret & _load_param("/scrap/path_follow/curve_fitting/min_cutoff_dist", min_cutoff_dist);
	return ret;
      case CurveFollowMethod::SKELETON:
	ret = ret & _load_param("/scrap/path_follow/curve_fitting/skel_leaf_size", skel_leaf_size);
	return ret;
      default:			// Active vision
	ret = ret & _load_param("/scrap/path_follow/active_vision/octomap_res", octomap_res);
	ret = ret & _load_param("/scrap/path_follow/active_vision/camera/res_x", camera_x);
	ret = ret & _load_param("/scrap/path_follow/active_vision/camera/res_y", camera_y);
	ret = ret & _load_param("/scrap/path_follow/active_vision/camera/focal_length", camera_focal_length);
	ret = ret & _load_param("/scrap/path_follow/active_vision/ray_length", ray_length);
	ret = ret & _load_param("/scrap/path_follow/active_vision/frontier_bounding_box", frontier_bounding_box);
	ret = ret & _load_param("/scrap/path_follow/active_vision/threads", threads);
	ret = ret & _load_param("/scrap/path_follow/active_vision/optimized", optimized);
	return ret;
      }
    }

    bool _loadVisualizationConfig() {
      bool ret = true;


      ret = ret & _load_param("/scrap/path_follow/visualization/view_curve", view_curve);
      ret = ret & _load_param("/scrap/path_follow/visualization/view_target", view_target);
      ret = ret & _load_param("/scrap/path_follow/visualization/view_target_norm", view_target_norm);
      ret = ret & _load_param("/scrap/path_follow/visualization/view_visited", view_visited);
      ret = ret & _load_param("/scrap/path_follow/visualization/view_segmented_curve", view_segmented_curve);

      return ret;
    }

    bool _loadSavingConfig() {
      bool ret = true;

      ret = ret & _load_param("/scrap/path_follow/save_filtered", save_filtered);
      ret = ret & _load_param("/scrap/path_follow/save_filtered_path", save_filtered_path);
      ret = ret & _load_param("/scrap/path_follow/save_points", save_points);
      ret = ret & _load_param("/scrap/path_follow/save_points_path", save_points_path);
      ret = ret & _load_param("/scrap/path_follow/save_global_skel_path", save_global_skel_path);
      ret = ret & _load_param("/scrap/path_follow/save_octomap_path", save_octomap_path);

      return ret;
    }

    static std::string _concat(std::string a, const std::string &b) {
      static constexpr char SEPARATOR='/';
      
      // Check sizes to ensure we're not concatenating with any empties
      if(a.size() == 0)      return b;
      else if(b.size() == 0) return a;
      
      // Check if we have a SEPARATOR between a and b
      if(a.back() != SEPARATOR || b[0] != SEPARATOR)
	a.push_back(SEPARATOR);
	
      // Concatenate
      a.insert(a.end(), b.begin(), b.end());
      
      return a;
    }
  };

  class Config {
  public:
    Config(ros::NodeHandle &nh)
      : _nh(nh) {}
    virtual ~Config() {}

    virtual void reloadParams(std::string_view param) = 0;
  protected:
    // Helper method to load a single parameter
    template <typename T>
    bool loadParam(std::string_view name, T &t) {
      return _nh.getParam(name, t);
    }

    
  private:
    ros::NodeHandle &_nh;
  };

  // A config object containing only a single value
  template <typename T>
  class ScalarConfig : public Config {
  public:
    ScalarConfig(ros::NodeHandle &nh)
      : Config(nh) {}

    operator T() const { return _t; }

    virtual void reloadParams(std::string_view param) override {
      loadParam(param, _t);
    }
  private:
    T _t;
  };

  class NestedConfig : public Config {
  public:
    NestedConfig(ros::NodeHandle &nh, const std::unordered_map<std::string, std::shared_ptr<Config> > &kvPairs)
      : Config(nh), _kvPairs(kvPairs) {}
    virtual ~NestedConfig() {}

    virtual void reloadParams(std::string_view param) override {
      bool ret = true;
      // for(const auto &kv : _kvPairs)
      // 	kv.second->reloadParams(concat(param, kv.first));
    }

    std::shared_ptr<Config> operator[](const std::string &key) {
      auto iter = _kvPairs.find(key);
      if(iter == _kvPairs.end()) return std::shared_ptr<Config>();
      return iter->second;
    }
  private:
    std::unordered_map<std::string, std::shared_ptr<Config> > _kvPairs;
  };

} // end of namespace scrap_burning

#endif	// end of SCRAP_BURNING_CONFIG_HPP
