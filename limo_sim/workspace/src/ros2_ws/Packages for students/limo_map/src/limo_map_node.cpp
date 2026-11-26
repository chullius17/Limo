#include "limo_map/node.hpp"
#include "limo_map/cloud.hpp"
#include "limo_map/map_io.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#ifdef LIMO_MAP_HAS_CARTOGRAPHER
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#endif

namespace limo_map
{
namespace
{
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Cloud = sensor_msgs::msg::PointCloud2;
using Trigger = std_srvs::srv::Trigger;
using Steady = std::chrono::steady_clock;

template<typename Translation, typename Rotation>
Pose planar_pose(const Translation & p, const Rotation & q)
{
  for (double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite transform");}
  }
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (norm < 1e-9) {throw std::invalid_argument("Zero quaternion");}
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  if (std::abs(2 * (w * y - z * x)) > 0.05 || std::abs(2 * (w * x + y * z)) > 0.05 ||
    1 - 2 * (x * x + y * y) < 0)
  {
    throw std::invalid_argument("Mapping requires planar frames with upright Z");
  }
  return {p.x, p.y, std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
}
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}
#ifdef LIMO_MAP_HAS_CARTOGRAPHER
// Some Cartographer releases expose is_frozen; Foxy's official SubmapEntry
// contains only IDs/version/pose. In that case use the latest submap of the
// selected trajectory, which must be the active mapping trajectory.
template<typename Entry>
auto frozen(const Entry & entry, int) -> decltype(static_cast<bool>(entry.is_frozen))
{
  return entry.is_frozen;
}
template<typename Entry>
bool frozen(const Entry &, long) {return false;}
#endif
}  // namespace

class MapNode : public rclcpp::Node
{
public:
  explicit MapNode(const rclcpp::NodeOptions & node_options) : Node("limo_map", node_options)
  {
    Options options;
    options.class_ids = declare_parameter("class_ids", options.class_ids);
    options.class_costs = declare_parameter("class_costs", options.class_costs);
    // Independent evidence is the generic default; the shipped profile groups aliases.
    options.evidence_groups = declare_parameter("evidence_groups", std::vector<int64_t>{});
    options.resolution = positive("resolution", options.resolution);
    options.hit = positive("hit_log_odds", options.hit);
    options.miss = positive("miss_log_odds", options.miss);
    options.limit = positive("log_odds_limit", options.limit);
    options.threshold = positive("min_evidence", options.threshold);
    options.free_confirmations = integer("free_confirmations", 1, 1, 255);
    options.max_cells = integer("max_cells", options.max_cells, 1024);
    options.max_output_cells = integer("max_output_cells", options.max_output_cells, 1);
    grid_ = std::make_unique<Grid>(options);
    map_frame_ = nonempty("map_frame", "map");
    odom_frame_ = nonempty("odom_frame", "odom");
    pose_source_ = nonempty("pose_source", "tf");
    if (pose_source_ != "tf" && pose_source_ != "cartographer") {
      throw std::invalid_argument("pose_source must be tf or cartographer");
    }
    cloud_topic_ = nonempty("cloud_topic", "/limo/cv_package/visual_ptcld/points");
    reference_topic_ = nonempty("reference_map_topic", "/map");
    auto prefix = nonempty("output_prefix", "/limo/map_package/offline");
    while (prefix.size() > 1 && prefix.back() == '/') {prefix.pop_back();}
    if (prefix == "/") {prefix.clear();}
    tf_wait_ = positive("tf_wait_sec", 0.5);
    submap_max_age_ = positive("submap_max_age_sec", 2.0);
    max_points_ = integer("max_input_points", 100000, 1);
    trajectory_id_ = integer("trajectory_id", 0, 0, std::numeric_limits<int>::max());
    median_kernel_ = integer("save_median_kernel", 1, 1, 255);
    if (median_kernel_ % 2 == 0) {throw std::invalid_argument("save_median_kernel must be odd");}
    free_max_ = integer("obstacle_free_max_cost", 39, 0, 99);
    occupied_max_ = integer("obstacle_occupied_max_cost", 95, free_max_ + 1, 100);
    save_directory_ = nonempty("save_directory", "ros2_maps/semantic");
    save_name_ = nonempty("save_map_name", "limo_map");
    const double rate = positive("publish_rate_hz", 4.0);
    const auto submap_topic = nonempty("submap_topic", "/submap_list");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    cloud_sub_ = create_subscription<Cloud>(cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&MapNode::cloud_callback, this, std::placeholders::_1));
    reference_sub_ = create_subscription<OccupancyGrid>(reference_topic_, map_qos,
      std::bind(&MapNode::map_callback, this, std::placeholders::_1));
    semantic_pub_ = create_publisher<OccupancyGrid>(prefix + "/map/semantic_grid", map_qos);
    combined_pub_ = create_publisher<OccupancyGrid>(prefix + "/map/combined_grid", map_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(prefix + "/map_saver/status", map_qos);
    for (auto id : options.class_ids) {
      class_pubs_.emplace(static_cast<uint8_t>(id), create_publisher<OccupancyGrid>(
          prefix + "/map/class_" + std::to_string(id) + "_map", map_qos));
    }
    reset_service_ = create_service<Trigger>(prefix + "/reset_map",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        reset();
        response->success = true;
        response->message = "Semantic evidence reset; SLAM laser map retained";
        status(response->message);
      });
    save_service_ = create_service<Trigger>(prefix + "/map_saver/save_map",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try {
          if (!laser_) {throw std::runtime_error("Waiting for laser map on " + reference_topic_);}
          const auto raster = grid_->render(geometry_);
          save_maps(save_directory_, save_name_, geometry_, raster.semantic, laser_->data,
            median_kernel_, free_max_, occupied_max_);
          response->success = true;
          response->message = "Saved complete, laser and CV obstacle maps in " +
            std::filesystem::absolute(save_directory_).string();
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
        status(response->message);
      });
    if (pose_source_ == "tf") {grid_->set_pose({0, 0}, {});}
    else {
#ifdef LIMO_MAP_HAS_CARTOGRAPHER
      submaps_sub_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
        submap_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MapNode::submaps_callback, this, std::placeholders::_1));
#else
      throw std::runtime_error("Cartographer mode requires rebuilding with cartographer_ros_msgs installed");
#endif
    }
    retry_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&MapNode::consume, this));
    publish_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)), std::bind(&MapNode::publish_maps, this));
    // This node's parameters describe persistent evidence and subscriptions;
    // require restart rather than reporting successful but ineffective updates.
    parameters_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "Mapping parameters are startup-only; restart the node to apply changes";
        return result;
      });
    status("LIMO map ready: " + std::to_string(options.class_ids.size()) +
      " configured input classes; pose_source=" + pose_source_);
    if (pose_source_ == "tf") {
      RCLCPP_INFO(get_logger(), "Historical semantic geometry follows loop closures only in Cartographer mode");
    }
  }

private:
  double positive(const std::string & name, double value)
  {
    value = declare_parameter<double>(name, value);
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }
  int64_t integer(
    const std::string & name, int64_t value, int64_t minimum,
    int64_t maximum = std::numeric_limits<int64_t>::max())
  {
    value = declare_parameter<int64_t>(name, value);
    if (value < minimum || value > maximum) {throw std::invalid_argument(name + " outside valid range");}
    return value;
  }
  std::string nonempty(const std::string & name, const std::string & value)
  {
    const auto result = declare_parameter<std::string>(name, value);
    if (result.empty()) {throw std::invalid_argument(name + " must be nonempty");}
    return result;
  }
  void status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }
  void warning(const std::string & text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", text.c_str());
  }
  void cloud_callback(Cloud::ConstSharedPtr msg)
  {
    const auto stamp = stamp_ns(msg->header.stamp);
    if (stamp <= 0 || msg->header.frame_id.empty()) {
      warning("Cloud requires a positive sensor timestamp and frame_id"); return;
    }
    const auto newest = pending_ ? std::max(last_stamp_, stamp_ns(pending_->header.stamp)) : last_stamp_;
    if (stamp <= newest) {warning("Ignoring duplicate/out-of-order cloud; reset_map before bag rewind"); return;}
    if (static_cast<uint64_t>(msg->width) * msg->height > max_points_) {
      warning("Cloud exceeds max_input_points"); return;
    }
    pending_ = std::move(msg);
    received_ = Steady::now();
    consume();
  }
  std::pair<Submap, Pose> cloud_pose(const Cloud & msg)
  {
    const auto stamp = rclcpp::Time(msg.header.stamp);
    if (pose_source_ == "tf") {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, msg.header.frame_id, stamp);
      return {{0, 0}, planar_pose(tf.transform.translation, tf.transform.rotation)};
    }
    if (!has_submap_stamp_ || submaps_.empty()) {throw std::runtime_error("Waiting for Cartographer submaps");}
    if (std::abs(stamp_ns(msg.header.stamp) - stamp_ns(submap_stamp_)) * 1e-9 > submap_max_age_) {
      throw std::runtime_error("Cartographer submap timestamp is too far from cloud");
    }
    auto selected = submaps_.end();
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
      if (!it->second && it->first.first == trajectory_id_) {selected = it;}
    }
    if (selected == submaps_.end()) {throw std::runtime_error("No active submap for selected trajectory");}
    const auto tf = tf_buffer_->lookupTransform(map_frame_, rclcpp::Time(submap_stamp_),
      msg.header.frame_id, stamp, odom_frame_);
    return {selected->first, planar_pose(tf.transform.translation, tf.transform.rotation)};
  }
  void consume()
  {
    if (!pending_) {return;}
    std::pair<Submap, Pose> selected;
    try {selected = cloud_pose(*pending_);}
    catch (const std::exception & error) {
      if (std::chrono::duration<double>(Steady::now() - received_).count() > tf_wait_) {
        warning(std::string("Dropping cloud: ") + error.what());
        pending_.reset();
      }
      return;
    }
    auto msg = std::move(pending_);
    // Mark consumed even on malformed data: a duplicate never becomes new evidence.
    last_stamp_ = stamp_ns(msg->header.stamp);
    try {
      const auto points = read_cloud(*msg, *grid_, selected.second, grid_->pose(selected.first));
      dirty_ = grid_->update(points, selected.first) || dirty_;
    } catch (const std::exception & error) {warning(error.what());}
  }
  void map_callback(OccupancyGrid::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != map_frame_) {warning("Reference map frame differs from map_frame"); return;}
    try {
      Geometry geometry{msg->info.resolution, msg->info.width, msg->info.height,
        planar_pose(msg->info.origin.position, msg->info.origin.orientation)};
      geometry.validate(grid_->options().max_output_cells);
      if (msg->data.size() != geometry.size()) {throw std::invalid_argument("Invalid reference map length");}
      for (auto value : msg->data) {
        if (value < -1 || value > 100) {throw std::invalid_argument("Invalid reference map cost");}
      }
      geometry_ = geometry;
      laser_ = std::move(msg);
      dirty_ = true;
    } catch (const std::exception & error) {warning(error.what());}
  }
  OccupancyGrid message(const std::vector<int8_t> & data, const rclcpp::Time & stamp)
  {
    OccupancyGrid msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = stamp;
    msg.info = laser_->info;
    msg.data = data;
    return msg;
  }
  void publish_maps()
  {
    if (!dirty_ || !laser_) {return;}
    try {
      const auto raster = grid_->render(geometry_);
      const auto stamp = now();
      semantic_pub_->publish(message(raster.semantic, stamp));
      combined_pub_->publish(message(combine(raster.semantic, laser_->data), stamp));
      // Reuse one dense buffer for all per-class layers. Aliases share a layer.
      std::vector<int8_t> layer(geometry_.size());
      for (const auto & publisher : class_pubs_) {
        const auto group = grid_->group(publisher.first);
        for (size_t i = 0; i < layer.size(); ++i) {
          layer[i] = raster.groups[i] < 0 ? -1 : (raster.groups[i] == group ? raster.semantic[i] : 0);
        }
        publisher.second->publish(message(layer, stamp));
      }
      dirty_ = false;
    } catch (const std::exception & error) {warning(error.what());}
  }
  void reset()
  {
    grid_ = std::make_unique<Grid>(grid_->options());
    if (pose_source_ == "tf") {grid_->set_pose({0, 0}, {});}
    submaps_.clear();
    has_submap_stamp_ = false;
    pending_.reset();
    last_stamp_ = -1;
    dirty_ = true;
    publish_maps();
  }
#ifdef LIMO_MAP_HAS_CARTOGRAPHER
  void submaps_callback(cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != map_frame_) {warning("SubmapList frame differs from map_frame"); return;}
    if (has_submap_stamp_ && stamp_ns(msg->header.stamp) < stamp_ns(submap_stamp_)) {return;}
    try {
      std::map<Submap, Pose> poses;
      std::map<Submap, bool> submaps;
      std::set<Submap> visible;
      for (const auto & entry : msg->submap) {
        const Submap key{entry.trajectory_id, entry.submap_index};
        poses[key] = planar_pose(entry.pose.position, entry.pose.orientation);
        submaps[key] = frozen(entry, 0);
        visible.insert(key);
      }
      for (const auto & entry : poses) {grid_->set_pose(entry.first, entry.second);}
      grid_->set_visible(visible);
      submaps_ = std::move(submaps);
      submap_stamp_ = msg->header.stamp;
      has_submap_stamp_ = true;
      dirty_ = true;
    } catch (const std::exception & error) {warning(error.what());}
  }
  rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submaps_sub_;
#endif
  std::unique_ptr<Grid> grid_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr reference_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr semantic_pub_, combined_pub_;
  std::map<uint8_t, rclcpp::Publisher<OccupancyGrid>::SharedPtr> class_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<Trigger>::SharedPtr save_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr retry_timer_, publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
  Cloud::ConstSharedPtr pending_;
  OccupancyGrid::ConstSharedPtr laser_;
  Geometry geometry_;
  Steady::time_point received_;
  int64_t last_stamp_ = -1;
  std::string cloud_topic_, reference_topic_, map_frame_, odom_frame_, pose_source_;
  std::filesystem::path save_directory_;
  std::string save_name_;
  double tf_wait_, submap_max_age_;
  uint64_t max_points_;
  unsigned median_kernel_;
  int trajectory_id_, free_max_, occupied_max_;
  bool dirty_ = false, has_submap_stamp_ = false;
  builtin_interfaces::msg::Time submap_stamp_;
  std::map<Submap, bool> submaps_;
};

rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<MapNode>(options);
}
}  // namespace limo_map
