#include "limo_cloud/processor.hpp"

#include <cstring>

#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

namespace
{
limo_cloud::Frame frame()
{
  limo_cloud::Frame f;
  auto depth = std::make_shared<sensor_msgs::msg::Image>();
  auto labels = std::make_shared<sensor_msgs::msg::Image>();
  depth->width = labels->width = 4;
  depth->height = labels->height = 2;
  depth->step = 20;
  labels->step = 6;
  depth->encoding = "32FC1";
  labels->encoding = "mono8";
  depth->data.resize(40);
  labels->data = {1, 2, 3, 255, 0, 0, 3, 2, 1, 255, 0, 0};
  for (size_t y = 0; y < 2; ++y) {
    for (size_t x = 0; x < 4; ++x) {
      float d = 1;
      std::memcpy(depth->data.data() + y * 20 + x * 4, &d, 4);
    }
  }
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->width = 4;
  info->height = 2;
  info->k = {2, 0, 1.5, 0, 2, 0.5, 0, 0, 1};
  f.depth = depth;
  f.labels = labels;
  f.camera_info = info;
  f.transform.transform.rotation.w = 1;
  f.options.input_crop_y_min = 0;
  return f;
}

void project(limo_cloud::Frame & f)
{
  limo_cloud::Projector projector;
  f.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  projector.project(*f.depth, *f.labels, *f.camera_info, f.transform.transform, f.options,
    *f.cloud, f.samples.get());
}

class Processors : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

TEST_F(Processors, InputCopyOnWritePreservesOriginal)
{
  auto f = frame();
  auto original = f.labels;
  f.edit_labels().data[0] = 9;
  EXPECT_EQ(original->data[0], 1);
  EXPECT_EQ(f.labels->data[0], 9);
  EXPECT_EQ(&f.edit_labels(), f.labels.get());
}

TEST_F(Processors, PythonImageIntermediateAndCppSelectionCompose)
{
  rclcpp::Node node("processor_test", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .parameter_overrides({rclcpp::Parameter("selection.keep_classes", std::vector<int64_t>{3, 255})}));
  auto python = limo_cloud::make_python_processor("limo_cloud.examples:BackgroundMask");
  python->configure(node, "mask");
  auto f = frame();
  python->process_images(f);
  ASSERT_EQ(f.images.count("background/mask"), 1U);
  EXPECT_EQ(f.images.at("background/mask")->data, (std::vector<uint8_t>{0, 0, 1, 0, 1, 0, 0, 0}));
  project(f);
  pluginlib::ClassLoader<limo_cloud::Processor> loader("limo_cloud", "limo_cloud::Processor");
  auto cpp = loader.createSharedInstance("limo_cloud/SelectClasses");
  cpp->configure(node, "selection");
  cpp->process_cloud(f);
  EXPECT_EQ(f.cloud->width, 4U);
  EXPECT_NO_THROW(limo_cloud::validate_cloud(*f.cloud));
  for (size_t i = 0; i < f.cloud->width; ++i) {
    const auto label = f.cloud->data[i * sizeof(limo_cloud::Point) + 12];
    EXPECT_TRUE(label == 3 || label == 255);
  }
}

TEST_F(Processors, PythonEditsSelectionScratchAndRetainedViews)
{
  rclcpp::Node node("python_ownership_test");
  auto plugin = limo_cloud::make_python_processor("test_python_plugin:OwnershipProbe");
  plugin->configure(node, "probe");
  for (int i = 0; i < 3; ++i) {
    auto f = frame();
    const auto original = f.labels;
    plugin->process_images(f);
    EXPECT_EQ(original->data[0], 1);
    EXPECT_EQ(f.labels->data[0], 7);
    project(f);
    plugin->process_cloud(f);
    EXPECT_EQ(f.cloud->width, 2U);
    EXPECT_EQ(f.cloud->data[12], 7);
    EXPECT_EQ(f.cloud->data[28], 1);
  }
}

TEST_F(Processors, PythonExceptionPropagatesForWholeFrameDrop)
{
  rclcpp::Node node("python_failure_test");
  auto plugin = limo_cloud::make_python_processor("test_python_plugin:FailureProbe");
  plugin->configure(node, "probe");
  auto f = frame();
  project(f);
  EXPECT_THROW(plugin->process_cloud(f), std::runtime_error);
  auto next = frame();
  project(next);
  EXPECT_NO_THROW(plugin->process_cloud(next));
}

TEST_F(Processors, PythonViewsSurviveCppCloudReplacementWithinFrame)
{
  rclcpp::Node node("mixed_ownership_test", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .parameter_overrides({rclcpp::Parameter("selection.keep_classes", std::vector<int64_t>{3, 255})}));
  auto before = limo_cloud::make_python_processor("test_python_plugin:RetainBeforeCpp");
  auto after = limo_cloud::make_python_processor("test_python_plugin:ValidateAfterCpp");
  before->configure(node, "before");
  after->configure(node, "after");
  pluginlib::ClassLoader<limo_cloud::Processor> loader("limo_cloud", "limo_cloud::Processor");
  auto cpp = loader.createSharedInstance("limo_cloud/SelectClasses");
  cpp->configure(node, "selection");
  auto f = frame();
  project(f);
  before->process_cloud(f);
  cpp->process_cloud(f);
  EXPECT_NO_THROW(after->process_cloud(f));
}

TEST_F(Processors, PythonSystemExitIsReportedAsAFrameFailure)
{
  rclcpp::Node node("python_exit_test");
  auto plugin = limo_cloud::make_python_processor("test_python_plugin:ExitProbe");
  plugin->configure(node, "probe");
  auto f = frame();
  project(f);
  EXPECT_THROW(plugin->process_cloud(f), std::runtime_error);
}

TEST_F(Processors, PythonOrderedSamplesReachCppProjection)
{
  rclcpp::Node node("python_sample_test");
  auto plugin = limo_cloud::make_python_processor("test_python_plugin:SparseProbe");
  plugin->configure(node, "probe");
  auto f = frame();
  plugin->process_images(f);
  ASSERT_TRUE(f.samples);
  ASSERT_EQ(f.samples->size(), 3U);
  project(f);
  EXPECT_EQ(f.cloud->width, 3U);
  EXPECT_EQ(f.cloud->data[12], 8);
  EXPECT_EQ(f.cloud->data[28], 3);
  EXPECT_EQ(f.cloud->data[44], 1);
}
}  // namespace
