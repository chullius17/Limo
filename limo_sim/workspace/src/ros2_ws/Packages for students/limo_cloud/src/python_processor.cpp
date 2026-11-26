#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "limo_cloud/processor.hpp"

#include <cstring>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace limo_cloud
{
namespace
{
class Gil
{
public:
  Gil()
  {
    static std::once_flag once;
    std::call_once(once, []() {
      if (!Py_IsInitialized()) {
        Py_Initialize();
        PyEval_SaveThread();
      }
      // CPython/NumPy belong to the process. Do not finalize/reinitialize them
      // on node destruction, especially when several nodes use Python.
    });
    state_ = PyGILState_Ensure();
  }
  ~Gil() {PyGILState_Release(state_);}
private:
  PyGILState_STATE state_;
};

std::runtime_error python_error()
{
  PyObject * type = nullptr, * value = nullptr, * traceback = nullptr;
  PyErr_Fetch(&type, &value, &traceback);
  PyErr_NormalizeException(&type, &value, &traceback);
  std::string message = "Python plugin error";
  PyObject * formatted = value ? PyObject_Str(value) : nullptr;
  if (formatted) {
    const char * text = PyUnicode_AsUTF8(formatted);
    if (text) {message += std::string(": ") + text;}
  }
  Py_XDECREF(formatted);
  // Preserve the traceback in the node's stderr for algorithm debugging.
  // PyErr_Print treats SystemExit as a request to terminate the host process.
  // Display it as a plugin failure instead, then let the worker drop this frame.
  if (type) {PyErr_Display(type, value, traceback);}
  Py_XDECREF(type);
  Py_XDECREF(value);
  Py_XDECREF(traceback);
  PyErr_Clear();
  return std::runtime_error(message);
}

struct Object
{
  explicit Object(PyObject * p) : value(p) {if (!p) {throw python_error();}}
  ~Object() {Py_DECREF(value);}
  Object(const Object &) = delete;
  Object & operator=(const Object &) = delete;
  PyObject * value;
};

struct OwnedBuffer
{
  PyObject_HEAD
  std::shared_ptr<const void> * owner;
  void * data;
  Py_ssize_t size;
  int readonly;
};

int get_buffer(PyObject * object, Py_buffer * view, int flags)
{
  auto * buffer = reinterpret_cast<OwnedBuffer *>(object);
  return PyBuffer_FillInfo(view, object, buffer->data, buffer->size, buffer->readonly, flags);
}

void delete_buffer(PyObject * object)
{
  delete reinterpret_cast<OwnedBuffer *>(object)->owner;
  Py_TYPE(object)->tp_free(object);
}

PyObject * buffer_view(
  std::shared_ptr<const void> owner, const void * data, size_t size, bool readonly)
{
  static PyBufferProcs protocol = {get_buffer, nullptr};
  static PyTypeObject type = {PyVarObject_HEAD_INIT(nullptr, 0)};
  if (!type.tp_name) {
    type.tp_name = "limo_cloud.OwnedBuffer";
    type.tp_basicsize = sizeof(OwnedBuffer);
    type.tp_flags = Py_TPFLAGS_DEFAULT;
    type.tp_dealloc = delete_buffer;
    type.tp_as_buffer = &protocol;
    if (PyType_Ready(&type) < 0) {throw python_error();}
  }
  auto * buffer = reinterpret_cast<OwnedBuffer *>(type.tp_alloc(&type, 0));
  if (!buffer) {throw python_error();}
  buffer->owner = new std::shared_ptr<const void>(std::move(owner));
  buffer->data = const_cast<void *>(data);
  buffer->size = static_cast<Py_ssize_t>(size);
  buffer->readonly = readonly;
  return reinterpret_cast<PyObject *>(buffer);
}

void dict_set(PyObject * dict, const char * key, PyObject * value)
{
  Object owned(value);
  if (PyDict_SetItemString(dict, key, owned.value) < 0) {throw python_error();}
}

PyObject * parameter_value(const rclcpp::Parameter & parameter)
{
  using rclcpp::ParameterType;
  switch (parameter.get_type()) {
    case ParameterType::PARAMETER_BOOL: return PyBool_FromLong(parameter.as_bool());
    case ParameterType::PARAMETER_INTEGER: return PyLong_FromLongLong(parameter.as_int());
    case ParameterType::PARAMETER_DOUBLE: return PyFloat_FromDouble(parameter.as_double());
    case ParameterType::PARAMETER_STRING: return PyUnicode_FromString(parameter.as_string().c_str());
    default: break;
  }
  Object result(PyList_New(0));
  auto append = [&result](PyObject * value) {
      Object item(value);
      if (PyList_Append(result.value, item.value) < 0) {throw python_error();}
    };
  switch (parameter.get_type()) {
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      for (auto v : parameter.as_integer_array()) {append(PyLong_FromLongLong(v));} break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      for (auto v : parameter.as_double_array()) {append(PyFloat_FromDouble(v));} break;
    case ParameterType::PARAMETER_STRING_ARRAY:
      for (const auto & v : parameter.as_string_array()) {append(PyUnicode_FromString(v.c_str()));} break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      for (auto v : parameter.as_bool_array()) {append(PyBool_FromLong(v));} break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      for (auto v : parameter.as_byte_array()) {append(PyLong_FromLong(v));} break;
    default: throw std::invalid_argument("Unsupported Python plugin parameter type");
  }
  Py_INCREF(result.value);
  return result.value;
}

class PythonProcessor : public Processor
{
public:
  explicit PythonProcessor(std::string specification) : specification_(std::move(specification)) {}
  ~PythonProcessor() override
  {
    if (plugin_) {
      Gil guard;
      if (PyObject_HasAttrString(plugin_, "close")) {
        PyObject * closed = PyObject_CallMethod(plugin_, "close", nullptr);
        if (!closed) {PyErr_WriteUnraisable(plugin_);} else {Py_DECREF(closed);}
      }
      Py_DECREF(plugin_);
      Py_DECREF(runtime_);
    }
  }

  void configure(rclcpp::Node & node, const std::string & name) override
  {
    namespace_ = node.get_namespace();
    Gil guard;
    Object module(PyImport_ImportModule("limo_cloud._runtime"));
    Object parameters(PyDict_New());
    const auto entries = node.list_parameters({name}, 0);
    const auto prefix = name + ".";
    for (const auto & key : entries.names) {
      if (key.compare(0, prefix.size(), prefix) == 0 && key != prefix + "type") {
        dict_set(parameters.value, key.substr(prefix.size()).c_str(),
          parameter_value(node.get_parameter(key)));
      }
    }
    Object plugin(PyObject_CallMethod(
      module.value, "load", "sO", specification_.c_str(), parameters.value));
    Object pre(PyObject_CallMethod(module.value, "has_hook", "Os", plugin.value, "process_images"));
    Object post(PyObject_CallMethod(module.value, "has_hook", "Os", plugin.value, "process_cloud"));
    pre_ = PyObject_IsTrue(pre.value);
    post_ = PyObject_IsTrue(post.value);
    plugin_ = plugin.value;
    runtime_ = module.value;
    Py_INCREF(plugin_);
    Py_INCREF(runtime_);
  }

  void process_images(Frame & frame) override {if (pre_) {invoke(frame, "process_images");}}
  void process_cloud(Frame & frame) override {if (post_) {invoke(frame, "process_cloud");}}

private:
  PyObject * image_view(sensor_msgs::msg::Image::ConstSharedPtr image, bool readonly)
  {
    Object buffer(buffer_view(image, image->data.data(), image->data.size(), readonly));
    return PyObject_CallMethod(
      runtime_, "image_array", "OkkksO", buffer.value,
      static_cast<unsigned long>(image->height), static_cast<unsigned long>(image->width),
      static_cast<unsigned long>(image->step), image->encoding.c_str(),
      image->is_bigendian ? Py_True : Py_False);
  }

  PyObject * cloud_view(sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    validate_cloud(*cloud);
    Object buffer(buffer_view(cloud, cloud->data.data(), cloud->data.size(), false));
    return PyObject_CallMethod(runtime_, "cloud_array", "Ok", buffer.value,
      static_cast<unsigned long>(cloud->width));
  }

  PyObject * samples_view(const std::shared_ptr<std::vector<PixelSample>> & samples)
  {
    if (!samples) {Py_RETURN_NONE;}
    Object buffer(buffer_view(samples, samples->data(), samples->size() * sizeof(PixelSample), true));
    return PyObject_CallMethod(runtime_, "sample_array", "Ok", buffer.value,
      static_cast<unsigned long>(samples->size()));
  }

  void read_samples(PyObject * value, Frame & frame)
  {
    if (value == Py_None) {frame.samples.reset(); return;}
    Object normal(PyObject_CallMethod(runtime_, "normalise_samples", "O", value));
    Py_buffer view;
    if (PyObject_GetBuffer(normal.value, &view, PyBUF_SIMPLE) < 0) {throw python_error();}
    const auto count = static_cast<size_t>(view.len) / sizeof(PixelSample);
    auto samples = std::make_shared<std::vector<PixelSample>>(count);
    if (view.len) {std::memcpy(samples->data(), view.buf, view.len);}
    PyBuffer_Release(&view);
    frame.samples = std::move(samples);
  }

  sensor_msgs::msg::Image::ConstSharedPtr read_image(
    PyObject * array, sensor_msgs::msg::Image::ConstSharedPtr original,
    const std_msgs::msg::Header & header)
  {
    Object normal(PyObject_CallMethod(runtime_, "normalise_image", "O", array));
    PyObject * contiguous = PyTuple_GetItem(normal.value, 0);
    Py_buffer view;
    if (PyObject_GetBuffer(contiguous, &view, PyBUF_ND) < 0) {throw python_error();}
    const uint32_t height = static_cast<uint32_t>(view.shape[0]);
    const uint32_t width = static_cast<uint32_t>(view.shape[1]);
    if (original && view.buf == original->data.data() &&
      width == original->width && height == original->height)
    {
      PyBuffer_Release(&view);
      return original;
    }
    auto result = std::make_shared<sensor_msgs::msg::Image>();
    result->header = header;
    result->width = width;
    result->height = height;
    result->encoding = PyUnicode_AsUTF8(PyTuple_GetItem(normal.value, 1));
    result->is_bigendian = PyObject_IsTrue(PyTuple_GetItem(normal.value, 2));
    result->step = width * view.itemsize;
    result->data.resize(view.len);
    if (view.len) {std::memcpy(result->data.data(), view.buf, view.len);}
    PyBuffer_Release(&view);
    return result;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr read_cloud(
    PyObject * array, sensor_msgs::msg::PointCloud2::SharedPtr original,
    sensor_msgs::msg::PointCloud2::SharedPtr prototype)
  {
    Object normal(PyObject_CallMethod(runtime_, "normalise_cloud", "O", array));
    Py_buffer view;
    if (PyObject_GetBuffer(normal.value, &view, PyBUF_SIMPLE) < 0) {throw python_error();}
    if (original && view.buf == original->data.data() &&
      static_cast<size_t>(view.len) == original->data.size())
    {
      PyBuffer_Release(&view);
      return original;
    }
    if (static_cast<uint64_t>(view.len) > UINT32_MAX) {
      PyBuffer_Release(&view);
      throw std::invalid_argument("Plugin cloud exceeds PointCloud2 row size limit");
    }
    auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
    result->header = prototype->header;
    result->fields = prototype->fields;
    result->is_bigendian = prototype->is_bigendian;
    result->is_dense = false;
    result->height = 1;
    result->point_step = sizeof(Point);
    result->row_step = static_cast<uint32_t>(view.len);
    result->width = result->row_step / sizeof(Point);
    result->data.resize(view.len);
    if (view.len) {std::memcpy(result->data.data(), view.buf, view.len);}
    PyBuffer_Release(&view);
    return result;
  }

  void invoke(Frame & frame, const char * hook)
  {
    Gil guard;
    auto & scratch = frame.state["limo_cloud/python"];
    if (!scratch) {
      scratch = std::shared_ptr<void>(PyDict_New(), [](void * object) {
          Gil gil;
          Py_DECREF(static_cast<PyObject *>(object));
        });
      if (!scratch) {throw python_error();}
    }
    Object images(PyDict_New()), clouds(PyDict_New()), metadata(PyDict_New());
    for (const auto & entry : frame.images) {
      dict_set(images.value, entry.first.c_str(), image_view(entry.second, false));
    }
    for (const auto & entry : frame.clouds) {
      dict_set(clouds.value, entry.first.c_str(), cloud_view(entry.second));
    }
    dict_set(metadata.value, "frame_id", PyUnicode_FromString(frame.labels->header.frame_id.c_str()));
    dict_set(metadata.value, "target_frame", PyUnicode_FromString(frame.transform.header.frame_id.c_str()));
    dict_set(metadata.value, "node_namespace", PyUnicode_FromString(namespace_.c_str()));
    dict_set(metadata.value, "stamp_ns", PyLong_FromLongLong(
      static_cast<int64_t>(frame.labels->header.stamp.sec) * 1000000000LL +
      frame.labels->header.stamp.nanosec));
    dict_set(metadata.value, "input_crop_y_min", PyFloat_FromDouble(frame.options.input_crop_y_min));
    dict_set(metadata.value, "roi_y_min", PyFloat_FromDouble(frame.options.roi_y_min));
    dict_set(metadata.value, "roi_y_max", PyFloat_FromDouble(frame.options.roi_y_max));
    dict_set(metadata.value, "min_depth", PyFloat_FromDouble(frame.options.min_depth));
    dict_set(metadata.value, "max_depth", PyFloat_FromDouble(frame.options.max_depth));
    dict_set(metadata.value, "flatten_to_bev", PyBool_FromLong(frame.options.flatten_to_bev));
    dict_set(metadata.value, "camera_width", PyLong_FromLong(frame.camera_info->width));
    dict_set(metadata.value, "camera_height", PyLong_FromLong(frame.camera_info->height));
    Object k(PyList_New(9));
    for (size_t i = 0; i < 9; ++i) {PyList_SET_ITEM(k.value, i, PyFloat_FromDouble(frame.camera_info->k[i]));}
    if (PyDict_SetItemString(metadata.value, "k", k.value) < 0) {throw python_error();}
    const auto & t = frame.transform.transform;
    dict_set(metadata.value, "translation", Py_BuildValue("(ddd)", t.translation.x, t.translation.y, t.translation.z));
    dict_set(metadata.value, "rotation_xyzw", Py_BuildValue("(dddd)", t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
    Object depth(image_view(frame.depth, true)), labels(image_view(frame.labels, true));
    Object cloud(frame.cloud ? cloud_view(frame.cloud) : (Py_INCREF(Py_None), Py_None));
    Object samples(samples_view(frame.samples));
    Object context(PyObject_CallMethod(
      runtime_, "Frame", "OOOOOOOO", depth.value, labels.value, cloud.value,
      images.value, clouds.value, metadata.value, static_cast<PyObject *>(scratch.get()), samples.value));
    Object result(PyObject_CallMethod(plugin_, hook, "O", context.value));
    if (frame.cloud) {frame.cloud->is_dense = false;}
    Object new_depth(PyObject_GetAttrString(context.value, "depth"));
    Object new_labels(PyObject_GetAttrString(context.value, "labels"));
    // Unchanged, possibly strided borrowed views require no copy.
    if (new_depth.value != depth.value) {
      frame.depth = read_image(new_depth.value, frame.depth, frame.depth->header);
    }
    if (new_labels.value != labels.value) {
      frame.labels = read_image(new_labels.value, frame.labels, frame.labels->header);
    }
    Object new_samples(PyObject_GetAttrString(context.value, "samples"));
    if (new_samples.value != samples.value) {read_samples(new_samples.value, frame);}
    Object new_cloud(PyObject_GetAttrString(context.value, "cloud"));
    if (frame.cloud && new_cloud.value == Py_None) {
      throw std::invalid_argument("Plugin removed the output cloud");
    }
    if (new_cloud.value != cloud.value && new_cloud.value != Py_None) {
      if (!frame.cloud) {throw std::invalid_argument("Cloud replacement requires the cloud stage");}
      frame.cloud = read_cloud(new_cloud.value, frame.cloud, frame.cloud);
    }
    if (frame.cloud) {
      Object dense(PyObject_GetAttrString(context.value, "cloud_is_dense"));
      const int is_dense = PyObject_IsTrue(dense.value);
      if (is_dense < 0) {throw python_error();}
      frame.cloud->is_dense = is_dense;
    }
    Object new_images(PyObject_GetAttrString(context.value, "images"));
    Object new_clouds(PyObject_GetAttrString(context.value, "clouds"));
    if (!PyDict_Check(new_images.value) || !PyDict_Check(new_clouds.value)) {
      throw std::invalid_argument("frame.images and frame.clouds must remain dictionaries");
    }
    PyObject * key, * value;
    Py_ssize_t position = 0;
    decltype(frame.images) updated_images;
    while (PyDict_Next(new_images.value, &position, &key, &value)) {
      const char * name = PyUnicode_AsUTF8(key);
      if (!name) {throw python_error();}
      auto previous = frame.images.find(name);
      auto image = read_image(value,
        previous == frame.images.end() ? nullptr : previous->second, frame.labels->header);
      updated_images[name] = std::const_pointer_cast<sensor_msgs::msg::Image>(image);
    }
    frame.images = std::move(updated_images);
    position = 0;
    decltype(frame.clouds) updated_clouds;
    while (PyDict_Next(new_clouds.value, &position, &key, &value)) {
      const char * name = PyUnicode_AsUTF8(key);
      if (!name) {throw python_error();}
      if (!frame.cloud) {throw std::invalid_argument("Intermediate clouds require the cloud stage");}
      auto previous = frame.clouds.find(name);
      updated_clouds[name] = read_cloud(value,
        previous == frame.clouds.end() ? nullptr : previous->second, frame.cloud);
    }
    frame.clouds = std::move(updated_clouds);
  }

  std::string specification_, namespace_;
  PyObject * runtime_{nullptr}, * plugin_{nullptr};
  bool pre_{false}, post_{false};
};
}  // namespace

std::shared_ptr<Processor> make_python_processor(const std::string & specification)
{
  return std::make_shared<PythonProcessor>(specification);
}

}  // namespace limo_cloud
