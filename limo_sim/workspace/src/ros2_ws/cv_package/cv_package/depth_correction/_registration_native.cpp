// Fused depth-to-RGB projection and nearest-Z buffer. Python buffer protocol
// keeps this independent of the NumPy C ABI and accepts padded/strided depth.
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
struct Buffer {
  Py_buffer view{};
  ~Buffer() { if (view.obj) PyBuffer_Release(&view); }
  bool get(PyObject * object, int flags) {
    return PyObject_GetBuffer(object, &view, flags) == 0;
  }
  bool type(const char * format, Py_ssize_t size) const {
    return view.itemsize == size && view.format && std::strcmp(view.format, format) == 0;
  }
};

PyObject * project_depth(PyObject *, PyObject * args) {
  PyObject * depth_object, * rays_object, * output_object, * parameters_object;
  if (!PyArg_ParseTuple(args, "OOOO", &depth_object, &rays_object,
                       &output_object, &parameters_object)) return nullptr;
  Buffer depth, rays, output, parameters;
  if (!depth.get(depth_object, PyBUF_STRIDES | PyBUF_FORMAT) ||
      !rays.get(rays_object, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) ||
      !output.get(output_object, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | PyBUF_WRITABLE) ||
      !parameters.get(parameters_object, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT)) return nullptr;
  const bool single = depth.type("f", sizeof(float));
  if ((!single && !depth.type("d", sizeof(double))) || depth.view.ndim != 2 ||
      !rays.type("d", sizeof(double)) || rays.view.ndim != 3 ||
      rays.view.shape[0] != depth.view.shape[0] ||
      rays.view.shape[1] != depth.view.shape[1] || rays.view.shape[2] != 3 ||
      !output.type("f", sizeof(float)) || output.view.ndim != 2 ||
      !parameters.type("d", sizeof(double)) || parameters.view.ndim != 1 ||
      parameters.view.shape[0] != 12 || output.view.shape[0] <= 0 || output.view.shape[1] <= 0) {
    PyErr_SetString(PyExc_ValueError, "Invalid depth registration buffer shape/dtype");
    return nullptr;
  }
  const auto * p = static_cast<const double *>(parameters.view.buf);
  const auto * direction = static_cast<const double *>(rays.view.buf);
  auto * target = static_cast<float *>(output.view.buf);
  const Py_ssize_t height = output.view.shape[0], width = output.view.shape[1];
  if (!(p[7] > 0 && p[8] > p[9] && p[10] > 0 && p[11] > p[10])) {
    PyErr_SetString(PyExc_ValueError, "Invalid projection dimensions/depth range");
    return nullptr;
  }
  // Buffers remain exported for the entire loop, including while releasing GIL.
  Py_BEGIN_ALLOW_THREADS
  std::fill(target, target + height * width, std::numeric_limits<float>::infinity());
  for (Py_ssize_t row = 0; row < depth.view.shape[0]; ++row) {
    const auto * source_row = static_cast<const char *>(depth.view.buf) + row * depth.view.strides[0];
    for (Py_ssize_t col = 0; col < depth.view.shape[1]; ++col, direction += 3) {
      const auto * source = source_row + col * depth.view.strides[1];
      double value;
      if (single) {
        float sample;
        std::memcpy(&sample, source, sizeof(sample));
        value = sample;
      } else {
        std::memcpy(&value, source, sizeof(value));
      }
      if (!std::isfinite(value) || value < p[10] || value > p[11]) continue;
      const double x = direction[0] * value + p[0];
      const double y = direction[1] * value + p[1];
      const double z = direction[2] * value + p[2];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          z < p[10] || z > p[11]) continue;
      double u = x / z * p[3] + p[5];
      double v = y / z * p[4] + p[6];
      u = (u + .5) * width / p[7] - .5;
      v = (v - p[9] + .5) * height / (p[8] - p[9]) - .5;
      if (!(u >= -.5 && u < width - .5 && v >= -.5 && v < height - .5)) continue;
      const auto index = static_cast<Py_ssize_t>(std::floor(v + .5)) * width +
                         static_cast<Py_ssize_t>(std::floor(u + .5));
      target[index] = std::min(target[index], static_cast<float>(z));
    }
  }
  for (Py_ssize_t i = 0; i < height * width; ++i) {
    if (!std::isfinite(target[i])) target[i] = std::numeric_limits<float>::quiet_NaN();
  }
  Py_END_ALLOW_THREADS
  Py_RETURN_NONE;
}

PyMethodDef methods[] = {
  {"project_depth", project_depth, METH_VARARGS, "Project into a reusable float32 nearest-Z buffer."},
  {nullptr, nullptr, 0, nullptr}
};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "_registration_native", nullptr, -1, methods};
}  // namespace

PyMODINIT_FUNC PyInit__registration_native() { return PyModule_Create(&module); }
