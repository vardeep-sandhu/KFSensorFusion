#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "kalman_filter.cpp"

namespace py = pybind11;
constexpr auto byref = py::return_value_policy::reference_internal;

PYBIND11_MODULE(kalmanfilter, kf) {
    kf.doc() = "This is a python wrapper for Kalman Filter class in cpp";

    py::class_<KalmanFilter>(kf, "KalmanFilter")
    .def(py::init<>())  
    .def("setMetrices", &KalmanFilter::setMetrices)
    .def("predict", &KalmanFilter::predict)
    .def("update", &KalmanFilter::update)
    .def("getX", &KalmanFilter::getX)
    ;
}