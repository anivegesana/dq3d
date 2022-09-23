#include <vector>
#include <sstream>
#define EIGEN_QUATERNION_PLUGIN <quaternion_plugin.h>
#include <Eigen/Dense>
#define private public
#include "DualQuaternion.h"
#undef private
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

namespace py = pybind11;
using namespace Eigen;
typedef double Float;

struct _scipy_Rotation {
    py::handle val; // py::object
};

namespace pybind11 { namespace detail {
    template <> struct type_caster<_scipy_Rotation> {
    public:
        PYBIND11_TYPE_CASTER(_scipy_Rotation, const_name("scipy.spatial.transform.Rotation"));

        bool load(handle src, bool) {
            /* Extract PyObject from handle */
            PyObject *source = src.ptr();
            /*handle type = src.attr("__class__");
            std::string prefix = "scipy."
            if (type.attr("__name__").cast<std::string>() != "Rotation" && type.attr("__module__").cast<std::string>().substr(0, prefix.size()) == prefix) {
            	throw py::type_error("Excepted a scipy.spatial.transform.Rotation");
            }*/
            /* Now try to convert into a C++ int */
            //if (strcmp(source->ob_type->tp_name, "Rotation") != 0)
                // TODO: assert type
            value.val = source;
            Py_DECREF(source);
            /* Ensure return code was OK (to avoid out-of-range errors etc) */
            return !PyErr_Occurred();
        }

        static handle cast(_scipy_Rotation src, return_value_policy /* policy */, handle /* parent */) {
            return src.val;
        }
    };
}}

PYBIND11_MODULE(_eigen_dq, m) {
    py::class_<Quaternion<Float> >(m, "quat")
      //.def(py::init<>())
      .def(py::init<Float, Float, Float, Float>(), py::arg("w") = 0, py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
      .def(py::init<Matrix<Float, 4, 1>&>(), py::arg("data"))
      .def(py::init<Matrix<Float, 3, 3>&>(), py::arg("rot_matrix"))
      .def(py::init([](Float ang, Matrix<Float, 3, 1>& axis) {
          return std::unique_ptr<Quaternion<Float>>(new Quaternion<Float>(AngleAxis<Float>(ang, axis)));}),
          py::arg("ang"), py::arg("axis"))
      .def_property("data", &Quaternion<Float>::getData, &Quaternion<Float>::setData)
      .def_property("w", &Quaternion<Float>::getW, &Quaternion<Float>::setW)
      .def_property("x", &Quaternion<Float>::getX, &Quaternion<Float>::setX)
      .def_property("y", &Quaternion<Float>::getY, &Quaternion<Float>::setY)
      .def_property("z", &Quaternion<Float>::getZ, &Quaternion<Float>::setZ)
      .def_property("vec", &Quaternion<Float>::getVec, &Quaternion<Float>::setVec)
      .def_static("identity", &Quaternion<Float>::Identity)
      .def("set_identity", &Quaternion<Float>::setIdentity)
      .def("squared_norm", &Quaternion<Float>::squaredNorm)
      .def("norm", &Quaternion<Float>::norm)
      .def("normalize", &Quaternion<Float>::normalize)
      .def("normalized", &Quaternion<Float>::normalized)
      .def("angular_distance", &Quaternion<Float>::angularDistance<Quaternion<Float>>)
      .def("dot", &Quaternion<Float>::dot<Quaternion<Float>>)
      .def("slerp", &Quaternion<Float>::slerp<Quaternion<Float>>)
      .def("to_rotation_matrix", &Quaternion<Float>::toRotationMatrix)
      .def("conjugate", &Quaternion<Float>::conjugate)
      .def("inverse", &Quaternion<Float>::inverse)
      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def("__repr__", [](const Quaternion<Float>& inst) {
          std::stringstream ss;
          ss << "< " << inst.getData().transpose() << " >";
          return ss.str();
        });

    py::class_<DualQuaternion<Float> >(m, "dualquat")
      //.def(py::init<>())
      .def(py::init<Quaternion<Float>&, Quaternion<Float>&>(),
           py::arg("real") = Quaternion<Float>(1, 0, 0, 0), py::arg("dual") = Quaternion<Float>(0, 0, 0, 0))
      .def(py::init<Quaternion<Float>&, Matrix<Float, 3, 1>&>(),
           py::arg("real") = Quaternion<Float>(1, 0, 0, 0), py::arg("dual") = Matrix<Float, 3, 1>::Zero())
      /*.def(py::init([](_scipy_Rotation& rotation, Matrix<Float, 3, 1>& translation) {
             Matrix<Float, 4, 1> m = rotation.val.attr("as_quat")().cast<Matrix<Float, 4, 1> >();
             Quaternion<Float> q(m);
             return DualQuaternion<Float>(q, translation);
           }),py::arg("rotation"), py::arg("dual") = Matrix<Float, 3, 1>::Zero())
      /*.def(py::init([](Quaternion<Float>& q) {
           return std::unique_ptr<DualQuaternion<Float>>(new DualQuaternion<Float>(q, Matrix<Float, 3, 1>::Zero()));}))*/
      .def_readwrite("real", &DualQuaternion<Float>::m_real)
      .def_readwrite("dual", &DualQuaternion<Float>::m_dual)
      .def_static("zeros", &DualQuaternion<Float>::zeros)
      .def_static("identity", &DualQuaternion<Float>::identity)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self * Float())
      .def(Float() * py::self)
      .def(py::self * py::self)
      .def("from_screw", &DualQuaternion<Float>::fromScrew)
      .def("conjugate", &DualQuaternion<Float>::conjugate)
      .def("norm", [](DualQuaternion<Float>& inst) {
           Float x, y; inst.norm(x, y); return std::make_tuple(x, y);
        })
      .def("normalize", &DualQuaternion<Float>::normalize)
      .def("normalized", &DualQuaternion<Float>::normalized)
      .def("transform_point", &DualQuaternion<Float>::transformPoint)
      .def("transform_vector", &DualQuaternion<Float>::transformVector)
      .def("inverse", &DualQuaternion<Float>::inverse)
      .def("exp", &DualQuaternion<Float>::exp)
      .def("log", &DualQuaternion<Float>::log)
      .def("pow", [](DualQuaternion<Float>& inst, Float t) {return (inst.log() * t).exp();})
      .def("rotation", &DualQuaternion<Float>::rotation)
      .def("translation", &DualQuaternion<Float>::translation)
      .def("translation_quaternion", &DualQuaternion<Float>::translationQuaternion)
      .def("to_matrix", &DualQuaternion<Float>::toMatrix)
      .def("__repr__", [](const DualQuaternion<Float>& inst) {
          std::stringstream ss;
          ss << "<real: " << inst.real().getData().transpose()
             << ", dual: "<< inst.dual().getData().transpose() << " >";
          return ss.str();
        });

    m.def("expq", &expq<Float>);
    m.def("logq", &logq<Float>);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
