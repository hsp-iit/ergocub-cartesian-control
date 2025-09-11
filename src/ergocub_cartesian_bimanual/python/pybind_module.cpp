#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BimanualIK.h>

namespace py = pybind11;

PYBIND11_MODULE(pysquale, m) {
  py::class_<PoseInput>(m, "PoseInput")
    .def(py::init<>())
    // Convenience ctor from numpy arrays: pos[3], quat_xyzw[4]
    .def(py::init([](const Eigen::Vector3d& pos, const Eigen::Vector4d& quat_xyzw){
        PoseInput p;
        p.pos = pos;
        p.quat = Eigen::Quaterniond(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
        return p;
      }), py::arg("pos"), py::arg("quat_xyzw"))
    .def_readwrite("pos", &PoseInput::pos)
    .def_property(
      "quat",
      [](const PoseInput& p){
        // Return [x, y, z, w]
        py::array_t<double> a(4);
        auto r = a.mutable_unchecked<1>();
        r(0) = p.quat.x(); r(1) = p.quat.y(); r(2) = p.quat.z(); r(3) = p.quat.w();
        return a;
      },
      [](PoseInput& p, py::object obj){
        py::sequence s = py::cast<py::sequence>(obj);
        if(py::len(s) != 4) throw std::runtime_error("quat expects a sequence of 4 numbers [x,y,z,w]");
        double x = py::cast<double>(s[0]);
        double y = py::cast<double>(s[1]);
        double z = py::cast<double>(s[2]);
        double w = py::cast<double>(s[3]);
        p.quat = Eigen::Quaterniond(w, x, y, z); // Eigen stores as (w,x,y,z)
      }
    )
    // Getter/setter using XYZW numpy arrays
    .def("get_quat_xyzw", [](const PoseInput& self){
        Eigen::Vector4d v; v << self.quat.x(), self.quat.y(), self.quat.z(), self.quat.w(); return v;
      })
    .def("set_quat_xyzw", [](PoseInput& self, const Eigen::Vector4d& quat_xyzw){
        self.quat = Eigen::Quaterniond(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
      }, py::arg("quat_xyzw"));

  py::class_<BimanualIK>(m, "BimanualIK")
    .def(py::init<const std::string&,
                  const std::vector<std::string>&,
                  const std::vector<std::string>&,
                  const std::vector<std::string>&,
                  const std::string&, const std::string&,
                  const std::string&, const std::string&,
                  double,
                  const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
                  const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&,
                  const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&,
                  double,
                  const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::Vector3d&>(),
         py::arg("urdf"),
         py::arg("right_joints"),
         py::arg("left_joints"),
         py::arg("torso_joints"),
         py::arg("right_root_frame"),
         py::arg("right_ee_frame"),
         py::arg("left_root_frame"),
         py::arg("left_ee_frame"),
         py::arg("sampling_time"),
         py::arg("joint_acc_weight"),
         py::arg("joint_pos_weights"),
         py::arg("joint_pos_kp"),
         py::arg("joint_pos_kd"),
         py::arg("cart_pos_weight"),
         py::arg("cart_pos_kp"),
         py::arg("cart_pos_kd"),
         py::arg("cart_ori_weight"),
         py::arg("cart_ori_kp"),
         py::arg("cart_ori_kd"),
         py::arg("improve_manip_weight"),
         py::arg("q_home"),
         py::arg("q_lower"),
         py::arg("q_upper"),
         py::arg("limit_gains_rlT"))
    .def("reset", &BimanualIK::reset, py::arg("q0"), py::arg("dq0") = Eigen::VectorXd())
  .def("solve_ik", [](BimanualIK& self,
               py::object right_pose, py::object left_pose,
               const Eigen::Vector3d& right_lin_vel, const Eigen::Vector3d& right_ang_vel,
               const Eigen::Vector3d& left_lin_vel,  const Eigen::Vector3d& left_ang_vel,
               const Eigen::Vector3d& right_lin_acc, const Eigen::Vector3d& right_ang_acc,
               const Eigen::Vector3d& left_lin_acc,  const Eigen::Vector3d& left_ang_acc){
          std::optional<PoseInput> ri, li;
      if(!right_pose.is_none()) ri = right_pose.cast<PoseInput>();
      if(!left_pose.is_none())  li = left_pose.cast<PoseInput>();
      auto q = self.solve_ik(ri, li, right_lin_vel, right_ang_vel, left_lin_vel, left_ang_vel, right_lin_acc, right_ang_acc, left_lin_acc, left_ang_acc);
          return q;
  }, py::arg("right_pose") = py::none(), py::arg("left_pose") = py::none(),
     py::arg("right_lin_vel") = Eigen::Vector3d::Zero(), py::arg("right_ang_vel") = Eigen::Vector3d::Zero(),
     py::arg("left_lin_vel")  = Eigen::Vector3d::Zero(), py::arg("left_ang_vel")  = Eigen::Vector3d::Zero(),
     py::arg("right_lin_acc") = Eigen::Vector3d::Zero(), py::arg("right_ang_acc") = Eigen::Vector3d::Zero(),
     py::arg("left_lin_acc")  = Eigen::Vector3d::Zero(), py::arg("left_ang_acc")  = Eigen::Vector3d::Zero());
}
