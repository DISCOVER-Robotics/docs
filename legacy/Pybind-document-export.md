---
sidebar: auto
---

## 生成docstrings

1.激活虚拟环境

2.在python环境中安装pybind11\_mkdoc库

3.指定需要生成文档的头文件和c++标准,运行pybind11\_mkdoc

```plain&#x20;text
 conda activate xxx
 python -m pip install git+git://github.com/pybind/pybind11_mkdoc.git@master
 python -m pybind11_mkdoc -o docstrings.h -std=c++17 header_file_1.h header_file_2.h
```



## 修改绑定代码

基于类和命名空间，在每个类和函数定义最后一个参数增加docstrings宏，例如：

```go
PYBIND11_MODULE(airbot, m) {
  m.doc() = "airbot";
  m.attr("__version__") = AIRBOT_VERSION;
  m.attr("AIRBOT_PLAY_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1.urdf";
  m.attr("AIRBOT_PLAY_WITH_GRIPPER_URDF") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  py::class_<Robot, std::unique_ptr<Robot>>(m, "Robot")
      .def("alter_logging", &Robot::alter_logging, DOC(arm, Robot, alter_logging))
      .def("set_target_pose",
           py::overload_cast<const std::vector<std::vector<double>> &, bool, double>(&Robot::set_target_pose),
           py::arg("target_pose"), py::arg("use_planning") = true, py::arg("time") = 1.,
           DOC(arm, Robot, set_target_pose))

  m.def("create_agent", &createAgent, py::arg("urdf_path") = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf",
        py::arg("direction") = "down", py::arg("can_interface") = "can0", py::arg("vel") = M_PI,
        py::arg("end_mode") = "newteacher", py::arg("constraint") = false, DOC(arm, Robot, Robot));
};

```

编译项目得到共享库文件airbot\*.so



## 基于模板修改docs配置

> 模板仓库地址
>
> https://github.com/readthedocs/tutorial-template.git

> 配置好的模板地址，直接clone到pybind项目的根目录
>
> git@github.com:RoboticsChen/docs.git



在docs目录运行

```go
make html
```

