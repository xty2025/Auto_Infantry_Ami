# AutoAim
version 0.0.2(unfinished)

**可使用WSL2运行+调试功能**

**AI GENERATED FILES**

**To see human-written files, turn to file TODO.md**.

## 系统架构
系统由以下核心模块组成：

- **Driver**: 硬件驱动模块，负责相机图像采集和串口通信
- **Detector**: 目标检测模块，基于深度学习识别装甲板 + 车体
- **Tracker**: 目标跟踪模块，确保目标持续跟踪
- **Predictor**: 运动预测模块，Kalman滤波器预测目标运动轨迹
- **Solver**: 姿态解算模块，解算相对位置关系
- **Controller**: 控制决策模块，计算控制命令
- **Replayer**: 数据回放模块，用于离线调试与分析

## 依赖库
- OpenCV 4.8.0+
- Eigen3
- Boost
- spdlog
- OpenVINO opset>14.0
- Ceres

## 构建与安装
### 环境准备
OpenVINO的升级建议使用官网APT安装方法。
### 编译
```bash
mkdir build && cd build
cmake .. && make
```

编译后的可执行文件将生成在 `bin` 目录下。

## 运行
```bash
cd bin
./AutoAim
```

## 配置说明
系统配置文件位于 `config.json`，主要参数包括：

- 相机参数
- 检测器模型路径
- 串口设置
- 算法参数

可以使用提供的 Web 配置工具进行图形化配置：
```bash
cd utils/scripts
python server.py
```

然后在浏览器中访问 http://127.0.0.1:8080 即可。

## 工具库

系统包含多个实用工具：

- **Log**: 日志记录系统，基于 spdlog
- **VideoStreamer**: 视频流处理与网络传输
- **Udpsend**: UDP 数据发送工具
- **Param**: JSON 配置参数管理
- **TimeStamp**: 时间戳处理
- **Location**: 位置信息与坐标转换
- **Recorder**: 视频录制与回放

## 开发指南

### 代码结构
```
AutoAim/
├── modules/                    # 核心功能模块
│   ├── ControlOptimizer/       # 主要是优化作用
│   │   ├── CMakeLists.txt
│   │   ├── ControlOptimizer.cpp
│   │   ├── ControlOptimizer.hpp
│   │   └── README.md
│   ├── RecordSolver/           # 记录解算模块
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── recordsolver.cpp
│   │   └── recordsolver.hpp
│   ├── buff/                   # 打符模块，分红蓝，预测+s控制
│   │   ├── BuffCalculator.cpp
│   │   ├── BuffCalculator.hpp
│   │   ├── BuffController.cpp
│   │   ├── BuffController.hpp
│   │   ├── BuffDetector.cpp
│   │   ├── BuffDetector.hpp
│   │   └── CMakeLists.txt
│   ├── controller/             # 控制模块
│   │   ├── CMakeLists.txt
│   │   ├── controller.cpp
│   │   ├── controller.hpp
│   │   ├── controller2.cpp
│   │   └── type.hpp
│   ├── detector/               # 检测模块
│   │   ├── ArmorOneStage.cpp
│   │   ├── ArmorOneStage.hpp
│   │   ├── CMakeLists.txt
│   │   ├── NumberClassifier.cpp
│   │   ├── NumberClassifier.hpp
│   │   ├── YoloDetector.cpp
│   │   ├── YoloDetector.hpp
│   │   ├── detector.cpp
│   │   ├── detector.hpp
│   │   └── type.hpp
│   ├── driver/                 # 驱动模块
│   │   ├── CMakeLists.txt
│   │   ├── driver.camera/
│   │   │   ├── DaHengCamera.cpp
│   │   │   ├── DaHengCamera.hpp
│   │   │   ├── DxImageProc.h
│   │   │   ├── GxIAPI.h
│   │   │   ├── camera.cpp
│   │   │   └── camera.hpp
│   │   ├── driver.cpp
│   │   ├── driver.hpp
│   │   ├── driver.serial/
│   │   │   ├── CRC16.cpp
│   │   │   ├── CRC16.h
│   │   │   ├── serial.cpp
│   │   │   └── serial.hpp
│   │   └── type.hpp
│   ├── modules.hpp
│   ├── predictor/              # 预测模块
│   │   ├── CMakeLists.txt
│   │   ├── MotionModel.cpp
│   │   ├── MotionModel.hpp
│   │   ├── ekf.hpp
│   │   ├── old/
│   │   ├── old2/
│   │   ├── old3/
│   │   ├── old_new/
│   │   ├── predictor.cpp
│   │   ├── predictor.hpp
│   │   ├── timeEKF.hpp
│   │   └── type.hpp
│   ├── replayer/              # 回放模块
│   │   ├── CMakeLists.txt
│   │   ├── replayer.cpp
│   │   └── replayer.hpp
│   ├── solver/                # 解算模块
│   │   ├── CMakeLists.txt
│   │   ├── solver.cpp
│   │   ├── solver.hpp
│   │   └── type.hpp
│   └── tracker/               # 跟踪模块
│       ├── CMakeLists.txt
│       ├── TrackerMatcher.hpp
│       ├── TrackerMatcherWithWholeCar.hpp
│       ├── old/
│       ├── tracker.cpp
│       ├── tracker.hpp
│       └── type.hpp
├── utils/                     # 工具库
│   ├── buff_models/           # 打符模型目录
│   │   ├── blue_buff_debug/
│   │   ├── blue_buff_useless/
│   │   ├── blue_model_best/
│   │   ├── buff_mix_416_openvino_model/
│   │   └── red_buff/
│   ├── include/               # 工具头文件目录
│   │   ├── Location/
│   │   │   └── location.hpp
│   │   ├── Log/
│   │   │   └── log.hpp
│   │   ├── Param/
│   │   │   └── param.hpp
│   │   ├── README.md
│   │   ├── Recorder/
│   │   │   └── recorder.hpp
│   │   ├── TimeStamp/
│   │   │   └── TimeStamp.hpp
│   │   ├── Udpsend/
│   │   │   └── udpsend.hpp
│   │   ├── VideoStreamer/
│   │   │   ├── VideoStreamer.hpp
│   │   │   └── httplib.h
│   │   └── nlohmann/
│   │       └── json.hpp
│   ├── models/                # 模型文件目录
│   │   ├── BRpoints_nano.bin
│   │   ├── BRpoints_nano.xml
│   │   ├── armor_yolo_x.bin
│   │   ├── armor_yolo_x.xml
│   │   ├── new_model.bin
│   │   ├── new_model.xml
│   │   ├── svm_numbers_rbf.xml
│   │   ├── yolo11n-416sgd4.bin
│   │   └── yolo11n-416sgd4.xml
│   └── scripts/               # 脚本目录
│       ├── editor.html
│       ├── removeZoneIdentifier.sh
│       ├── reverseInternet.ps1
│       ├── server.log
│       ├── server.py
│       ├── solverFit/
│       │   └── process-PitchYawFit.py
│       └── startGTK.sh
├── record/                    # 记录目录
│   ├── serial/                # 串口记录
│   ├── test1.mkv              # 测试视频
│   └── video/                 # 视频记录
├── bin/                       # 编译输出目录
├── CMakeLists.txt             # CMake 配置文件
├── config.json                # 配置文件
├── json.hpp                   # JSON 头文件
└── AutoAim.cpp                # 主程序文件
```

### 模块扩展
要扩展特定模块功能，可以参考每个模块下的类型定义和接口说明。每个模块都设计了清晰的接口，便于功能扩展。

## 调试工具

- **日志调试**: 使用 `Log` 模块记录调试信息
- **参数调整**: 通过 Web 界面实时调整参数 + 查看视频
- **录制回放**: 使用 Recorder 工具录制数据，用 Replayer 模块回放分析

## 注意事项
- 确保相机和串口设备权限正确

### TODO
查看 `TODO.md` 文件，了解当前开发进度，问题，待办事项。