代码的一个显著特点是主程序仅进行所谓的**知识**的更新，即获取并处理传感器数据。  
而对车体的运动控制是由另一个线程完成的，也就是下面伪代码中注册的controlThread。   
这个线程会在每次获取到新的IMU数据时被调用，因此控制量发送的频率等于接收的频率。  
这样做可以提升控制的频率（250Hz），实现更丝滑的控制。
因此，要添加一个模块比如打符，为了维持代码逻辑不变，又考虑到打符作为一个完整的功能，且维护人不同，应该将打符作为一个完整的新模块，
但**提供状态更新接口和控制量计算接口，而不是每次状态更新后再进行控制**。
下面是伪代码建议：
原逻辑：
```C++
// AutoAim伪代码，仅供参考
main()
{
    auto param = readJson();
    auto modules = createModules(param);// 事实上模块并没有一个完整的父类，仅为简略显示
    modules.startSerialThread(); // 开启串口线程
    modules.startCameraThread(); // 开启相机线程
    modules.controlThread = [](imuData imu)
    {
        auto movement = modules.calcControl(imu);
        modules.sendControl(movement);
    };
    modules.startControlThread(); // 开启控制线程
    while(true)
    {
        if(existNewCameraData())
        {
            auto frame, imu = getCameraData();
            auto trackResult = modules.solve_track(frame, imu);
            modules.predictUpdate(trackResult);
        }
    }
}
```
```C++
// AutoAim+逻辑1打符伪代码
// 为了性能提升，可以测试两种执行方法。
// 1. 打符更新时不更新车辆状态
// 2. 打符更新时更新车辆状态
// 前者性能更好（需要吗？），后者可以做到更快的打符后击打车辆（打符时的仰角也许看不到其他车辆？）。
// 可能需要测试。
main()
{
    auto param = readJson();
    auto modules = createModules(param);
    auto module_buff = createBuff(param);
    modules.startSerialThread(); // 开启串口线程
    modules.startCameraThread(); // 开启相机线程
    modules.controlThread = [](imuData imu)
    {
        bool hitBuff = imu.aim_request == 2;
        auto movement = hitBuff?module_buff.calcControl(imu):modules.calcControl(imu);
        modules.sendControl(movement);
    };
    modules.startControlThread(); // 开启控制线程
    while(true)
    {
        if(existNewCameraData())
        {
            auto frame, imu = getCameraData();
            bool hitBuff = imu.aim_request == 2;
            BuffUpdataThread = std::thread([&]()
            {
                module_buff.updateState(frame, imu);
            });
            if(!EfficiencyFirst || !hitBuff)//性能优先时不进行装甲板检测
            {
                auto trackResult = modules.detect_solve_track(frame, imu);
                modules.predictUpdate(trackResult);
                modules.updateState(); // 更新状态
            }
        }
    }
}
```
