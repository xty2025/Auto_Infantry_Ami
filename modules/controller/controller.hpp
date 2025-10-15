
#include "type.hpp"
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "solver/type.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include "Param/param.hpp"
#include<stack>
//核心计算函数：
// bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
//       std::chrono::duration<double> flyTime = 0.0s;
//声明一个函数对象，返回类型为Predictions(内部匹配的类型)：
/*
std::stack<int>stk
std::function<Predictions(Param::param)>PredictionFunc()
std::function<Prediction(Time::TimeStamp)>PredictionFunc2()
*/


namespace controller
{
    using predictor::Armor;
    using predictor::Prediction;
    using predictor::Predictions;
    using driver::ParsedSerialData;
    using solver::ImuData;
    const double PI = 3.1415926;
    const double GRAVITY = 9.794;
    const double C_D = 0.42;//空气阻力
    const double RHO = 1.169;//空气密度
    class Controller 
    {
    public:
        //Controller(param::Param &a):a(a){readJsonParam()};
        Controller(param::Param& json_param) : json_param(json_param) {readJsonParam();}
        /*构造函数声明：Controller(param::Param& json_param) 声明了 Controller 类的一个构造函数，它接受一个 param::Param 类型的引用参数 json_param。
初始化列表：: json_param(json_param) 是构造函数的初始化列表，用于初始化类的成员变量：
冒号 : 后面的部分是初始化列表
json_param(json_param) 表示用构造函数的参数 json_param 来初始化类的成员变量 json_param
这里两个 json_param 分别指：成员变量（左侧）和构造函数参数（右侧）
构造函数体：{readJsonParam();} 是构造函数的函数体，在初始化列表完成成员变量初始化后，会执行 readJsonParam() 函数（通常用于进一步的参数读取或初始化操作）。*/
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        //
        ControlResult control(const ParsedSerialData& parsedData);
    private:
        void readJsonParam(void);
        //从json读取，@param json_param
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        param::Param json_param;
        bool aim_new = false;
        bool aiming = false;
        const int waitFrame = 5;
        int accumulate_aim_request = 0;
        std::pair<int, int> aim_armor_id = {-1, -1};//(car, armor)
        int max_iter = 100;
        double tol = 1e-6;
        //子弹的物理参数。
        double bullet_mass = 3.2e-3;
        double bullet_diameter = 16.8e-3;
        bool judgeAimNew(bool request);
        //核心函数：解算。
        bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        std::chrono::duration<double> flyTime = 0.0s;

        double bullet_speed = 23.0;
        const double min_bullet_speed = 20.0;
        const double bullet_speed_alpha = 0.5;

        std::chrono::duration<double> shootDelay = 0.1s;

        double tol_deltax = 0.2;
        double tol_deltay = 0.1;
        double response_speed = 0.5; // 0.5 rad/s
        double armor_yaw_allow = 45.0;
        bool mouse_require = false;
        double pic_camera_x = 640.0; //图传模块中心点在相机坐标系中的坐标
        double pic_camera_y = 512.0; //图传模块中心点在相机坐标系中的坐标
    };

    std::shared_ptr<Controller> createController(param::Param json_param);
    //std::unique_ptr<Controller>createController2(param::Param json_param);
} // namespace controller