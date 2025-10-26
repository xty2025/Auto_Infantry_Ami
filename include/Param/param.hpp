#pragma once
#include <string>
#include <fstream>
#include <stdexcept>
#include <Log/log.hpp>
#include <nlohmann/json.hpp>  // 请确保已安装 nlohmann/json 库

//usage:    param.get<std::string>("model_path")
//          param["model_path"].to<std::string>()
//          param["model_path"].String()
namespace param{

    class Param {
    public:
        // 构造函数：从指定文件中加载 JSON 参数
        //explicit：防止被隐式转换。
        explicit Param(const std::string & filePath) {
            std::ifstream in(filePath);
            if (!in.is_open()) {
                ERROR("无法打开文件：{}", filePath);
            }
            try {
                in >> json_;
            } catch (const nlohmann::json::parse_error & e) {
                ERROR("JSON解析错误: {}", e.what());
            }
        }
        
        // 支持层次调用，返回子级 Param 对象
        Param operator[](const std::string & key) const {
            if (!json_.contains(key)) {
                ERROR("参数不存在：{}", key);
            }
            return Param(json_.at(key), InternalTag::Internal);
        }
        /*
        Param operator{}(const std::string &key) const{
            if(!json_.contains(key)){
                ERORR("None:{}",key);
            }
            return Param(json_.at(key),InternalTag::Internal);
        }*/

        // 模板接口：根据参数名称获取对应类型的值
        template<typename T>
        T get(const std::string & key) const {
            if (!json_.contains(key)) {
                ERROR("参数不存在：{}", key);
            }
            return json_.at(key).template get<T>();
            //使用nlohmann::json的模板get方法将值转换为指定类型并返回
        }

        template<typename T>
        T to() const {
            return json_.template get<T>();
            //与get方法不同，to方法不接受键名参数，而是转换当前对象本身
        }
        
        // 判断某参数是否存在
        bool exists(const std::string & key) const {
            return json_.contains(key);
        }

        int Int() const {
            return to<int>();
        }

        float Float() const {
            return to<float>();
        }
        
        double Double() const {
            return to<double>();
        }
        
        bool Bool() const {
            return to<bool>();
        }

        std::string String() const {
            return to<std::string>();
        }
        //转为string

    private:
        // 内部标识类型，用于区分内部构造函数
        enum class InternalTag { Internal };

        // 私有构造函数，通过 JSON 对象构造 Param（用于 operator[] 链式调用）
        //接受json 里的key命，返回键值。
        explicit Param(const nlohmann::json & j, InternalTag) : json_(j) {}

        nlohmann::json json_;
        //内部声明，周期与Param一样。
    };

}