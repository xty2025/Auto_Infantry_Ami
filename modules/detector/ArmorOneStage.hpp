#pragma once
#include <iostream>
#include "type.hpp"
#include "openvino/openvino.hpp"
#include "NumberClassifier.hpp"
#include <Eigen/Eigen>

namespace detector
{    
    
    class ArmorOneStage
    {
    public:
        explicit ArmorOneStage(const std::string &model_file,bool useOldModel = false,bool allowGray = true);

        ~ArmorOneStage();

        BBoxes operator()(const cv::Mat &img);
        //重载（）运算符。
        //cv::Mat getProposalPic();
        void setColorFlag(int flag_)
        {
            if(flag_ == 0) // enemy_red
            {
                this->color_flag = 1;
                std::cout << "ENEMY RED" << std::endl;
            }
            else if(flag_ == 1)
            {
                this->color_flag = 0;
                std::cout << "ENEMY BLUE" << std::endl;
            }
            else if(flag_ == -2)
            {
                this->color_flag = -2;
                std::cout << "ENEMY ALL (EXPERIMENTAL)" << std::endl;
            }
            else if(flag_ == -3)
            {
                this->color_flag = -3;
                std::cout << "ENEMY R&B (EXPERIMENTAL)" << std::endl;
            }
            else
            {
                this->color_flag = 0;
                std::cout << "ENEMY BLUE" << std::endl;
            }
        };
    private:
        std::unique_ptr<NumberClassifier> number_classifier;

        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;

        void initModel(const std::string &model_file);
        //blue: 0, red: 1, gray: 2, purple: 3
        int color_flag = -1;
        bool allowGray = true;
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };
} //namespace DETECTOR