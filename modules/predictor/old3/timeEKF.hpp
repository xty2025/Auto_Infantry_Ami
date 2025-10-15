// 包装了ekf.hpp，用于更方便的时间序列预测
#pragma once
#include "ekf.hpp"
#include "TimeStamp/TimeStamp.hpp"

namespace ekf::timeEKF
{
template<class stateTransFunc, class measureFunc,int N_X, int N_Y>
class TimeEKF 
{
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    public:
    // predict func there will not update inner state
    // which is significant different from so-called predict in EKF
    VectorX predict(const Time::TimeStamp& timestamp)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt((timestamp - lastTime).toSeconds());
        stateTrans(X, Xp);
        return Xp;
    }
    VectorX predict(double dt)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt(dt);
        stateTrans(X, Xp);
        return Xp;
    }
    // dont check whether timestamp initialized, just take care of it by yourself 
    VectorX update(const VectorY& Y, const Time::TimeStamp& timestamp, int id)
    {
        stateTrans.setDt((timestamp - lastTime).toSeconds());
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure.setId(id);
        return ekf.update(measure, Y);
    }
    
    void init(const MatrixXX& P, const MatrixXX& Q, const MatrixYY& R)
    {
        ekf.P = P;
        ekf.Q = Q;
        ekf.R = R;
    }
    void setX(const VectorX& X)
    {
        ekf.Xe = X;
    }
    void setTimeStamp(const Time::TimeStamp& timestamp)
    {
        lastTime = timestamp;
    }
    //get inner state
    VectorX getState()
    {
        return ekf.Xe;
    }
    //get inner covariance
    MatrixXX getCovariance()
    {
        return ekf.P;
    }
    private:
        base::EKF<N_X, N_Y> ekf;
        stateTransFunc stateTrans;
        measureFunc measure;
        Time::TimeStamp lastTime;
};

template<class stateTransFunc, class measureFunc1, class measureFunc2, int N_X, int N_Y1, int N_Y2>
class TimeBMEKF
{
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY1 = Eigen::Matrix<double, N_Y1, 1>;
    using VectorY2 = Eigen::Matrix<double, N_Y2, 1>;
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYY1 = Eigen::Matrix<double, N_Y1, N_Y1>;
    using MatrixYY2 = Eigen::Matrix<double, N_Y2, N_Y2>;
    public:
    VectorX predict(const Time::TimeStamp& timestamp)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt((timestamp - lastTime).toSeconds());
        stateTrans(X.data(), Xp.data());
        return Xp;
    }
    VectorX predict(double dt)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt(dt);
        stateTrans(X.data(), Xp.data());
        return Xp;
    }
    VectorX update(const VectorY1& Y1, const Time::TimeStamp& timestamp, int id)
    {
        stateTrans.setDt((timestamp - lastTime).toSeconds());
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure1.setId(id);
        return ekf.update(measure1, Y1);
    }
    VectorX update(const VectorY2& Y2, const Time::TimeStamp& timestamp, int id1, int id2)
    {
        stateTrans.setDt((timestamp - lastTime).toSeconds());
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure2.setId(id1,id2);
        return ekf.update(measure2, Y2);
    }

    void init(const MatrixXX& P, const MatrixXX& Q, const MatrixYY1& R1, const MatrixYY2& R2)
    {
        ekf.P = P;
        ekf.Q = Q;
        ekf.R1 = R1;
        ekf.R2 = R2;
    }
    void setX(const VectorX& X)
    {
        ekf.Xe = X;
    }

    private:
        base::BMEKF<N_X, N_Y1, N_Y2> ekf;
        stateTransFunc stateTrans;
        measureFunc1 measure1;
        measureFunc2 measure2;
        Time::TimeStamp lastTime;
};
}

