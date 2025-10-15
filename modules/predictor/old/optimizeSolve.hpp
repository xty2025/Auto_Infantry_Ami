#include <Log/log.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>
#include <mutex>

namespace StateFitting
{
    using namespace aimlog;
    class OptimizeSolve
    {
    public:
        typedef CPPAD_TESTVECTOR( CppAD::AD<double> ) ADvector;
        typedef CPPAD_TESTVECTOR(double) Dvector;
        OptimizeSolve()
        {
            //关闭任何优化器的输出
            options += "Integer print_level  0\n";
            options += "String  sb           yes\n";
            //设置最大迭代次数
            options += "Integer max_iter     200\n";
            // 设置收敛条件
            // see Mathematical Programming, Volume 106, Number 1,
            // Pages 25-57, Equation (6)
            options += "Numeric tol          1e-10\n";
        }
        void setBoundInit(Eigen::Matrix3Xd &bound)// 0: lower bound, 1: upper bound, 2: initial value
        {
            if(state_dim != 0 && state_dim != bound.cols())
                ERROR("Recieve: {}, but expect: {}", bound.cols(), state_dim);
            state_dim = bound.cols();
            state_lower_bound.resize(state_dim);
            state_upper_bound.resize(state_dim);
            state.resize(state_dim);
            for (int i = 0; i < state_dim; i++)
            {
                state_lower_bound[i] = bound(0, i);
                state_upper_bound[i] = bound(1, i);
                state[i] = bound(2, i);
            }
        }
        void setStateCoef(Eigen::VectorXd &state_coef_)
        {
            if(state_dim != 0 && state_dim != state_coef_.size())
                ERROR("Recieve: {}, but expect: {}", state_coef_.size(), state_dim);
            state_dim = state_coef_.size();
            state_coef.resize(state_dim);
            for (int i = 0; i < state_dim; i++)
            {
                state_coef[i] = state_coef_(i);
            }
        }
        void setMeasureCoef(Eigen::VectorXd &measure_coef_)
        {
            if(measure_dim != 0 && measure_dim != measure_coef_.size())
                ERROR("Recieve: {}, but expect: {}", measure_coef_.size(), measure_dim);
            measure_dim = measure_coef_.size();
            measure_coef.resize(measure_dim);
            for (int i = 0; i < measure_dim; i++)
            {
                measure_coef[i] = measure_coef_(i);
            }
        }
        void setHistorySizeAndLoss(double max_loss, double min_loss, int max_size, int min_size)
        {
            loss_max = max_loss;
            loss_min = min_loss;
            max_history_size = max_size;
            min_history_size = min_size;
        }
        virtual ADvector ideal_stateUpdate(const ADvector& state, const double dt) = 0;
        virtual ADvector ideal_measure(const ADvector& state, const int measure_id) = 0;
        //由于CppAD多线程支持很繁琐，故要求使用者多写一个Eigen版本的函数以提供预测功能
        //使用者应自行保证两个版本函数的一致性
        virtual Eigen::VectorXd ideal_stateUpdate(const Eigen::VectorXd& state, const double dt) = 0;
        virtual Eigen::VectorXd ideal_measure(const Eigen::VectorXd& measure, const int measure_id) = 0;
        virtual Dvector first_state_estimate(const Dvector& measure, const int measure_id) = 0;
        void lockState(){state_mutex.lock();}
        void unlockState(){state_mutex.unlock();}
        Eigen::VectorXd getCurrentState(){return eigen_state;}
        bool Update(const Dvector &measure, const Time::TimeStamp &timestamp, const int measure_id);
        double timestamp2dt(const Time::TimeStamp &timestamp){return (timestamp - last_timestamp).toSeconds();}
        CppAD::AD<double> loss(const ADvector &state_);
        void operator()(ADvector& constraints, const ADvector& state)
        {
            constraints[0] = loss(state);
        }
        
    private:

        //FIX PARAMETER
        std::string options;
        int state_dim = 0;
        Dvector state_lower_bound;
        Dvector state_upper_bound;
        Dvector state_coef;
        int measure_dim = 0;
        Dvector measure_coef;
        int max_history_size,min_history_size;
        double loss_max,loss_min;//if loss > loss_max, then current_history_size change to min_history_size

        //RUNTIME VARIABLE
        int current_history_size;
        double decay_rate;
        std::vector<Dvector> history_measure;
        std::vector<Time::TimeStamp> history_timepoint;
        std::vector<int> history_measure_id;
        Dvector state;

        //MUTEX for state
        std::mutex state_mutex;
        Eigen::VectorXd eigen_state;
        Time::TimeStamp last_timestamp;

        Dvector ideal_stateUpdate_noAD(const Dvector& state, const double dt);
        Dvector ideal_measure_noAD(const Dvector& state, const int measure_id);
    };
    using Dvector = OptimizeSolve::Dvector;
    using ADvector = OptimizeSolve::ADvector;
}