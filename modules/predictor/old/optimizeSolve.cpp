#include "optimizeSolve.hpp"
#include <cppad/ipopt/solve.hpp>

namespace StateFitting {

bool OptimizeSolve::Update(const Dvector &measure, const Time::TimeStamp &timestamp, const int measure_id) {
    if(history_measure.size() >= max_history_size) {
        history_measure.erase(history_measure.begin());
        history_timepoint.erase(history_timepoint.begin());
        history_measure_id.erase(history_measure_id.begin());
    }
    history_measure.push_back(measure);
    history_timepoint.push_back(timestamp);
    history_measure_id.push_back(measure_id);
    if(history_measure.size() <= 1) {
        state = first_state_estimate(measure, measure_id);
        eigen_state = Eigen::Map<Eigen::VectorXd>(state.data(), state.size());
        return true;
    }
    state = ideal_stateUpdate_noAD(state, (timestamp - history_timepoint[history_timepoint.size() - 2]).toSeconds());
    Dvector predicted_measure = ideal_measure_noAD(state, measure_id);
    double current_step_measure_loss = 0.0;
    for(size_t i = 0; i < measure.size(); ++i) {
        current_step_measure_loss += (predicted_measure[i] - measure[i]) * (predicted_measure[i] - measure[i]) * measure_coef[i];
    }
    if(current_step_measure_loss > loss_max) {
        current_history_size = min_history_size;
    }
    else if(current_step_measure_loss < loss_min) {
        current_history_size = max_history_size;
    }
    decay_rate = std::pow(0.01, 1.0 / current_history_size);
    
    CppAD::ipopt::solve_result<Dvector> solution;
    Dvector gl_nouse, gu_nouse;
    CppAD::ipopt::solve(options, state, state_lower_bound, state_upper_bound, gl_nouse, gu_nouse, *this, solution);
    bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if(!ok) WARN("Optimization failed, status: {}", static_cast<int>(solution.status));
    state = solution.x;
    std::lock_guard<std::mutex> lock(state_mutex);
    eigen_state = Eigen::Map<Eigen::VectorXd>(state.data(), state.size());
    last_timestamp = timestamp;
    return ok;
}


CppAD::AD<double> OptimizeSolve::loss(const ADvector &state_) {
    // 计算损失值
    CppAD::AD<double> state_change_loss = 1.0; 
    for(size_t i = 0; i < state_dim; ++i) {
        state_change_loss += (state[i] - state_[i]) * (state[i] - state_[i]) * state_coef[i];
    }
    CppAD::AD<double> measure_loss = 0.0;
    double decay = 1.0;
    int size = history_measure.size();
    for(int i = size - 1; i >= size - current_history_size; --i) {
        if(i < 0) break;
        ADvector state_update = ideal_stateUpdate(state_, (history_timepoint[i] - history_timepoint[size - 1]).toSeconds());
        ADvector measure = ideal_measure(state_update, history_measure_id[i]);
        for(size_t j = 0; j < measure.size(); ++j) {
            measure_loss += (measure[j] - history_measure[i][j]) * (measure[j] - history_measure[i][j]) * measure_coef[j] * decay;
        }
        decay *= decay_rate;
    }
    return measure_loss * state_change_loss;
}

Dvector OptimizeSolve::ideal_stateUpdate_noAD(const Dvector& state, const double dt)
{
    ADvector state_AD(state.size());
    for (int i = 0; i < state.size(); i++)
    {
        state_AD[i] = state[i];
    }
    ADvector state_update = ideal_stateUpdate(state_AD, dt);
    Dvector state_update_noAD(state_update.size());
    for (int i = 0; i < state_update.size(); i++)
    {
        state_update_noAD[i] = CppAD::Value(state_update[i]);
    }
    return state_update_noAD;
}
Dvector OptimizeSolve::ideal_measure_noAD(const Dvector& state, const int measure_id)
{
    ADvector state_AD(state.size());
    for (int i = 0; i < state.size(); i++)
    {
        state_AD[i] = state[i];
    }
    ADvector measure = ideal_measure(state_AD, measure_id);
    Dvector measure_noAD(measure.size());
    for (int i = 0; i < measure.size(); i++)
    {
        measure_noAD[i] = CppAD::Value(measure[i]);
    }
    return measure_noAD;
}

} // namespace StateFitting
