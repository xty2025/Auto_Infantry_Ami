#ifndef CONTROL_OPTIMIZER_HPP
#define CONTROL_OPTIMIZER_HPP

#include <functional>
#include <vector>
#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <thread>

class ControlOptimizer {
public:
    // Type definitions for callback functions
    using ControlFunc = std::function<void(double pitch, double yaw)>;
    using PositionFunc = std::function<std::pair<double, double>()>; // Returns current pitch, yaw

    // Test result structure
    struct TestResult {
        double deadzone_pitch;
        double deadzone_yaw;

        struct PointResponse {
            double amplitude;
            double first_arrival_time;  // Time to first reach target
            double settling_time;       // Time to stabilize
            double average_speed;       // Average speed during movement
        };
        std::vector<PointResponse> point_responses_pitch;
        std::vector<PointResponse> point_responses_yaw;

        struct WaveResponse {
            double frequency;
            double amplitude;
            double phase_delay;     // Time delay
            double amplitude_loss;  // Percentage
            double tracking_error;  // Average error
        };
        std::vector<WaveResponse> sine_responses_pitch;
        std::vector<WaveResponse> sine_responses_yaw;
        std::vector<WaveResponse> sawtooth_responses_pitch;
        std::vector<WaveResponse> sawtooth_responses_yaw;
    };

    // Constructor
    ControlOptimizer(ControlFunc send_control, PositionFunc get_position);

    // Main test methods
    void testAll();
    void testDeadZone();
    void testPointResponse();
    void testSineResponse();
    void testSawtoothResponse();

    // Settings
    void setTestAmplitudes(const std::vector<double>& amplitudes);
    void setSineFrequencies(const std::vector<double>& frequencies);

    // Get results
    const TestResult& getResults() const;

private:
    // Callback functions
    ControlFunc sendControl;
    PositionFunc getPosition;

    // Test parameters
    static constexpr double DRIFT_THRESHOLD = 0.01;        // Degrees
    static constexpr double STABILITY_TOLERANCE = 0.05;    // Degrees
    static constexpr int POSITION_CHECK_INTERVAL = 5;      // Milliseconds
    static constexpr int MAX_TEST_TIME = 10000;            // Milliseconds per test

    std::vector<double> testAmplitudes = {2.0, 5.0, 10.0}; // Degrees
    std::vector<double> sineFrequencies = {0.5, 1.0, 2.0}; // Hz

    // Initial position storage
    double initialPitch;
    double initialYaw;

    // Test results
    TestResult results;

    // Helper methods
    void resetToInitialPosition();
    void recordInitialPosition();
    static bool isStable(double current, double target);
    void waitForStabilization(double target_pitch, double target_yaw);
    double measureResponseTime(double start_time, double target, const std::function<double()>& get_value);
};

#endif // CONTROL_OPTIMIZER_HPP