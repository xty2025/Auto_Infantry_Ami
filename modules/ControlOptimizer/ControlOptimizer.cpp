#include "ControlOptimizer.hpp"

ControlOptimizer::ControlOptimizer(ControlFunc send_control, PositionFunc get_position)
        : sendControl(send_control), getPosition(get_position) {
    // Initialize with empty results
    results.deadzone_pitch = 0.0;
    results.deadzone_yaw = 0.0;
}

void ControlOptimizer::recordInitialPosition() {
    auto [pitch, yaw] = getPosition();
    initialPitch = pitch;
    initialYaw = yaw;
    std::cout << "Initial position recorded: Pitch=" << initialPitch
              << "°, Yaw=" << initialYaw << "°" << std::endl;
}

void ControlOptimizer::resetToInitialPosition() {
    std::cout << "Resetting to initial position..." << std::endl;
    sendControl(initialPitch, initialYaw);
    waitForStabilization(initialPitch, initialYaw);
    std::cout << "Reset complete" << std::endl;
}

bool ControlOptimizer::isStable(double current, double target) {
    return std::abs(current - target) <= STABILITY_TOLERANCE;
}

void ControlOptimizer::waitForStabilization(double target_pitch, double target_yaw) {
    std::cout << "Waiting for stabilization..." << std::endl;

    int stable_count = 0;
    const int required_stable_readings = 10; // Require multiple stable readings

    while (stable_count < required_stable_readings) {
        auto [current_pitch, current_yaw] = getPosition();

        if (isStable(current_pitch, target_pitch) && isStable(current_yaw, target_yaw)) {
            stable_count++;
        } else {
            stable_count = 0; // Reset if unstable
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
    }
}

double ControlOptimizer::measureResponseTime(double start_time, double target, const std::function<double()>& get_value) {
    auto start = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + MAX_TEST_TIME;

    while (std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000 < end_time) {
        double current = get_value();

        if (isStable(current, target)) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            return elapsed;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
    }

    return MAX_TEST_TIME; // Timeout
}

void ControlOptimizer::testDeadZone() {
    std::cout << "\n----- TESTING DEAD ZONE -----" << std::endl;

    // Test for each axis (0 = pitch, 1 = yaw)
    for (int axis = 0; axis < 2; axis++) {
        std::string axis_name = (axis == 0) ? "pitch" : "yaw";
        std::cout << "Testing " << axis_name << " dead zone..." << std::endl;

        // Reset to initial position
        resetToInitialPosition();

        // Get stable initial position
        auto [init_pitch, init_yaw] = getPosition();
        double init_pos = (axis == 0) ? init_pitch : init_yaw;

        // Increment the control signal gradually
        double control_value = 0.0;
        double increment = 0.01; // Small increment for precision
        double threshold_exceeded = false;
        double deadzone = 0.0;

        while (control_value <= 2.0 && !threshold_exceeded) { // Upper limit for safety
            // Apply control
            if (axis == 0) {
                sendControl(initialPitch + control_value, initialYaw);
            } else {
                sendControl(initialPitch, initialYaw + control_value);
            }

            // Wait a moment for potential response
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Check if position changed beyond drift threshold
            auto [current_pitch, current_yaw] = getPosition();
            double current_pos = (axis == 0) ? current_pitch : current_yaw;

            if (std::abs(current_pos - init_pos) > DRIFT_THRESHOLD) {
                threshold_exceeded = true;
                deadzone = control_value;
                std::cout << "  Movement detected at control value: " << control_value << std::endl;
            } else {
                // Increment control value
                control_value += increment;
            }
        }

        // Store the result
        if (axis == 0) {
            results.deadzone_pitch = deadzone;
        } else {
            results.deadzone_yaw = deadzone;
        }

        std::cout << "  " << axis_name << " dead zone: " << deadzone << std::endl;

        // Reset to initial position
        resetToInitialPosition();
    }
}

void ControlOptimizer::testPointResponse() {
    std::cout << "\n----- TESTING POINT RESPONSE -----" << std::endl;

    // Test for each axis (0 = pitch, 1 = yaw)
    for (int axis = 0; axis < 2; axis++) {
        std::string axis_name = (axis == 0) ? "pitch" : "yaw";
        std::cout << "Testing " << axis_name << " point response..." << std::endl;

        for (double amplitude : testAmplitudes) {
            std::cout << "  Testing amplitude: " << amplitude << "°" << std::endl;

            // Reset to initial position
            resetToInitialPosition();

            // Prepare test result record
            TestResult::PointResponse response;
            response.amplitude = amplitude;

            // Get current time for start reference
            auto start_time = std::chrono::high_resolution_clock::now();

            // Calculate target position
            double target_pitch = initialPitch;
            double target_yaw = initialYaw;

            if (axis == 0) {
                target_pitch += amplitude;
            } else {
                target_yaw += amplitude;
            }

            // Send control command to target position
            sendControl(target_pitch, target_yaw);

            // Record start time in milliseconds
            auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    start_time.time_since_epoch()).count();

            // Measure time to first arrival
            bool target_reached = false;
            double first_arrival_time = 0.0;
            double settling_time = 0.0;

            auto timeout = start_ms + MAX_TEST_TIME;
            auto pos_check_start = start_ms;

            while (std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000 < timeout) {
                auto [current_pitch, current_yaw] = getPosition();
                double current_pos = (axis == 0) ? current_pitch : current_yaw;
                double target_pos = (axis == 0) ? target_pitch : target_yaw;

                // Check for first arrival
                if (!target_reached && isStable(current_pos, target_pos)) {
                    target_reached = true;
                    auto now = std::chrono::high_resolution_clock::now();
                    first_arrival_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()).count() - start_ms;

                    std::cout << "    First arrival at: " << first_arrival_time << " ms" << std::endl;
                    pos_check_start = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()).count();
                }

                // Check for settling (stable for a period)
                if (target_reached) {
                    int stable_count = 0;
                    bool settled = true;

                    // Check multiple consecutive readings for stability
                    for (int i = 0; i < 10; i++) {
                        auto [check_pitch, check_yaw] = getPosition();
                        double check_pos = (axis == 0) ? check_pitch : check_yaw;

                        if (!isStable(check_pos, target_pos)) {
                            settled = false;
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
                    }

                    if (settled) {
                        auto now = std::chrono::high_resolution_clock::now();
                        settling_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now.time_since_epoch()).count() - start_ms;

                        std::cout << "    Settled at: " << settling_time << " ms" << std::endl;
                        break;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
            }

            // Calculate average speed (degrees/second)
            response.first_arrival_time = first_arrival_time;
            response.settling_time = settling_time;
            response.average_speed = first_arrival_time > 0 ?
                                     (amplitude * 1000.0 / first_arrival_time) : 0.0;

            std::cout << "    Average speed: " << response.average_speed << "°/s" << std::endl;

            // Store the results
            if (axis == 0) {
                results.point_responses_pitch.push_back(response);
            } else {
                results.point_responses_yaw.push_back(response);
            }

            // Reset position
            resetToInitialPosition();
        }
    }
}

void ControlOptimizer::testSineResponse() {
    std::cout << "\n----- TESTING SINE RESPONSE -----" << std::endl;

    // Test for each axis (0 = pitch, 1 = yaw)
    for (int axis = 0; axis < 2; axis++) {
        std::string axis_name = (axis == 0) ? "pitch" : "yaw";
        std::cout << "Testing " << axis_name << " sine response..." << std::endl;

        for (double frequency : sineFrequencies) {
            std::cout << "  Testing frequency: " << frequency << " Hz" << std::endl;

            // Test amplitude for sine wave will be the middle value in testAmplitudes
            double amplitude = testAmplitudes[testAmplitudes.size() / 2];

            // Reset to initial position
            resetToInitialPosition();

            // Prepare test result record
            TestResult::WaveResponse response;
            response.frequency = frequency;
            response.amplitude = amplitude;

            // Store values for analysis
            std::vector<double> command_values;
            std::vector<double> response_values;
            std::vector<double> timestamps;

            // Calculate period and number of cycles to test
            double period_ms = 1000.0 / frequency; // Period in milliseconds
            int num_cycles = 3; // Test for 3 cycles
            int total_test_time = static_cast<int>(num_cycles * period_ms); // Total test time in milliseconds

            // Record initial time
            auto start_time = std::chrono::high_resolution_clock::now();
            auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    start_time.time_since_epoch()).count();
            auto end_time = start_ms + total_test_time;

            // Run test for specified number of cycles
            while (std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000 < end_time) {
                // Get current time relative to start
                auto now = std::chrono::high_resolution_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - start_time).count();
                double phase = 2.0 * M_PI * (elapsed_ms / period_ms);

                // Calculate sine wave control value
                double control_value = amplitude * sin(phase);

                // Apply control
                if (axis == 0) {
                    sendControl(initialPitch + control_value, initialYaw);
                } else {
                    sendControl(initialPitch, initialYaw + control_value);
                }

                // Get current position
                auto [current_pitch, current_yaw] = getPosition();
                double current_pos = (axis == 0) ? current_pitch - initialPitch : current_yaw - initialYaw;

                // Store values
                command_values.push_back(control_value);
                response_values.push_back(current_pos);
                timestamps.push_back(elapsed_ms);

                // Wait for next sample
                std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
            }

            // Calculate metrics (ignoring the first cycle for settling)
            int skip_samples = static_cast<int>(period_ms / POSITION_CHECK_INTERVAL);
            double phase_sum = 0.0;
            double amplitude_sum = 0.0;
            double error_sum = 0.0;
            int count = 0;

            // Find peaks in command and response to calculate phase delay and amplitude loss
            std::vector<int> cmd_peaks;
            std::vector<int> rsp_peaks;

            for (int i = skip_samples + 1; i < command_values.size() - 1; i++) {
                // Find command peaks
                if (command_values[i] > command_values[i-1] &&
                    command_values[i] > command_values[i+1] &&
                    command_values[i] > 0.9 * amplitude) {
                    cmd_peaks.push_back(i);
                }

                // Find response peaks
                if (response_values[i] > response_values[i-1] &&
                    response_values[i] > response_values[i+1] &&
                    response_values[i] > 0) {
                    rsp_peaks.push_back(i);
                }

                // Calculate error
                error_sum += std::abs(command_values[i] - response_values[i]);
                count++;
            }

            // Calculate phase delay from peaks
            double phase_delay = 0.0;
            int min_peaks = std::min(cmd_peaks.size(), rsp_peaks.size());

            if (min_peaks > 0) {
                for (int i = 0; i < min_peaks; i++) {
                    phase_delay += timestamps[rsp_peaks[i]] - timestamps[cmd_peaks[i]];
                }
                phase_delay /= min_peaks;
            }

            // Calculate amplitude loss from peaks
            double cmd_max = 0.0;
            double rsp_max = 0.0;

            for (int idx : cmd_peaks) {
                cmd_max += command_values[idx];
            }
            cmd_max /= cmd_peaks.size();

            for (int idx : rsp_peaks) {
                rsp_max += response_values[idx];
            }
            rsp_max /= rsp_peaks.size();

            double amplitude_loss = 100.0 * (1.0 - rsp_max / cmd_max);
            double avg_error = error_sum / count;

            // Store the results
            response.phase_delay = phase_delay;
            response.amplitude_loss = amplitude_loss;
            response.tracking_error = avg_error;

            std::cout << "    Phase delay: " << phase_delay << " ms" << std::endl;
            std::cout << "    Amplitude loss: " << amplitude_loss << "%" << std::endl;
            std::cout << "    Average tracking error: " << avg_error << "°" << std::endl;

            // Store the results
            if (axis == 0) {
                results.sine_responses_pitch.push_back(response);
            } else {
                results.sine_responses_yaw.push_back(response);
            }

            // Reset position
            resetToInitialPosition();
        }
    }
}

void ControlOptimizer::testSawtoothResponse() {
    std::cout << "\n----- TESTING SAWTOOTH RESPONSE -----" << std::endl;

    // Test for each axis (0 = pitch, 1 = yaw)
    for (int axis = 0; axis < 2; axis++) {
        std::string axis_name = (axis == 0) ? "pitch" : "yaw";
        std::cout << "Testing " << axis_name << " sawtooth response..." << std::endl;

        for (double frequency : sineFrequencies) {
            std::cout << "  Testing frequency: " << frequency << " Hz" << std::endl;

            // Test amplitude for sawtooth wave
            double amplitude = testAmplitudes[testAmplitudes.size() / 2];

            // Reset to initial position
            resetToInitialPosition();

            // Prepare test result record
            TestResult::WaveResponse response;
            response.frequency = frequency;
            response.amplitude = amplitude;

            // Store values for analysis
            std::vector<double> command_values;
            std::vector<double> response_values;
            std::vector<double> timestamps;

            // Calculate period and number of cycles to test
            double period_ms = 1000.0 / frequency; // Period in milliseconds
            int num_cycles = 3; // Test for 3 cycles
            int total_test_time = static_cast<int>(num_cycles * period_ms); // Total test time in milliseconds

            // Record initial time
            auto start_time = std::chrono::high_resolution_clock::now();
            auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    start_time.time_since_epoch()).count();
            auto end_time = start_ms + total_test_time;

            // Run test for specified number of cycles
            while (std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000 < end_time) {
                // Get current time relative to start
                auto now = std::chrono::high_resolution_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - start_time).count();

                // Calculate sawtooth wave value (ramp up quickly, fall down slowly)
                double phase = (elapsed_ms / period_ms) - floor(elapsed_ms / period_ms);
                double control_value = amplitude * (2.0 * phase - 1.0);

                // Apply control
                if (axis == 0) {
                    sendControl(initialPitch + control_value, initialYaw);
                } else {
                    sendControl(initialPitch, initialYaw + control_value);
                }

                // Get current position
                auto [current_pitch, current_yaw] = getPosition();
                double current_pos = (axis == 0) ? current_pitch - initialPitch : current_yaw - initialYaw;

                // Store values
                command_values.push_back(control_value);
                response_values.push_back(current_pos);
                timestamps.push_back(elapsed_ms);

                // Wait for next sample
                std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_CHECK_INTERVAL));
            }

            // Calculate metrics (ignoring the first cycle for settling)
            int skip_samples = static_cast<int>(period_ms / POSITION_CHECK_INTERVAL);
            double phase_sum = 0.0;
            double amplitude_sum = 0.0;
            double error_sum = 0.0;
            int count = 0;

            // Find peaks in command and response
            double cmd_max = -amplitude;
            double rsp_max = -amplitude;
            double cmd_min = amplitude;
            double rsp_min = amplitude;

            for (int i = skip_samples; i < command_values.size(); i++) {
                // Update max/min values
                if (command_values[i] > cmd_max) cmd_max = command_values[i];
                if (command_values[i] < cmd_min) cmd_min = command_values[i];
                if (response_values[i] > rsp_max) rsp_max = response_values[i];
                if (response_values[i] < rsp_min) rsp_min = response_values[i];

                // Calculate error
                error_sum += std::abs(command_values[i] - response_values[i]);
                count++;
            }

            // Calculate amplitude loss
            double cmd_amplitude = cmd_max - cmd_min;
            double rsp_amplitude = rsp_max - rsp_min;
            double amplitude_loss = 100.0 * (1.0 - rsp_amplitude / cmd_amplitude);

            // Estimate phase delay by finding time of max value in each cycle
            std::vector<double> cmd_max_times;
            std::vector<double> rsp_max_times;

            for (int cycle = 0; cycle < num_cycles; cycle++) {
                int start_idx = static_cast<int>((cycle * period_ms) / POSITION_CHECK_INTERVAL);
                int end_idx = static_cast<int>(((cycle + 1) * period_ms) / POSITION_CHECK_INTERVAL);

                if (end_idx >= command_values.size()) {
                    end_idx = static_cast<int>(command_values.size()) - 1;
                }

                // Find max in command
                int cmd_max_idx = start_idx;
                for (int i = start_idx + 1; i <= end_idx; i++) {
                    if (command_values[i] > command_values[cmd_max_idx]) {
                        cmd_max_idx = i;
                    }
                }

                // Find max in response
                int rsp_max_idx = start_idx;
                for (int i = start_idx + 1; i <= end_idx; i++) {
                    if (response_values[i] > response_values[rsp_max_idx]) {
                        rsp_max_idx = i;
                    }
                }

                // Store times
                cmd_max_times.push_back(timestamps[cmd_max_idx]);
                rsp_max_times.push_back(timestamps[rsp_max_idx]);
            }

            // Calculate average phase delay
            double phase_delay = 0.0;
            for (int i = 1; i < cmd_max_times.size(); i++) {  // Skip first cycle
                phase_delay += rsp_max_times[i] - cmd_max_times[i];
            }
            phase_delay /= (cmd_max_times.size() - 1);

            double avg_error = error_sum / count;

            // Store the results
            response.phase_delay = phase_delay;
            response.amplitude_loss = amplitude_loss;
            response.tracking_error = avg_error;

            // Store the results
            if (axis == 0) {
                results.sawtooth_responses_pitch.push_back(response);
            } else {
                results.sawtooth_responses_yaw.push_back(response);
            }

            std::cout << "  Phase delay: " << response.phase_delay << " ms" << std::endl;
            std::cout << "  Amplitude loss: " << response.amplitude_loss << "%" << std::endl;
            std::cout << "  Average tracking error: " << response.tracking_error << "°" << std::endl;

            // Reset position
            resetToInitialPosition();
        }
    }
}

void ControlOptimizer::testAll() {
    std::cout << "\n========== CONTROL RESPONSE TEST SUITE ==========\n" << std::endl;

    // Record initial position at the beginning
    recordInitialPosition();

    // Run all tests sequentially
    testDeadZone();
    testPointResponse();
    testSineResponse();
    testSawtoothResponse();

    // Final reset to initial position
    resetToInitialPosition();

    std::cout << "\n========== TEST SUITE COMPLETED ==========\n" << std::endl;

    // Summary output
    std::cout << "SUMMARY:" << std::endl;
    std::cout << "Dead zone (pitch): " << results.deadzone_pitch << "°" << std::endl;
    std::cout << "Dead zone (yaw): " << results.deadzone_yaw << "°" << std::endl;

    // Could add more summary statistics here if desired
}

void ControlOptimizer::setTestAmplitudes(const std::vector<double>& amplitudes) {
    testAmplitudes = amplitudes;
    std::cout << "Test amplitudes updated: ";
    for (size_t i = 0; i < testAmplitudes.size(); i++) {
        std::cout << testAmplitudes[i];
        if (i < testAmplitudes.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

void ControlOptimizer::setSineFrequencies(const std::vector<double>& frequencies) {
    sineFrequencies = frequencies;
    std::cout << "Test frequencies updated: ";
    for (size_t i = 0; i < sineFrequencies.size(); i++) {
        std::cout << sineFrequencies[i];
        if (i < sineFrequencies.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

const ControlOptimizer::TestResult& ControlOptimizer::getResults() const {
    return results;
}