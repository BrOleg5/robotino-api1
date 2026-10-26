// Hardware check of the IO connector (step 6 of docs/hardware-testing.md). Walks through the
// digital outputs, relays, digital and analog inputs: it switches the outputs one at a time and
// asks what the multimeter shows, shows live readings while you apply a signal to an input, and
// prints PASS, FAIL or SKIP for each check. The power output and the encoder input are on
// separate connectors and are not checked.
// Usage: check_io [host[:port]] [dout] [relay] [din] [ain]
// Without section names all sections run. At a question answer y (yes), n (no), s (skip) or
// q (quit); while live readings are shown, Enter goes on.
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common.hpp"

namespace {

using robotino_api1::Client;
using robotino_api1::Milliseconds;
using robotino_api1::SensorState;
using Clock = std::chrono::steady_clock;

constexpr double kAnalogTolerance = 0.2;  // V
constexpr int kAnalogSamples = 20;

struct Quit {};

int passed = 0;
int failed = 0;
int skipped = 0;

void report(std::optional<bool> ok, const std::string& what) {
    const char* result = !ok ? "SKIP" : *ok ? "PASS" : "FAIL";
    std::printf("  %s  %s\n", result, what.c_str());
    (!ok ? skipped : *ok ? passed : failed) += 1;
}

// Lines typed on the console, read by a separate thread so that live readings keep updating
// while the program waits for Enter.
std::mutex input_mutex;
std::condition_variable input_cv;
std::deque<std::string> input_lines;
bool input_closed = false;

void read_input() {
    std::string line;
    while (std::getline(std::cin, line)) {
        std::lock_guard<std::mutex> lock(input_mutex);
        input_lines.push_back(line);
        input_cv.notify_one();
    }
    std::lock_guard<std::mutex> lock(input_mutex);
    input_closed = true;
    input_cv.notify_one();
}

// The next typed line, or nothing if none came within the timeout. The end of input quits.
std::optional<std::string> next_line(Milliseconds timeout) {
    std::unique_lock<std::mutex> lock(input_mutex);
    input_cv.wait_for(lock, timeout, [] { return !input_lines.empty() || input_closed; });
    if (input_lines.empty()) {
        if (input_closed) {
            throw Quit{};
        }
        return std::nullopt;
    }
    std::string line = input_lines.front();
    input_lines.pop_front();
    if (line == "q") {
        throw Quit{};
    }
    return line;
}

std::string read_line() {
    std::optional<std::string> line;
    while (!line) {
        line = next_line(Milliseconds(1000));
    }
    return *line;
}

// Asks a yes/no question: true for y, false for n, nothing for s.
std::optional<bool> ask(const char* question) {
    for (;;) {
        std::printf("%s [y/n/s/q] ", question);
        std::fflush(stdout);
        const std::string answer = read_line();
        if (answer == "y") {
            return true;
        }
        if (answer == "n") {
            return false;
        }
        if (answer == "s") {
            return std::nullopt;
        }
    }
}

// Shows a line of readings, updated in place, until Enter is pressed (returns false) or `done`
// returns true for a reading (returns true).
bool live(Client& robot, const std::function<std::string(const SensorState&)>& show,
          const std::function<bool(const SensorState&)>& done = nullptr) {
    std::size_t width = 0;
    for (;;) {
        robot.wait_for_update(Milliseconds(200));
        const SensorState s = robot.sensors();
        std::string text = show(s);
        const std::size_t length = text.size();
        text.resize(std::max(length, width), ' ');  // clear the rest of the previous line
        width = length;
        std::printf("\r  %s", text.c_str());
        std::fflush(stdout);
        if (done && done(s)) {
            std::printf("\n");
            return true;
        }
        if (next_line(Milliseconds(100))) {
            return false;  // Enter already moved to a new line
        }
    }
}

std::string bits(const bool* values, std::size_t count) {
    std::string text;
    for (std::size_t i = 0; i < count; ++i) {
        text += values[i] ? '1' : '0';
    }
    return text;
}

std::string format(const char* fmt, double a, double b = 0.0) {
    char buffer[128];
    std::snprintf(buffer, sizeof buffer, fmt, a, b);
    return buffer;
}

void check_digital_outputs(Client& robot) {
    std::printf("\nDigital outputs: each one is switched on alone; measure it against GND.\n");
    for (std::size_t i = 0; i < robotino_api1::kNumDigitalOutputs; ++i) {
        robot.set_digital_output(i, true);
        std::printf("digital output %zu on\n", i);
        report(ask("  Is this output at 24 V and all others at 0 V?"), "digital output " + std::to_string(i));
        robot.set_digital_output(i, false);
    }
}

void check_relays(Client& robot) {
    std::printf("\nRelays: each one is closed alone; check its contacts for continuity.\n");
    for (std::size_t i = 0; i < robotino_api1::kNumRelays; ++i) {
        robot.set_relay(i, true);
        std::printf("relay %zu closed\n", i);
        report(ask("  Is this relay closed (and the other one open)?"), "relay " + std::to_string(i) + " closes");
        robot.set_relay(i, false);
        std::printf("relay %zu open\n", i);
        report(ask("  Is it open now?"), "relay " + std::to_string(i) + " opens");
    }
}

void check_digital_inputs(Client& robot) {
    std::printf("\nDigital inputs: connect each one to 24 V (a digital output or the 24 V pin) and\n"
                "disconnect it again. Enter skips an input.\n");
    for (std::size_t i = 0; i < robotino_api1::kNumDigitalInputs; ++i) {
        std::printf("digital input %zu: connect it to 24 V\n", i);
        const SensorState before = robot.sensors();
        bool others_changed = false;
        const auto show = [&](const SensorState& s) {
            for (std::size_t j = 0; j < robotino_api1::kNumDigitalInputs; ++j) {
                others_changed = others_changed || (j != i && s.digital_input[j] != before.digital_input[j]);
            }
            return "inputs 0-7: " + bits(s.digital_input.data(), s.digital_input.size());
        };
        if (!live(robot, show, [&](const SensorState& s) { return s.digital_input[i]; })) {
            report(std::nullopt, "digital input " + std::to_string(i));
            continue;
        }
        std::printf("  now disconnect it\n");
        if (!live(robot, show, [&](const SensorState& s) { return !s.digital_input[i]; })) {
            report(std::nullopt, "digital input " + std::to_string(i));
            continue;
        }
        if (others_changed) {
            std::printf("  other inputs changed as well\n");
        }
        report(!others_changed, "digital input " + std::to_string(i));
    }
}

void check_analog_inputs(Client& robot) {
    std::printf("\nAnalog inputs: apply a voltage (0-10 V) to each input, press Enter and type the\n"
                "voltage the multimeter shows. An empty answer skips the input.\n");
    for (std::size_t i = 0; i < robotino_api1::kNumAnalogInputs; ++i) {
        std::printf("analog input %zu\n", i);
        live(robot, [&](const SensorState& s) {
            std::string text = format("input %.0f: %5.2f V | all:", static_cast<double>(i), s.analog_input[i]);
            for (double v : s.analog_input) {
                text += format(" %.2f", v);
            }
            return text;
        });
        std::printf("  multimeter, V: ");
        std::fflush(stdout);
        const std::string answer = read_line();
        char* end = nullptr;
        const double expected = std::strtod(answer.c_str(), &end);
        if (answer.empty() || end == answer.c_str()) {
            report(std::nullopt, "analog input " + std::to_string(i));
            continue;
        }
        double measured = 0.0;
        for (int n = 0; n < kAnalogSamples; ++n) {
            robot.wait_for_update(Milliseconds(200));
            measured += robot.sensors().analog_input[i];
        }
        measured /= kAnalogSamples;
        std::printf("  measured %.2f V, multimeter %.2f V\n", measured, expected);
        report(std::abs(measured - expected) < kAnalogTolerance, "analog input " + std::to_string(i));
    }
}

struct Section {
    const char* name;
    void (*run)(Client&);
};

const Section kSections[] = {
    {"dout", check_digital_outputs},
    {"relay", check_relays},
    {"din", check_digital_inputs},
    {"ain", check_analog_inputs},
};

}  // namespace

int main(int argc, char** argv) {
    std::vector<const Section*> sections;
    for (int a = 2; a < argc; ++a) {
        const Section* found = nullptr;
        for (const Section& s : kSections) {
            found = std::strcmp(argv[a], s.name) == 0 ? &s : found;
        }
        if (!found) {
            std::fprintf(stderr, "unknown section %s; use dout, relay, din or ain\n", argv[a]);
            return 2;
        }
        sections.push_back(found);
    }
    if (sections.empty()) {
        for (const Section& s : kSections) {
            sections.push_back(&s);
        }
    }

    Client robot(options_from_args(argc, argv));
    print_connection_events(robot);
    if (robotino_api1::Error e = robot.connect()) {
        std::fprintf(stderr, "connect: %s\n", e.message.c_str());
        return 1;
    }
    std::printf("%s\n", robot.info().text.c_str());
    std::printf("Answers: y yes, n no, s skip, q quit.\n");

    // Never joined: it may be blocked reading the console when the program ends.
    std::thread(read_input).detach();
    try {
        for (const Section* s : sections) {
            s->run(robot);
        }
    } catch (const Quit&) {
        std::printf("\nquit\n");
    }
    // All outputs off.
    robot.set_command(robotino_api1::Command{});
    robot.wait_for_update(Milliseconds(200));
    robot.disconnect();
    std::printf("%s: %d passed, %d failed, %d skipped\n", failed == 0 ? "OK" : "FAILED", passed, failed, skipped);
    // The input thread is still blocked reading the console and holds the lock of stdin, which
    // exit() waits for when it flushes the streams; end the process without that cleanup.
    std::fflush(stdout);
    std::_Exit(failed == 0 ? 0 : 1);
}
