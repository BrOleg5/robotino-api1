// Saves ten camera images: JPEG as .jpg, raw RGB as binary .ppm.
// Usage: camera_dump [host[:port]]
#include <atomic>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "common.hpp"

int main(int argc, char** argv) {
    robotino_api1::Client robot(options_from_args(argc, argv));
    std::atomic<int> saved{0};
    robot.on_image([&](const robotino_api1::Image& image) {
        const int n = saved;
        if (n >= 10) {
            return;
        }
        const bool jpeg = image.type == robotino_api1::protocol::ImageType::JPEG;
        const std::string name = "image" + std::to_string(n) + (jpeg ? ".jpg" : ".ppm");
        std::ofstream out(name, std::ios::binary);
        if (!jpeg) {
            out << "P6\n" << image.width << " " << image.height << "\n255\n";
        }
        out.write(reinterpret_cast<const char*>(image.data.data()), static_cast<std::streamsize>(image.data.size()));
        std::printf("saved %s (%ux%u, %zu bytes)\n", name.c_str(), image.width, image.height, image.data.size());
        ++saved;
    });
    if (robotino_api1::Error e = robot.set_camera_streaming(true)) {
        std::fprintf(stderr, "camera: %s\n", e.message.c_str());
        return 1;
    }
    if (robotino_api1::Error e = robot.connect()) {
        std::fprintf(stderr, "connect: %s\n", e.message.c_str());
        return 1;
    }
    for (int i = 0; i < 100 && saved < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (saved == 0) {
        std::fprintf(stderr, "no images received (is UDP port %u reachable?)\n", robot.options().image_port);
        return 1;
    }
    return 0;
}
