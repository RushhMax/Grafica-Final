#include "camera.h"
#include "chessboard_detector.h"
#include "renderer.h"
#include "shared_data.h"

int main() {
    try {
        SharedData shared;
        std::cout << shared.running << std::endl;

        Camera camara(0, shared);
        ChessboardDetector chessboard_detector(camara, shared); // se hace la calibración
        Renderer renderer(shared); // se inicializa toda la parte de gl + shaders

        std::jthread cvThread([&shared, &camara]() {
            while (shared.running) {
                cv::Mat temp_frame;
                if (!camara.get_frame(temp_frame)) continue;

                std::unique_lock lock(shared.mut);
                shared.cv.wait(lock, [&shared] {
                    return shared.processed || !shared.running;
                    });

                if (!shared.running) break;

                int write_idx = 1 - shared.read_idx;
                temp_frame.copyTo(shared.frames[write_idx]);
                shared.frame_ready = true;
                shared.processed = false;

                lock.unlock();
                shared.cv.notify_one();

                std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~60fps
            }
            });

        std::cout << "[main] ID: " << std::this_thread::get_id() << "\n";
        renderer.run();

        {
            std::lock_guard lock(shared.mut);
            shared.running = false;
            shared.cv.notify_all();
        }
    }
    catch (std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return -1;
    }

    //cvThread.join ya no porque se hace un jthread
    return 0;
}