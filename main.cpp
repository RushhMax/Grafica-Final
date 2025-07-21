#include "camera.h"
#include "chessboard_detector.h"
#include "renderer.h"
#include "shared_data.h"

constexpr std::chrono::milliseconds FRAME_RATE = std::chrono::milliseconds(33); // ~60fps

int main() {
    try {
        SharedData shared;
        std::cout << shared.running << std::endl;

        Camera camara(0, shared);
        ChessboardDetector chessboard_detector(camara, shared); // se hace la calibración
        Renderer renderer(shared); // se inicializa toda la parte de gl + shaders

        std::jthread inputThread([&camara, &shared]() {
            while (shared.running) {
                camara.set_frame();
            }
            });

        std::jthread cvThread([&chessboard_detector, &shared]() {
            while (shared.running) {
                chessboard_detector.update();
                /** update de la detección de manos va aquí **/

                std::this_thread::sleep_for(FRAME_RATE);
            }
            });

        std::cout << "[main] ID: " << std::this_thread::get_id() << "\n";
        renderer.run();
    }
    catch (std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return -1;
    }

    //cvThread.join ya no porque se hace un jthread
    return 0;
}