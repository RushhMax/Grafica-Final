#include "camera.h"
#include "chessboard_detector.h"
#include "renderer.h"
#include "shared_data.h"
#include "hand_detector.h"

constexpr std::chrono::milliseconds FRAME_RATE = std::chrono::milliseconds(33); // ~60fps

int main() {
    try {
        SharedData shared;
        std::cout << shared.running << std::endl;

        Camera camara(0, shared);
        ChessboardDetector chessboard_detector(camara, shared); // se hace la calibracion
        Renderer renderer(shared); // se inicializa toda la parte de gl + shaders
        HandDetector hand_detector(shared);//agrega la deteccion de la mano

        std::jthread inputThread([&camara, &shared]() {
            while (shared.running) {
                camara.set_frame();
                // std::cout << "[SIZE] Good frame of " << shared.opencv_frames.front().cols << "x" << shared.opencv_frames.front().rows << "pixels\n";
            }
            });

        std::jthread cvThread([&chessboard_detector,&hand_detector, &shared]() {
            while (shared.running) {
                chessboard_detector.update();
                hand_detector.update();
                shared.opencv_frames.pop();
                
                std::this_thread::sleep_for(FRAME_RATE);
            }
            });

        renderer.run();
    }
    catch (std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return -1;
    }

    //cvThread.join ya no porque se hace un jthread
    return 0;
}