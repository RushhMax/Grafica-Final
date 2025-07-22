#include "hand_detector.h"
#include <iostream>
using namespace cv;
using namespace std;

HandDetector::HandDetector(SharedData& s) : shared(s) {}

void HandDetector::update() {
    if (shared.opencv_frames.empty()) return;

    cv::Mat frame = shared.opencv_frames.front();
    std::cout << "[hand_detector] yum\n";
    Mat hsv, mask;
    flip(frame, frame, 1);
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 30, 60), Scalar(20, 150, 255), mask);

    erode(mask, mask, Mat(), Point(-1, -1), 2);
    dilate(mask, mask, Mat(), Point(-1, -1), 2);

    vector<vector<Point>> contornos;
    findContours(mask, contornos, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contornos.empty()) {
        size_t max_index = 0;
        double max_area = 0;

        for (size_t i = 0; i < contornos.size(); ++i) {
            double area = contourArea(contornos[i]);
            if (area > max_area) {
                max_area = area;
                max_index = i;
            }
        }

        vector<Point> contornoMano = contornos[max_index];
        vector<int> hullIndices;
        convexHull(contornoMano, hullIndices, false, false);

        vector<Vec4i> defects;
        if (hullIndices.size() > 3)
            convexityDefects(contornoMano, hullIndices, defects);

        int defectCount = 0;
        for (const auto& d : defects)
            if (d[3] > 10000)
                defectCount++;

        if (defectCount >= 3) {
            contadorApertura++;
            contadorCierre = 0;
            if (contadorApertura > 5 && estadoMano != 1) {
                estadoMano = 1;
                cout << "[HandDetector] Mano abierta detectada." << endl;
            }
        } else {
            contadorCierre++;
            contadorApertura = 0;
            if (contadorCierre > 5 && estadoMano == 1) {
                estadoMano = 2;
                cout << "Mano cerrada detectada, salto" << endl;

                //salto
                // saltar(); 
            }
        }
    }
    //debug
    imshow("Mascara piel", mask);
    imshow("Detección de Mano", frame);
}