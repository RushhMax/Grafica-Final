#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

// void saltar();

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "No se pudo abrir la cam." << endl;
        return -1;
    }

    Mat frame, hsv, mask, result;
    int estadoMano = 0;
    int contadorCierre = 0;
    int contadorApertura = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        flip(frame, frame, 1);
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        Scalar lower(0, 30, 60);
        Scalar upper(20, 150, 255);//color
        inRange(hsv, lower, upper, mask);

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

            drawContours(frame, contornos, max_index, Scalar(0, 255, 0), 2);
            vector<int> hullIndices;
            convexHull(contornoMano, hullIndices, false, false);

            vector<Vec4i> defects;
            if (hullIndices.size() > 3) {
                convexityDefects(contornoMano, hullIndices, defects);

                int defectCount = 0;
                for (size_t i = 0; i < defects.size(); ++i) {
                    if (defects[i][3] > 10000)
                        defectCount++;
                }
                if (defectCount >= 3) {//umbarl
                    contadorApertura++;
                    contadorCierre = 0;
                    if (contadorApertura > 5 && estadoMano != 1) {
                        estadoMano = 1; //abierta
                        cout << "Mano abierta detectada." << endl;
                    }
                }
                else {
                    contadorCierre++;
                    contadorApertura = 0;
                    if (contadorCierre > 5 && estadoMano == 1) {
                        estadoMano = 2; //cerrada
                        cout << "Mano cerrada detectada." << endl;

                        cout << "Salto ejecutado" << endl;//prueba

                        // saltar() la parte del salto si es que estuviese abierta y cerrada
                    }
                }
            }
        }
        imshow("Detección de mano", frame);
        imshow("Mascara piel", mask);
        if (waitKey(10) == 27) break; 
    }
    cap.release();
    destroyAllWindows();
    return 0;
}
/*
void saltar() {
    // aquí va la logica del salto del soldado de star wras
}
*/
