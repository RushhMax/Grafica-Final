#pragma once

#include <opencv2/opencv.hpp>
#include "shared_data.h"

class HandDetector {
    SharedData& shared;
    int estadoMano = 0;
    int contadorCierre = 0;
    int contadorApertura = 0;

public:
    explicit HandDetector(SharedData& s);
    void update();
};
