#include "sslconfig.h"

void SSLConfig::Field::setFieldLimits(){
    this->xMax = this->fieldLength+this->fieldMargin;
    this->yMax = this->fieldWidth+this->fieldMargin;
    this->xMin = -this->xMax;
    this->yMin = -this->yMax;
}

void SSLConfig::Field::setFieldLimits(double xMin, double xMax, double yMin, double yMax){
    this->xMin = xMin;
    this->xMax = xMax;
    this->yMin = yMin;
    this->yMax = yMax;
}

