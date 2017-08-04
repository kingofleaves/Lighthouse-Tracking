/**
 * Euler angle class
 *
 * @author Hayato Ikoma <hikoma@stanford.edu>
 * @copyright The Board of Trustees of the Leland
   Stanford Junior University
 * @version 2017/03/28
 */
#ifndef EULER_H
#define EULER_H

#include "Arduino.h"

class Euler {
public:

  double pitch, yaw, roll;

  Euler(double _pitch, double _yaw, double _roll)
    : pitch(_pitch), yaw(_yaw), roll(_roll) {}


  Euler()
    : pitch(0), yaw(0), roll(0) {}
};

#endif // ifndef EULER_H
