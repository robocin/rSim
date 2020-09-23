#include <ode/ode.h>

#define PI 3.14159265358979323846

dReal randn_notrig(dReal mu = 0.0, dReal sigma = 1.0);
dReal randn_trig(dReal mu = 0.0, dReal sigma = 1.0);
dReal rand0_1();
dReal fric(dReal f);