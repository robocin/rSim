#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"

class VSSWorld
{
    public:
        VSSWorld();
        ~VSSWorld();
        void step(double dt);

    private:
        PWorld* physics;
};