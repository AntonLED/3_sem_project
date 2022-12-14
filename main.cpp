#include "swarm.h"


int main() {
    PSO pso; 

    pso.init(
        1, 2, 3,
        {{-1, 2}, {-2, 3}},
        10
    );

    

    return 0; 
}