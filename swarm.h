#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <fstream>
#include <vector>



class PSO final {
private:
    double uniform01(); 
    struct pso_params { double w, phi_p, phi_g; };
    struct pso_state {
        unsigned curNumIterations;
        std::vector<double> globalPos;
        double globalVal; 
        std::vector<double> minBounds; 
        std::vector<double> maxBounds; 
        pso_params parameters; 
        std::vector<std::vector<double>> swarmPoses; 
        std::vector<std::vector<double>> swarmVelos;
        std::vector<std::vector<double>> swarmBestPoses; 
        std::vector<double> swarmBestVal; 
        unsigned swarmSize; 
        unsigned dimention; 
    };
    void makeStep(pso_state &curSwarmState); 

public:
    PSO(); 
    ~PSO(); 
    void init(
        double w,
        double phi_p, 
        double phi_g, 
        const std::vector<double> &min,
        const std::vector<double> &max, 
        unsigned swarmSize
    ); 
    void run(
        const std::function<double(const std::vector<double>&)> &targFunc, 
        unsigned numOfIterations
    ); 
    bool dump(
        std::string dumpfileName
    ); 
    bool dumpResult(
        std::string dumpfileName
    );
}; 


