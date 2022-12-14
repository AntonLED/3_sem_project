#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <fstream>
#include <vector>



class PSO final {
private:
    double uniformAB(double A, double B); 
    struct pso_params { double w, phi_p, phi_g; };
    struct pso_state {
        unsigned curNumIterations;
        std::vector<double> globalPos;
        double globalBestVal; 
        std::vector<std::pair<double, double>> bounds; 
        pso_params swarmParameters; 
        std::vector<std::vector<double>> swarmPoses; 
        std::vector<std::vector<double>> swarmVelos;
        std::vector<std::vector<double>> swarmBestPoses; 
        std::vector<double> swarmBestVals; 
        unsigned swarmSize; 
        unsigned dimention; 
    };
    void makeStep(pso_state &curSwarmState); 

    pso_params swarmParams; 
    pso_state swarmState;

public:
    PSO(); 
    ~PSO(); 
    void init(
        double w,
        double phi_p, 
        double phi_g, 
        std::vector<std::pair<double, double>> bounds,
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
    void visualize(); 
}; 


