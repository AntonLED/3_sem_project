#include "swarm.h"


double PSO::uniformAB(double A, double B) {
    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> distribution(A, B);
    return distribution(generator);
}

PSO::PSO() { }

PSO::~PSO() { }

void PSO::init(
    double w,
    double phi_p, 
    double phi_g, 
    const std::vector<std::pair<double, double>> bounds,
    unsigned swarmSize
) {
    // params
    PSO::swarmParams = { w, phi_p, phi_g }; 
    PSO::swarmState.swarmParameters = PSO::swarmParams; 
    // dims calc    
    unsigned dimensions = bounds.size(); 
    PSO::swarmState.dimention = dimensions; 
    // poss init 
    PSO::swarmState.swarmPoses.resize(swarmSize);
    for (int i = 0; i < swarmSize; ++i) {
        PSO::swarmState.swarmPoses[i].resize(dimensions); 
        for (int d = 0; d < dimensions; ++d)
            PSO::swarmState.swarmPoses[i][d] = uniformAB(bounds[d].first, bounds[d].second); 
    }
    // best_poss init
    PSO::swarmState.swarmBestPoses.resize(swarmSize);
    for (int i = 0; i < swarmSize; ++i) {
        PSO::swarmState.swarmBestPoses[i].resize(dimensions); 
        for (int d = 0; d < dimensions; ++d)
            PSO::swarmState.swarmBestPoses[i][d] = PSO::swarmState.swarmPoses[i][d]; 
    }
    // velsinit 
    PSO::swarmState.swarmVelos.resize(swarmSize);
    for (int i = 0; i < swarmSize; ++i) {
        PSO::swarmState.swarmVelos[i].resize(dimensions);
        for (int d = 0; d < dimensions; ++d) 
            PSO::swarmState.swarmVelos[i][d] = uniformAB(
                -1 * std::abs( bounds[d].first - bounds[d].second ),
                1 * std::abs( bounds[d].first - bounds[d].second )
            );
    }
    // swarm size init 
    PSO::swarmState.swarmSize = swarmSize; 

    PSO::swarmState.globalBestPos.resize(dimensions); 

    PSO::swarmState.swarmBestVals.resize(swarmSize); 

    PSO::swarmState.curNumIterations = 0; 

    double globalBestVal = HUGE_VAL; 
}

void PSO::makeStep(
    const std::function<double(const std::vector<double>&)> &targFunc,
    pso_state &curSwarmState
) {
    for (int i = 0; i < curSwarmState.swarmSize; ++i) {
        for (int d = 0; d < curSwarmState.dimention; ++d) {
            double r_p = uniformAB(0.0, 1.0); 
            double r_g = uniformAB(0.0, 1.0);
            swarmState.swarmVelos[i][d] = swarmState.swarmParameters.w * swarmState.swarmVelos[i][d] +
                swarmState.swarmParameters.phi_p * r_p * (swarmState.swarmBestPoses[i][d] - swarmState.swarmPoses[i][d]) + 
                swarmState.swarmParameters.phi_g * r_g * (swarmState.swarmBestPoses[i][d] - swarmState.swarmPoses[i][d]); 
            swarmState.swarmPoses[i][d] += swarmState.swarmVelos[i][d]; 
        }
        if (targFunc(swarmState.swarmPoses[i]) < targFunc(swarmState.swarmBestPoses[i])) {
            for (int d = 0; d < PSO::swarmState.dimention; ++d)
                 PSO::swarmState.swarmBestPoses[i][d] = PSO::swarmState.swarmPoses[i][d]; 
            if (targFunc(PSO::swarmState.swarmBestPoses[i]) < targFunc(PSO::swarmState.globalBestPos)) 
                for (int d = 0; d < PSO::swarmState.dimention; ++d)
                    PSO::swarmState.globalBestPos[d] = PSO::swarmState.swarmBestPoses[i][d];
        }
    }
    PSO::swarmState.curNumIterations++; 
}

 void PSO::run(
    const std::function<double(const std::vector<double>&)> &targFunc,
    unsigned numOfIterations
) {
    while (PSO::swarmState.curNumIterations < numOfIterations)
        PSO::makeStep(targFunc, PSO::swarmState); 
 }

 bool PSO::dumpResult(std::string s) {
    std::cout << s << '\n'; 
    for (int d = 0; d < PSO::swarmState.dimention; ++d)
        std::cout << PSO::swarmState.globalBestPos[d] << ' ';
    std::cout << '\n' ;
    return true; 
 }

double mccormick(const std::vector<double> & x) {
    auto a = x[0];
    auto b = x[1];
    return sin(a + b) + (a - b) * (a - b) + 1.0 + 2.5 * b - 1.5 * a;
}

double test(const std::vector<double> & x) {
    auto a = x[0];
    auto b = x[1];
    
    return (a-2)*(a-2) + (b-3)*(b-3);
}

double michalewicz(const std::vector<double> & x) {
    auto m = 10;
    auto d = x.size();
    auto sum = 0.0;
    for (int i = 1; i < d; ++i) {
        auto j = x[i - 1];
        auto k = sin(i * j * j / std::acos(-1.0));
        sum += sin(j) * pow(k, (2.0 * m));
    }
    return -sum;
}

int main() {
    PSO pso; 
    pso.init(
        0.1, 0.1, 0.1,
        {{-1.5, 4.0}, {-3.0, 4.0}},
        1000
    );
    pso.run(mccormick, 100); 
    pso.dumpResult("FUCK"); 
//
    pso.init(
        0.4, 0.5, 0.3,
        {{-4, 4}, {-4, 4}},
        100
    );
    pso.run(test, 100); 
    pso.dumpResult("FUCKKK"); 
//
    pso.init(
        0.3, 0.3, 0.3,
        {{0, std::acos(-1.0)}, {0, std::acos(-1.0)}},
        2000
    );
    pso.run(michalewicz, 300); 
    pso.dumpResult("FUCKKKKKK"); 

    return 0; 
}

