#include "QLearner.hpp"
#include <errno.h>
class QLearningSimulate
{
   QLearner agent;
   std::string qtablePath;
   std::string policyPath;
   double myTime;
   double timeStep;
public:
   QLearningSimulate();
   QLearningSimulate(std::string qtablePath, std::string policyPath);
   bool initialize();
   double* simulateStateData(int type);
   int randomLimit(unsigned int min, unsigned int max);
   bool flipCoin (double p);
   FeetState determineState(double lfrontL, double lfrontR, double rfrontL, double rfrontR, double lbackL, double lbackR, double rbackL, double rbackR);
   int run();
};
