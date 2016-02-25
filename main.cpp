#include "src/QLearningSimulate.hpp" 

int main(int argc, char** argv)
{

   std::string qtablePath = "persistent_storage/qtable.uy";
   std::string policyPath = "persistent_storage/policy.uy";
   QLearningSimulate simulate(qtablePath, policyPath);
   
   simulate.run();

   return 0;
}

