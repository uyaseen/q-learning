#include "QLearner.hpp"
#include <errno.h>

class QLearnerNode
{
   std::string qtablePath;
   std::string policyPath;
public:
   QLearner agent;
   QLearnerNode();
   QLearnerNode(std::string qtablePath, std::string policyPath);
   
   bool initialize();
   bool initializer();
};
