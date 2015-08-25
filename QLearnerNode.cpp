#include "QLearnerNode.hpp"

QLearnerNode::QLearnerNode() { }

QLearnerNode::QLearnerNode(std::string qtablePath, 
                           std::string policyPath):qtablePath(qtablePath), 
                           policyPath(policyPath)
{
   /*
    * QLearner(epsilon, alpha, gamma, tsprate, fallcount, myTime);
    * TODO: Make sure that the values of epsilon, gamma are optimal. @Ref: (Paper) epsilon = 0.3, alpha = 0.1
   */
   agent.init(0.05f, 0.8f, 0.2f, 0.7f, 50, 9.04);
}

bool QLearnerNode::initialize()
{
   if(agent.loadQTable(qtablePath) && agent.loadPolicy(policyPath))
      return true;
   return false;
}

bool QLearnerNode::initializer()
{
   if(initialize())
   {
      LOG("Q-Table : %s & Policy : %s loaded.\n", qtablePath.c_str(), policyPath.c_str());
   }
   else
   {
      ERROR("Error in Loading files %s ('q-table')/%s & ('policy')\nError Code (errno): %d\nCreating new file, Warning: The 'q-table' and 'policy' will be created from scratch ...\n", 
            qtablePath.c_str(), policyPath.c_str(), errno);
      if(agent.createPersistence(qtablePath, policyPath))
         LOG("Q-table File = '%s' and Policy File = '%s' created ...\n", qtablePath.c_str(), policyPath.c_str());
      else
      {
         LOG("Error in creating files %s ('q-table')/%s & ('policy')\n. Make Sure you have the permissions to store the files on your disk.\n",         qtablePath.c_str(), policyPath.c_str());
         return false;
      }
   }
   return true;
}

