#ifndef _QLEARNER_
#define _QLEARNER_

#include "core.hpp"
#include "log.hpp"
#include <float.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <sstream>

/*
 * Agent that uses Q-learning with ...
*/
class QLearner: public Agent
{
private:

   std::vector<QTable> Q;

   std::vector<QTable> Policy;
   
   float epsilon; /* exploration prob */
   float alpha; /* learning rate */
   float gamma; /* discount rate */

   float tsprate; /* probability to try a particular TSP solution */

   double myTime; //TODO: temperary variable, time at which collision occurs
   int fallcount;
   unsigned int fallthreshold;

   bool hit; // true when external perturbation occurs. TODO: How to get this ??
   bool down; // true when robot falls on the ground. TODO: How to get this ??

   double *currentQ; // need to update 'Q-values' :)

   bool isStateSeen(const StateActionPair& s_a_pair, const State& state, 
			const Action& action) const;

   std::vector<Action> getTriedActions(const State& state) const;

   std::vector<Action> getLegalActions(const State& state, unsigned int type) const ;

   bool flipCoin (double p);

   int randomLimit(unsigned int min, unsigned int max) const;

   bool insertStateActionPair(const State& state, const Action& action);

   FeetState determineState(double lfrontL, double lfrontR, double rfrontL, double rfrontR, double lbackL, double lbackR, double rbackL, double rbackR);

   bool isStatePresent(std::vector<QTable>& q, const State& state) const;

   int getStateIndex(std::vector<QTable>& q, const State& state) const;

   std::vector<QTable> getCurrentPolicy() const;

   Action getBaseActionTSP() const;

   Action getAllActionTSP() const;  

   FeetState getFeetState(const int idx) const;

   PatternType getPattern(const int idx) const;

   /*Optimization Stuff*/

   /*
    * A naive approach but can work if you are lucky :).
   */
   PatternType getRandomPattern(unsigned int min_val, unsigned int max_val) const;

   void getTSP(Action& act1) const;

public:

   QLearner();

   QLearner(float epsilon, float alpha, 
            float gamma, float tsprate
            );

   void init(float epsilon, float alpha, float gamma, float tsprate, unsigned int fallthreshold, double myTime);

   int getReward();

   double getQValue(const State& state, const Action& action);

   bool updateQValue(const State& state, const Action& action, double qvalue);

   double getValue(const State& state);

   Action getAction(State& state);

   void doAction(Action& action);

   virtual Action getPolicy(const State& state);

   Action justPolicy(State& state);

   void update(State& state, Action& action, State& nextstate, int reward);

   virtual ~QLearner();

   virtual bool savePolicy(const std::string filename);

   bool loadPolicy(const std::string filename);
   
   void printCurrentPolicy();

   void printPersistentPolicy();

   bool loadQTable(const std::string filename);

   bool saveQTable(const std::string filename);

   void printQTable();

   bool createPersistence(const std::string qtablePath, const std::string policyPath);

   bool detectPerturbation(double myTime);

   bool detectFall(FeetState fstate);
 
   bool getHit();

   bool getFall();
   
};

#endif
