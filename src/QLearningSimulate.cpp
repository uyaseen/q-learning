#include "QLearningSimulate.hpp"

QLearningSimulate::QLearningSimulate() { }

QLearningSimulate::QLearningSimulate(std::string qtablePath, 
                                     std::string policyPath):qtablePath(qtablePath), 
                                     policyPath(policyPath)
{
   /*
    * QLearner(epsilon, alpha, gamma, tsprate, fallcount, myTime);
    * TODO: Make sure that the values of epsilon, gamma are optimal. @Ref: (Paper) epsilon = 0.3, alpha = 0.1
   */
   agent.init(0.05f, 0.8f, 0.2f, 0.7f, 50, 9.04);
   myTime = 8.5;
   timeStep = 0.5; /* To achieve randomness, make more realistic etc */
}

bool QLearningSimulate::initialize()
{
   if(agent.loadQTable(qtablePath) && agent.loadPolicy(policyPath))
      return true;
   return false;
}

/*
 * Simulate the required sensor values i.e 'double lfrontL, double lfrontR, double rfrontL, double rfrontR, double lbackL, double lbackR, double rbackL, double rbackR' needed by QLearner::determineState(...).
 * Type = 0 (0.0), 1 (random), 2 (half random[probability]), 3 (odd (fix), even (random)), 4 ('-1' to represent "robot fall"), ..)
*/

double*  QLearningSimulate::simulateStateData(int type)
{
   unsigned int sensorcount = 8;
   double* sensorvalues = new double[sensorcount];
   switch(type)
   {
      case 0:
        for(unsigned int i = 0; i < 8; i++)
           sensorvalues[i] = 0.0;
        break;
     case 1:{
        double probability = 0.5;
        for(unsigned int i = 0; i < 8; i++)
        {
           if(flipCoin(probability))
              sensorvalues[i] = (double) randomLimit(0,23);
           else
              sensorvalues[i] = 0.0;
        }
        break;}
     case 2:{
        double probability = 0.7;
        for(unsigned int i = 0; i < 8; i++)
        {
           /*
            * Generate values in b/w 0-1 with 0.7 probability.
           */
           if(flipCoin(probability))
              sensorvalues[i] = (double) (randomLimit(0,23)/23);
           else
              sensorvalues[i] = 2.0 + i;
        }
        break;}
     case 3:
        for(unsigned int i = 0; i < 8; i++)
        {
           if(i%2 == 0)
              sensorvalues[i] = (double) randomLimit(0,23);
           else
              sensorvalues[i] = i;
        }
        break;
     case 4:
        for(unsigned int i = 0; i < 8; i++)
           sensorvalues[i] = -1.0;
        break;
     default:
        for(unsigned int i = 0; i < 8; i++)
           sensorvalues[i] = 0.0;
        break;
   }

   return sensorvalues;

}

/*
 * Note: flipCoint(..), randomLimit(..) & determineFeetState(..) are already implemented in QLearner but they are re-implemented to make the design of
 * QLearner consistent and sensible.
*/

/*
 * Use for selecting an action with probability 'p'
 * TODO: Use better version of random generation, sth like 'std::uniform_real_distribution'
*/

bool QLearningSimulate::flipCoin(double p)
{
   double r = ((double) rand() / (RAND_MAX));
   return r < p;
}

/*
 * Generate random number in between the limit (boundry values included).
 * TODO: Use better version of random generation, sth like 'std::uniform_real_distribution'
*/

int QLearningSimulate::randomLimit(unsigned int min, unsigned int max)
{
   return (min + (rand() % (int)(max - min + 1)));
}

/*
 * This method takes the 8 FSRS of both feet to determine the state of the robot, which would be one state out of possible 
 * 'FeetState', see core.hpp for more details.
*/

FeetState QLearningSimulate::determineState(double lfrontL, double lfrontR, double rfrontL, double rfrontR, double lbackL, double lbackR, double rbackL, double rbackR)
{
   double lfront, rfront, lback, rback;
   lfront = lfrontL + lfrontR;
   rfront = rfrontL + rfrontR;
   lback  = lbackL  + lbackR;
   rback  = rbackL  + rbackR;
   FeetState fstate;
   if( (lfront < 1.0) && (rfront < 1.0) && (lback < 1.0) && (rback < 1.0) )
      fstate = ZERO_FSRS;
   else if( (lfront < 1.0) && (rfront < 1.0) && (lback < 1.0) && (rback > 1.0) )
      fstate = R_BACK;
   else if( (lfront < 1.0) && (rfront < 1.0) && (lback > 1.0) && (rback < 1.0) )
      fstate = L_BACK;
   else if( (lfront < 1.0) && (rfront < 1.0) && (lback > 1.0) && (rback > 1.0) )
      fstate = L_R_BACK;
   else if( (lfront < 1.0) && (rfront > 1.0) && (lback < 1.0) && (rback < 1.0) )
      fstate = R_FRONT;
   else if( (lfront < 1.0) && (rfront > 1.0) && (lback < 1.0) && (rback > 1.0) )
      fstate = R_FRONT_BACK;
   else if( (lfront < 1.0) && (rfront > 1.0) && (lback > 1.0) && (rback < 1.0) )
      fstate = L_BACK_R_FRONT;
   else if( (lfront < 1.0) && (rfront > 1.0) && (lback > 1.0) && (rback > 1.0) )
      fstate = L_R_BACK_R_FRONT;
   else if( (lfront > 1.0) && (rfront < 1.0) && (lback < 1.0) && (rback < 1.0) )
      fstate = L_FRONT;
   else if( (lfront > 1.0) && (rfront < 1.0) && (lback < 1.0) && (rback > 1.0) )
      fstate = L_FRONT_R_BACK;
   else if( (lfront > 1.0) && (rfront < 1.0) && (lback > 1.0) && (rback < 1.0) )
      fstate = L_FRONT_BACK;
   else if( (lfront > 1.0) && (rfront < 1.0) && (lback > 1.0) && (rback > 1.0) )
      fstate = L_R_BACK_L_FRONT;
   else if( (lfront > 1.0) && (rfront > 1.0) && (lback < 1.0) && (rback < 1.0) )
      fstate = L_R_FRONT;
   else if( (lfront > 1.0) && (rfront > 1.0) && (lback < 1.0) && (rback > 1.0) )
      fstate = L_R_FRONT_R_BACK;
   else if( (lfront > 1.0) && (rfront > 1.0) && (lback > 1.0) && (rback < 1.0) )
      fstate = L_R_FRONT_L_BACK;
   else if( (lfront > 1.0) && (rfront > 1.0) && (lback > 1.0) && (rback > 1.0) )
      fstate = ALL_FSRS;
   return fstate;
}

int QLearningSimulate::run()
{
   if(initialize())
   {
      agent.printQTable();
//      agent.printCurrentPolicy();
      int i = 0;
      srand (time(NULL)); /* Seed random numbers */
      while(i < 5)
      {
         /*
          * Step 0: Get the simulated 'feet' data.
          * Simulate data of type '2' with 0.7 probability as this is most closest to real data.
         */
         int type;
         if(flipCoin(0.7))
            type = 2;
         else
            type = randomLimit(0, 3);
         LOG("Simulate Type: %i\n", type);
         double* feetdata = simulateStateData(type);
         /*
          * Step 1: Get the state of the feet
          * TODO: The sequence doesn't matter for now but in reality mode, change this accordingly :)
         */
         FeetState fstate = determineState(feetdata[0], feetdata[1], feetdata[2], feetdata[3],
                        feetdata[4], feetdata[5], feetdata[6], feetdata[7]);
         State state;
         state.feet_state = fstate;
         LOG("fstate: %s\n", state.getName().c_str());
         State nstate; /* next state after applying the 'action' */
         /*
          * Step 2: Detect Perturbation
         */
         agent.detectPerturbation(myTime);
         if(agent.getHit())
         {
            /*
             * Step 3: Take an appropriate action, since perturbation has occured :(
            */
            Action action;
            #ifdef JUSTPOLICY
               action = agent.justPolicy(state);
            #else
               action = agent.getAction(state);
            #endif
            if(!action.isValid())
            {
               LOG("\t\t\tScrewed :'(\n");
               while(!action.isValid())
                  action = agent.getAction(state);
            }
            LOG("*\t*\t*\t*\t*\t*\t*\t*\t*\t*\n");
            LOG("*\t*\t*\t*\t*\t*\t*\t*\t*\t*\n");
            LOG("*\t*\t*\t*\t*\t*\t*\t*\t*\t*\n");
            LOG("Perturbation Occurs\n");
            
            LOG("Action after perturbation\n%s", action.getName().c_str());
            agent.doAction(action);
            /*
             * Keep on getting FeetState for quite some time to make sure that robot survived the collission or not.
            */
            int count = 100;
            while(count > 0)
            {
               /*
                * Get the state of the robot again.
               */
               feetdata = simulateStateData(2);
               fstate = determineState(feetdata[0], feetdata[1], feetdata[2], feetdata[3],
                        feetdata[4], feetdata[5], feetdata[6], feetdata[7]);
               /*
                * Detect if collision has occured.
               */
               agent.detectFall(fstate);
               count--;
            }
            
            /*
             * Step 4: Update the 'q-value' after the action, according to the reward.
             * call the update(..) and finish this episode.
            */
            agent.update(state, action, nstate, agent.getReward());
            
            /*
             * Step 5: Save the 'q-table' & 'policy' to persistent storage :)
            */
            if((agent.saveQTable(qtablePath)) && (agent.savePolicy(policyPath)))
               LOG("Saved 'q-table' [%s] & 'policy' [%s].\n \t\t\t* Cheers *\t\t\t\n", 
                    qtablePath.c_str(), policyPath.c_str());
            else
               LOG("Error in saving files %s ('q-table')/%s & ('policy')\n. Make Sure you have the permissions to store the files on your disk.\n", 
                   qtablePath.c_str(), policyPath.c_str());
            
            break;
         }
         myTime += timeStep;
         i++;
      }
      agent.printQTable();

   }
   else
   {
      ERROR("Error in Loading files %s ('q-table')/%s & ('policy')\nError Code (errno): %d\nCreating new file, Warning: The 'q-table' and 'policy' will be created from scratch ...\n", 
            qtablePath.c_str(), policyPath.c_str(), errno);
      if(agent.createPersistence(qtablePath, policyPath))
         LOG("Q-table File = '%s' and Policy File = '%s' created ...\n", qtablePath.c_str(), policyPath.c_str());
      else
         return -1;
      return 1;
   }
   return 1;
}


