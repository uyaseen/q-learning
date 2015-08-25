#ifndef _RLCOREH_
#define _RLCOREH_

#include <vector>
#include <string>

/*
 * Standing, Right foot on ground, Left foot on ground, Fallen on ground
*/
//enum FeetState{BOTH_ON_GROUND, RIGHT_ON_GROUND, LEFT_ON_GROUND, BOTH_NOT_ON_GROUND};
/*
 * Now we have 16 states based on the values of 8 fsrs on each foot i.e 2 states for each foot i.e front back e.g: value 
 * of front and back  fsrs for each foot, 2 exp 4 = 16 states.
 *    left	 right
 * +--------+ +--------+
 * |0.0  0.0| |0.0  0.0|  front
 * |        | |        |
 * |0.0  0.0| |0.0  0.0|  back
*/
enum FeetState{ZERO_FSRS, R_BACK, L_BACK, L_R_BACK, R_FRONT, R_FRONT_BACK, L_BACK_R_FRONT, L_R_BACK_R_FRONT, L_FRONT, L_FRONT_R_BACK, L_FRONT_BACK, L_R_BACK_L_FRONT, L_R_FRONT, L_R_FRONT_R_BACK, L_R_FRONT_L_BACK, ALL_FSRS};

/*
 * Available Patterns
*/
enum PatternType{ PLATEAU , QUIESCENT , AMORTI , OSCILLATORY, SLOWOSCILLATION , FASTOSCILLATION};

/*
 * sigma_s, sigma_f values for the available patterns. //TODO: Move the '#define' values to a seperate header file ('rl_define.h').
*/
#define Plateau_sigma_s = 0;
#define Plateau_sigma_f = 2.8;
#define Quiescent_sigma_s = 0.5;
#define Quiescent_sigma_f = 0.5;
#define Amorti_sigma_s = 4.5;
#define Amorti_sigma_f = 0.6;
#define Oscillatory_sigma_s = 4.6;
#define Oscillatory_sigma_f = 1.5;
#define Oscillatory_slow_sigma_s = 0;
#define Oscillatory_slow_sigma_f = 0;
#define Oscillatory_fast_sigma_s = 5.6;
#define Oscillatory_fast_sigma_f = 1.15;

/*
 * Struct to represent base sigma_s, sigma_f of rs_neuron
*/
struct rs_neuron_b
{
   PatternType pattern;
};

/*
 * Struct to be used by 'State' etc, as one state would have 24 values i.e one for each joint/CPG
*/

struct rs_neuron
{
   rs_neuron_b rsneuron[24];
};

/*
 * Interface for a state.
*/
class State
{
   public:
   FeetState feet_state;
   // TODO: Point of impact
   // TODO: injected_current ...

   /*
    * A Simple method to comapare feet state.
   */
   bool compareFeetState(const FeetState f1) const
   {
      if(this->feet_state == f1)
         return true;
      return false;
   }

   std::string getName()
   {
      std::string state = "";
      switch(feet_state)
      {
         case ZERO_FSRS:
            state += "Zero FSRS -- ";
            break;
         case R_BACK:
            state += "Right Back -- ";
            break;
         case L_BACK:
            state += "Left Back -- ";
            break;
         case L_R_BACK:
            state += "Left & Right Back -- ";
            break;
         case R_FRONT:
            state += "Right Front -- ";
            break;
         case R_FRONT_BACK:
            state += "Right Front & Back -- ";
            break;
         case L_BACK_R_FRONT:
            state += "Left Back & Right Front -- ";
            break;
         case L_R_BACK_R_FRONT:
            state += "Left & Right Back, Right Front -- ";
            break;
         case L_FRONT:
            state += "Left Front -- ";
            break;
         case L_FRONT_R_BACK:
            state += "Left Front & Right Front -- ";
            break;
         case L_FRONT_BACK:
            state += "Left Front & Back -- ";
            break;
         case L_R_BACK_L_FRONT:
            state += "Left & Right Back, Left Front -- ";
            break;
         case L_R_FRONT:
            state += "Left & Right Front -- ";
            break;
         case L_R_FRONT_R_BACK:
            state += "Left & Right Front, Right Back -- ";
            break;
         case L_R_FRONT_L_BACK:
            state += "Left & Right Front, Left Back -- ";
            break;
         case ALL_FSRS:
            state += "All FSRS -- ";
            break;
         default:
            state += "Unknown/Invalid State";
            break;
      }
      state += "\n";
      return state;
   }

   /*
    * Overloaded needed because of 'this' thing.
   */
   static std::string getName(FeetState fstate)
   {
      std::string state = "";
      switch(fstate)
      {
         case ZERO_FSRS:
            state += "Zero FSRS -- ";
            break;
         case R_BACK:
            state += "Right Back -- ";
            break;
         case L_BACK:
            state += "Left Back -- ";
            break;
         case L_R_BACK:
            state += "Left & Right Back -- ";
            break;
         case R_FRONT:
            state += "Right Front -- ";
            break;
         case R_FRONT_BACK:
            state += "Right Front & Back -- ";
            break;
         case L_BACK_R_FRONT:
            state += "Left Back & Right Front -- ";
            break;
         case L_R_BACK_R_FRONT:
            state += "Left & Right Back, Right Front -- ";
            break;
         case L_FRONT:
            state += "Left Front -- ";
            break;
         case L_FRONT_R_BACK:
            state += "Left Front & Right Front -- ";
            break;
         case L_FRONT_BACK:
            state += "Left Front & Back -- ";
            break;
         case L_R_BACK_L_FRONT:
            state += "Left & Right Back, Left Front -- ";
            break;
         case L_R_FRONT:
            state += "Left & Right Front -- ";
            break;
         case L_R_FRONT_R_BACK:
            state += "Left & Right Front, Right Back -- ";
            break;
         case L_R_FRONT_L_BACK:
            state += "Left & Right Front, Left Back -- ";
            break;
         case ALL_FSRS:
            state += "All FSRS -- ";
            break;
         default:
            state += "Unknown/Invalid State -- ";
            state += fstate;
            state += " ";
            break;
      }
      state += "\n";
      return state;
   }

};

/*
 * Interface for an Action.
*/
class Action
{
   public:
   rs_neuron rs_neuron_pattern;

   /*
    * A simple method to compare Actions i.e rs_neuron values //TODO: Also incorporate pfneuron only if applicable.
   */
   bool compareActions(const Action& action) const
   {
      for(int i = 0; i < 24; i++)
      {
         if(this->rs_neuron_pattern.rsneuron[i].pattern == action.rs_neuron_pattern.rsneuron[i].pattern)
            continue;
         else
            return false;
      }
      return true;
   }

   /*
    * To make sure that valid action is generated, invalid actions also got generated sometimes due to 'unknown' reasons.
   */
   bool isValid()
   {
      bool valid = true;
      for(int i = 0; i < 24; i++)
      {
         switch(rs_neuron_pattern.rsneuron[i].pattern)
         {
            case PLATEAU:
               break;
            case QUIESCENT:
               break;
            case AMORTI:
               break;
            case OSCILLATORY:
               break;
            case SLOWOSCILLATION:
               break;
            case FASTOSCILLATION:
               break;
	    default:
               valid = false;
               break;
         }
      }
      return valid;
   }

   std::string getName()
   {
      std::string action = "";
      for(int i = 0; i < 24; i++)
      {
         switch(rs_neuron_pattern.rsneuron[i].pattern)
         {
            case PLATEAU:
               action += "Plateau ";
               break;
            case QUIESCENT:
               action += "Quiescent ";
               break;
            case AMORTI:
               action += "Amorti ";
               break;
            case OSCILLATORY:
               action += "Oscillatory ";
               break;
            case SLOWOSCILLATION:
               action += "Slow Oscillation ";
               break;
            case FASTOSCILLATION:
               action += "Fast Oscillation ";
               break;
	    default:
               action += "Unknown/Invalid Action -- " ;
               action +=  rs_neuron_pattern.rsneuron[i].pattern;
               action += " ";
               break;
         }
      }
      action += "\n";
      return action;
   }
};

class StateActionPair
{
public:
   State state;
   Action action;
   StateActionPair() {}
   StateActionPair(State state, Action) {this->state = state; this->action = action; } 
   StateActionPair(State& state, Action& action) {this->state = state, this->action = action; }
};

class QTable
{
public:
   StateActionPair state_action_pair;
   double qvalue;
};

/*
 * Interface for an agent.
*/
class Agent
{
public:

   virtual Action getPolicy(const State& state) = 0;

   virtual bool savePolicy(const std::string filename) {return true;};

   virtual ~Agent() {};

};


#endif
