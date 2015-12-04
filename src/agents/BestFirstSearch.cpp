#include "IW2.hpp"
#include "BestFirstSearch.hpp"
#include "SearchAgent.hpp"
#include <list>
#include <cstdlib>
BestFirstSearch::BestFirstSearch(RomSettings *rom_settings, Settings &settings,
			       ActionVect &actions, StellaEnvironment* _env) 
    : IW2( rom_settings, settings, actions, _env){
}
TreeNode *BestFirstSearch::choose_node(){
    bool decide =  (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) >0.5);
    TreeNode *node = NULL;
    if( q_exploration->empty() &&  q_exploitation->empty())
        return NULL;
    else if (q_exploration->empty()){
        node =  q_exploitation->top();
        q_exploitation->pop();
    }
    else if (q_exploitation->empty()){
        node =  q_exploration->top();
        q_exploration->pop();
    }
    else{
        //std::cout << p;
        if (decide){
            node = q_exploitation->top();
            q_exploitation->pop();       
            //std::cout<<"exploit"; 
        }
        else{
            node =  q_exploration->top();
            q_exploration->pop();
            //std::cout<<"explore";
        }
    }
	return node;
}

