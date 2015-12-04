#ifndef __BEST_FIRST_SEARCH_HPP__
#define __BEST_FIRST_SEARCH_HPP__

#include "IW1Search.hpp"

class BestFirstSearch : public IW2 {
public:
    BestFirstSearch(RomSettings *, Settings &settings, ActionVect &actions, StellaEnvironment* _env);
    TreeNode* choose_node();    
};



#endif 
