#ifndef __IW2__
#define __IW2__

#include "IW1Search.hpp"

class IW2 : public IW1Search {
public:
    IW2(RomSettings *, Settings &settings, ActionVect &actions, StellaEnvironment* _env);

	virtual ~IW2();
    virtual TreeNode* choose_node();
    void pushqueue(TreeNode* child);
    class TreeNodeComparerExploration
    {
    public:
	
	bool operator()( TreeNode* a, TreeNode* b ) const 
	{
		if ( b->novelty < a->novelty ) return true;
		else if( b->novelty == a->novelty && b->fn < a->fn ) return true;
		return false;
	}
    };


    class TreeNodeComparerExploitation
    {
    public:
	
	bool operator()( TreeNode* a, TreeNode* b ) const 
	{
	    if ( b->fn < a->fn ) return true;
	    else if( b->fn == a->fn && b->novelty < a->novelty ) return true;
	    return false;
	}
    };

    virtual int expand_node(TreeNode* child_node);
    void clear_queues(){
	    delete q_exploration;
	    delete q_exploitation;
	    q_exploration = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploration >;
	    q_exploitation = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploitation >;
    }

protected:	
    int calculate_novelty(TreeNode * child_node);
    void reset_branch(TreeNode* node);
    int  reuse_branch(TreeNode* node);
    unsigned size_branch(TreeNode* node);
    
    virtual void expand_tree(TreeNode* start);

    std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploration >* q_exploration;
    std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploitation >* q_exploitation;
    
    reward_t		m_max_reward;
    unsigned m_gen_count_novelty2;
    unsigned m_gen_count_novelty1;
    unsigned m_exp_count_novelty2;
    unsigned m_exp_count_novelty1;
};



#endif // __IW_DIJKSTRA_SEARCH_HPP__
