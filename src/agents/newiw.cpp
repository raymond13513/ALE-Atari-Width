#include "newiw.hpp"
#include "SearchAgent.hpp"
#include <list>
#define MAXI 300000

Newiw::Newiw(RomSettings *rom_settings, Settings &settings,
			       ActionVect &actions, StellaEnvironment* _env) 
    : IW1Search( rom_settings, settings, actions, _env){
 
	m_max_reward = settings.getInt( "max_reward" );
    //create a new pq ranked by novelty
	q_exploration = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploration >;
    //create a new pg ranked by reward
	q_exploitation = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploitation >;
}

Newiw::~Newiw() {
		    delete q_exploration;
		    delete q_exploitation;	
}

//tree compare class that compare two nodes from a tree
//used for pq
class TreeNodeComparer
{
public:

	bool operator()( TreeNode* a, TreeNode* b ) const 
	{
        //fn is the award that we compare
		if ( b->fn < a->fn ) return true;
		return false;
	}
};

int Newiw::expand_node( TreeNode* curr_node )
{
    
	int num_simulated_steps =0;
    //number of availible actions from the state
	int num_actions = available_actions.size();
    //check if the current node is at the bottom of the tree
	bool leaf_node = (curr_node->v_children.empty());
    //increase the count of expanded node
	m_expanded_nodes++;
    //check if the currety ovelty is 1 or 2
	if(curr_node->novelty == 1)
	    m_exp_count_novelty1++;
	else
	    m_exp_count_novelty2++;
	// Expand all of its children (simulates the result)	
	if(leaf_node){ 
        //add num_actions of child to the current node
		curr_node->v_children.resize( num_actions );
		curr_node->available_actions = available_actions;
        //suffle the order of operations
		if(m_randomize_successor)
			std::random_shuffle ( curr_node->available_actions.begin(), curr_node->available_actions.end() );

	
	}
	
	for (int a = 0; a < num_actions; a++) {
		Action act = curr_node->available_actions[a];
		
		TreeNode * child;
		// If re-expanding an internal node, don't creates new nodes
		if (leaf_node) {
            //increase the count of generated node
			m_generated_nodes++;
			child = new TreeNode(	curr_node,	
						curr_node->state,
						this,
						act,
						sim_steps_per_node
						, discount_factor); 
	
			if ( check_novelty_1( child->state.getRAM() ) ) {
			    update_novelty_table( child->state.getRAM() );
			    child->novelty = 1;
			    m_gen_count_novelty1++;
			}
			else{
			    child->novelty = 2;
			    m_gen_count_novelty2++;
			}
			child->fn += ( m_max_reward - child->discounted_accumulated_reward ); // Miquel: add this to obtain Hector's BFS + m_max_reward * (720 - child->depth()) ;

			child->num_nodes_reusable = curr_node->num_nodes_reusable + num_actions;

			if (child->depth() > m_max_depth ) m_max_depth = child->depth();
			num_simulated_steps += child->num_simulated_steps;
					
			curr_node->v_children[a] = child;

		}
		else {
			
			child = curr_node->v_children[a];
			m_pruned_nodes++;
			// This recreates the novelty table (which gets resetted every time
			// we change the root of the search tree)
			if ( m_novelty_pruning ){
                int temp = MAXI - check_novelty_1( child->state.getRAM());
				if (temp>0 ){
					update_novelty_table( child->state.getRAM() );
					child->novelty = temp;
					m_gen_count_novelty1++;
				}
				else{
					child->novelty = MAXI;
					m_gen_count_novelty2++;
					
				}
			}
			
			child->updateTreeNode();
			child->fn += ( m_max_reward - child->discounted_accumulated_reward ); // Miquel: add this to obtain Hector's BFS + m_max_reward * (720 - child->depth()) ;

			if (child->depth() > m_max_depth ) m_max_depth = child->depth();

			
			num_simulated_steps += child->num_simulated_steps;
		}
	
		// // Don't expand duplicate nodes, or terminal nodes
		if (!child->is_terminal) {
		    if (! (ignore_duplicates && test_duplicate(child)) ){				
				if( child->fn !=  m_max_reward )
					q_exploitation->push(child);
				else
					q_exploration->push(child);
		    }
		}

	
	}
	

	curr_node->already_expanded = true;
	return num_simulated_steps;
}

/* *********************************************************************
	update novelty_value to 0 to a node and all its children, all the way down the branch
 ******************************************************************* */
void Newiw::reset_branch(TreeNode* node) {
	if (!node->v_children.empty()) {
		for(size_t c = 0; c < node->v_children.size(); c++) {	
		    //node->v_children[c]->updateTreeNode();	
		    reset_branch(node->v_children[c]);
			
		}
	}
	//node->novelty = 0;
	//node->fn = 0;	
	node->already_expanded = false;
}

int Newiw::reuse_branch(TreeNode* node) {
	int num_simulated_steps = 0;

	node->updateTreeNode();
	update_novelty_table( node->state.getRAM() );

	queue<TreeNode*> q;
	q.push( node );

	while(!q.empty()) {
		// Pop a node to expand
		TreeNode* curr_node = q.front();
		q.pop();
		
		if ( curr_node->depth() > m_reward_horizon - 1 ) continue;
		if (!node->v_children.empty()) {
			for(size_t c = 0; c < node->v_children.size(); c++) {			
				TreeNode* child = curr_node->v_children[c];				
				
				// This recreates the novelty table (which gets resetted every time
				// we change the root of the search tree)
			        if ( m_novelty_pruning ){				

						if ( check_novelty_1( child->state.getRAM() ) ){
							update_novelty_table( child->state.getRAM() );
							if(!child->already_expanded){
								child->novelty = 1;
							}
						}
						else{
							if(!child->already_expanded)
								child->novelty = 2;
							
							
						}
					
				}
				
				child->updateTreeNode();
				child->fn += ( m_max_reward - child->discounted_accumulated_reward ); // Miquel: add this to obtain Hector's BFS + m_max_reward * (720 - child->depth()) ;
				
				//First applicable action
				if(child->depth() == 1)
				    child->num_nodes_reusable = child->num_nodes();
				else
				    child->num_nodes_reusable = curr_node->num_nodes_reusable;
				if (child->depth() > m_max_depth ) m_max_depth = child->depth();
				
				
				num_simulated_steps += child->num_simulated_steps;
				
				// Don't expand duplicate nodes, or terminal nodes
				if (!child->is_terminal) {
					if (! (ignore_duplicates && test_duplicate(child)) ){
						if(!child->already_expanded){
							if( child->fn !=  m_max_reward )
								q_exploitation->push(child);

							q_exploration->push(child);


						}
						else
							q.push(child);

					}
				}
				
			}
		}
		// // Stop once we have simulated a maximum number of steps
		// if (num_simulated_steps >= max_sim_steps_per_frame) {
		// 	break;
		// }
		
	}
	
	return num_simulated_steps;

}

unsigned Newiw::size_branch(TreeNode* node) {
	unsigned size = 1;

	if (!node->v_children.empty()) {
		for(size_t c = 0; c < node->v_children.size(); c++) {			
			size += size_branch(node->v_children[c]);
			
		}
	}
	return size;	

}

/* *********************************************************************
   Expands the tree from the given node until i_max_sim_steps_per_frame
   is reached
	
   ******************************************************************* */

void Newiw::expand_tree(TreeNode* start_node) {
    // If the root is terminal, we will not expand any of its children; deal with this
    //  appropriately
    if (start_node->is_terminal) {
	set_terminal_root(start_node);
	return;
    }

    int num_simulated_steps = 0;
    bool explore = true;
    
    int max_nodes_per_frame = max_sim_steps_per_frame / sim_steps_per_node;
    clear_queues();
    
    if(!start_node->v_children.empty()) {
	    start_node->updateTreeNode();
	    num_simulated_steps += reuse_branch( start_node );
	    std::cout  << "Num_reused_steps: "<< num_simulated_steps << std::endl;
	    num_simulated_steps = 0;
	    //COMMENT LINES BELOW, AND UNCOMMENT ABOVE TO WORKSHOP STYLE. ALSO CHANGE FN NOVEL 2ND QUEUE
	    //reset_branch( start_node );
	    //q_exploration->push(start_node);        
	     update_novelty_table( start_node->state.getRAM() );
    }
    else
	    {
	    q_exploration->push(start_node);        
	    update_novelty_table( start_node->state.getRAM() );
    }

    m_expanded_nodes = 0;
    m_generated_nodes = 0;
    m_exp_count_novelty1 = 0;
    m_exp_count_novelty2 = 0;
    m_gen_count_novelty1 = 0;
    m_gen_count_novelty2 = 0;
    m_pruned_nodes = 0;
    

    while( ! (q_exploration->empty() && q_exploitation->empty()) ) {
	// Pop a node to expand
	TreeNode* curr_node;
	if( q_exploration->empty() &&  q_exploitation->empty())
	    break;

	if( explore ){	    

		if( q_exploration->empty() ){
			explore = false;
			continue;
		}
		curr_node = q_exploration->top();
		q_exploration->pop();
		explore = false;

	}
	else{
		if( q_exploitation->empty() ){
			explore = true;
			continue;
		}

	
	    curr_node = q_exploitation->top();
	    q_exploitation->pop();
	    explore = true;
	}


	if ( curr_node->depth() > m_max_depth ) m_max_depth = curr_node->depth();

	/**
	 * check nodes that have been expanded by other queue
	 */
	 if(  curr_node->already_expanded ) 
	 	continue;
	 
	 /**
	 * check if subtree is bigger than max_budget of nodes
	 */
	 if(  curr_node->num_nodes_reusable > max_nodes_per_frame  ) {
	 	continue;
	 }

	if ( curr_node->depth() > m_reward_horizon - 1 ) continue;


	num_simulated_steps +=  expand_node( curr_node );
	// std::cout << "q_exploration size: "<< q_exploration.size() << std::endl;
	// std::cout << "q_exploitation size: "<< q_exploitation.size() << std::endl;
	// Stop once we have simulated a maximum number of steps
	if (num_simulated_steps >= max_sim_steps_per_frame) {
	    break;
	}
    }
   
    std::cout << "\tExpanded so far something: " << m_expanded_nodes << std::endl;	

    if ( q_exploration->empty() && q_exploitation->empty() ) std::cout << "Search Space Exhausted!" << std::endl;
    std::cout << "q_exploration size: "<< q_exploration->size() << std::endl;
    std::cout << "q_exploitafweiwftion size: "<< q_exploitation->size() << std::endl;

	
    update_branch_return(start_node);
}

