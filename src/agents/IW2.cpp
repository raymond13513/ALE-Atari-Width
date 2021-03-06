#include "IW2.hpp"
#include "SearchAgent.hpp"
#include <list>
#include <cstdlib>
#define MAXI 300000
#define PROB 0.5
IW2::IW2(RomSettings *rom_settings, Settings &settings,
			       ActionVect &actions, StellaEnvironment* _env) 
    : IW1Search( rom_settings, settings, actions, _env){
 
	m_max_reward = settings.getInt( "max_reward" );
    //create a new pq ranked by novelty
	q_exploration = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploration >;
    //create a new pg ranked by reward
	q_exploitation = new std::priority_queue<TreeNode*, std::vector<TreeNode*>, TreeNodeComparerExploitation >;
}

IW2::~IW2() {
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

int IW2:: calculate_novelty(TreeNode * child_node){

    return MAXI - check_novelty_1( child_node->state.getRAM()); 
}

void IW2:: pushqueue(TreeNode* child){
    if (!child->is_terminal) {
        if (! (ignore_duplicates && test_duplicate(child)) ){	
            	if (child -> novelty != MAXI)
		            q_exploration->push(child);
                if (child-> fn != m_max_reward)
                    q_exploitation->push(child);
            
                }
    }
    
}

int IW2:: expand_node(TreeNode* curr_node){
    int num_simulated_steps =0;
    //cout << "come";
    bool leaf_node = (curr_node->v_children.empty());
    int num_actions = available_actions.size();
    // if leaf node we resize the children list
    if (leaf_node){
        curr_node->v_children.resize( num_actions );
		curr_node->available_actions = available_actions;
        //shuffle the order of operations
        if(m_randomize_successor)
			std::random_shuffle ( curr_node->available_actions.begin(), curr_node->available_actions.end() );

    }
    TreeNode * child;
    //exploring all actions 
    std::vector<TreeNode *> expanded_childs = std::vector<TreeNode*>();   
    for (int a = 0; a < num_actions; a++) {
        Action act = curr_node->available_actions[a];
		
        //To avoid creating new child if the parent is previously explored, we check if the node is leaf or not
        if (leaf_node){
        	m_generated_nodes++;
            //create a new child
			child = new TreeNode(	curr_node,	
						curr_node->state,
						this,
						act,
						sim_steps_per_node
						, discount_factor); 
            //add in to the child list
            curr_node->v_children[a] = child;
            child->updateTreeNode();
            num_simulated_steps += child->num_simulated_steps;
        }
        else{
 
            child = curr_node ->v_children[a];
            child ->updateTreeNode();
        }
        //now we have our child, do stuff to it
        //calculate the child's novelty
        child ->novelty = calculate_novelty(child);
        if (child->novelty != MAXI){
            
            update_novelty_table( child->state.getRAM() );
        }
            
                
        child->fn += ( m_max_reward - child->discounted_accumulated_reward ); // Miquel: add this to obtain Hector's BFS + m_max_reward * (720 - child->depth()) ;
        if (child->depth() > m_max_depth ) m_max_depth = child->depth();   
        
       
    
		// Push the child node to the queue
        pushqueue(child);

    }
    // only update the novelty table of the child that have maximum novelty to dampen the agressive pruning

    //for ( auto &i : expanded_childs) {
    //    update_novelty_table( i->state.getRAM() );
    //}
    curr_node->already_expanded = true;
	return num_simulated_steps;
        
} 

/* *********************************************************************
	update novelty_value to 0 to a node and all its children, all the way down the branch
 ******************************************************************* */
void IW2::reset_branch(TreeNode* node) {
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

int IW2::reuse_branch(TreeNode* node) {
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
                        int temp = MAXI -check_novelty_1( child->state.getRAM() ) ;
						if ( temp != MAXI){
							update_novelty_table( child->state.getRAM() );
							if(!child->already_expanded){
								child->novelty = temp;
							}
						}
						else{
							if(!child->already_expanded)
								child->novelty = temp;
							
							
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
                if (child->already_expanded){
                    q.push(child);
                }
                else{
                    pushqueue(child);
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

unsigned IW2::size_branch(TreeNode* node) {
	unsigned size = 1;

	if (!node->v_children.empty()) {
		for(size_t c = 0; c < node->v_children.size(); c++) {			
			size += size_branch(node->v_children[c]);
			
		}
	}
	return size;	

}

TreeNode *IW2::choose_node(){
    bool decide =  (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) >0.5);
    decide = false;
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

/* *********************************************************************
   Expands the tree from the given node until i_max_sim_steps_per_frame
   is reached
	
   ******************************************************************* */

void IW2::expand_tree(TreeNode* start_node) {
    clear_queues();
    start_node ->updateTreeNode();
    start_node -> already_expanded = false;
   //check if start node is expanded or not
    //if expanded
    //cout <<"depth: "<< start_node -> depth()<<"is empty"<<start_node ->v_children.empty()<<"Novelty: "<<start_node ->novelty;
   
    
    update_novelty_table(start_node->state.getRAM());
    q_exploration -> push(start_node);
    int num_simulated_steps = 0;
    
    int max_nodes_per_frame = max_sim_steps_per_frame / sim_steps_per_node;
    

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

        curr_node = choose_node();
        if (curr_node == NULL)
            break;
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
    if( m_novelty_int_representation ){
		std::cout <<"New Update: "<< std::endl;	
		for ( size_t i = 0; i < RAM_SIZE; i++ ){
			std::cout <<"Byte "<< i <<": ";	
			for(int j = 0; j < 256; j++) {
				int count = m_ram_int_novelty_table->at( i )->at( j );
				if( count > 0)
					std::cout << "[" << j << "]: " << count << " " ;
			}
			std::cout << std::endl;
		}
    }
    
    std::cout << "\tExpanded so far: " << m_expanded_nodes << std::endl;	
    std::cout << "\tExpanded Novelty 1: " << m_exp_count_novelty1 << std::endl;	
    std::cout << "\tExpanded Novelty 2: " << m_exp_count_novelty2 << std::endl;	
    std::cout << "\tPruned so far: " << m_pruned_nodes << std::endl;	
    std::cout << "\tGenerated so far: " << m_generated_nodes << std::endl;	
    std::cout << "\tGenerated Novelty 1: " << m_gen_count_novelty1 << std::endl;	
    std::cout << "\tGenerated Novelty 2: " << m_gen_count_novelty2 << std::endl;	

    if ( q_exploration->empty() && q_exploitation->empty() ) std::cout << "Search Space Exhausted!" << std::endl;
    std::cout << "q_exploration size: "<< q_exploration->size() << std::endl;
    std::cout << "q_exploitation size: "<< q_exploitation->size() << std::endl;

	
    update_branch_return(start_node);
}

