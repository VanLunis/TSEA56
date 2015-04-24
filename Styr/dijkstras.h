#ifndef DIJKSTRAS_H_INCLUDED
#define DIJKSTRAS_H_INCLUDED
#define INFTY 300
#define NORTH 0
#define EAST 1
#define SOUTH 2
// #define WEST 3


typedef struct node
{
    struct node *north;
    struct node *south;
    struct node *east;
    struct node *west;
    uint8_t node_cost_north;
    uint8_t node_cost_south;
    uint8_t node_cost_east;
    uint8_t node_cost_west;
} NODE;

/*
 
 NODE * addnode(NODE * prior_node, uint8_t direction_prior_node, uint8_t direction_new_node, uint8_t new_node_cost ){
	
	//	adds and returns a new node when found and updates both prior and new node connections and node costs.
	
	NODE * new_node = malloc(sizeof(NODE)); // allocates memory for a new found node
	
	
	//connects prior node with new node and updates corresponding node costs
	
	switch (direction_new_node)
	{
 case NORTH:
 new_node->node_cost_north = new_node_cost;
 switch (direction_prior_node)
 {
 case NORTH:
 new_node->north = prior_node->north;
 prior_node->node_cost_north = new_node_cost;
 break;
 case EAST:
 new_node->north = prior_node->east;
 prior_node->node_cost_east = new_node_cost;
 break;
 case SOUTH:
 new_node->north = prior_node->south;
 prior_node->node_cost_south = new_node_cost;
 break;
 case WEST:
 new_node->north = prior_node->west;
 prior_node->node_cost_west = new_node_cost;
 break;
 }
 break;
 case EAST:
 new_node->node_cost_east = new_node_cost;
 switch (direction_prior_node)
 {
 case NORTH:
 new_node->east = prior_node->north;
 prior_node->node_cost_north = new_node_cost;
 break;
 case EAST:
 new_node->east = prior_node->east;
 prior_node->node_cost_east = new_node_cost;
 break;
 case SOUTH:
 new_node->east = prior_node->south;
 prior_node->node_cost_south = new_node_cost;
 break;
 case WEST:
 new_node->east = prior_node->west;
 prior_node->node_cost_west = new_node_cost;
 break;
 }
 break;
 case SOUTH:
 new_node->node_cost_south = new_node_cost;
 switch (direction_prior_node)
 {
 case NORTH:
 new_node->south = prior_node->north;
 prior_node->node_cost_north = new_node_cost;
 break;
 case EAST:
 new_node->south = prior_node->east;
 prior_node->node_cost_east = new_node_cost;
 break;
 case SOUTH:
 new_node->south = prior_node->south;
 prior_node->node_cost_south = new_node_cost;
 break;
 case WEST:
 new_node->south = prior_node->west;
 prior_node->node_cost_west = new_node_cost;
 break;
 }
 break;
 case WEST:
 new_node->node_cost_west = new_node_cost;
 switch (direction_prior_node)
 {
 case NORTH:
 new_node->west = prior_node->north;
 prior_node->node_cost_north = new_node_cost;
 break;
 case EAST:
 new_node->west = prior_node->east;
 prior_node->node_cost_east = new_node_cost;
 break;
 case SOUTH:
 new_node->west = prior_node->south;
 prior_node->node_cost_south = new_node_cost;
 break;
 case WEST:
 new_node->west = prior_node->west;
 prior_node->node_cost_west = new_node_cost;
 break;
 }
 break;
	} // end of switch
 
	return new_node;
 }
 
 
 
 struct node_stack
 {
	
 };
 
 void push()
 {
	
 }
 struct dijkstras_node pop()
 {
	
 }
 
 */

#endif // DIJKSTRAS_H_INCLUDED
