node floodgrid[17][17];
int found;
int nqueue;
int queue[][2];
int cost;

struct node{
	int cost;
	int drivable;
	node* neighbor[3];
	};








void floodfill(int current[2], int end[2], nextcost){
	if (abs(current[0]-end[0]) <= 1 or abs(current[1]-end[1]) <= 1){
		endclose = 1;
	}
	int paths = 0;
	
	node *curnode = floodgrid[current[1]][current[0]];
	curnode->cost == nextcost;
	
	for (int i = 0; i<3; i++){
		if 
		
		if (curnode->neighbor[i]->drivable and curnode->neighbor[i]->cost <= newcost){
			updateAll(nextcost);
			break;
		}
		
		else if (curnode->neighbor[i]->drivable)
		{
				curnode->neighbor[i]->cost == nextcost;
				paths++;
		}
		
	}

	}
	
}



















/*

int[][2] add_drivable(int node[2], int end[2]){
	
	if (node[0] > 0){
		if (drivable[node[0]-1][node[1]] == 1){
			queue[nqueue][0] = node[0]-1;
			queue[nqueue][1] = node[1];
			nqueue++;
		}
	}
	if (node[0] < 17){
		if (drivable[node[0]+1][node[1]] == 1){
			queue[nqueue][0] = node[0]+1;
			queue[nqueue][1] = node[1];
			nqueue++;
		}
	}
	if (node[1] > 0){
		if (drivable[node[0]][node[1]-1] == 1){
			queue[nqueue][0] = node[0];
			queue[nqueue][1] = node[1]-1;
			nqueue++;
		}
	}
	if (node[1] < 17){
		if (drivable[node[0]][node[1]+1] == 1){
			queue[nqueue][0] = node[0];
			queue[nqueue][1] = node[1]+1;
			nqueue++;
		}
	}
}

void val(int end[2]){
	if (queue[nqueue] == end){
		
	}
}

int floodfill(int node[2], int end[2]){
	cost = 0;
	while (not found){
		add_drivable();
		val();
	}
	
	
	*/
	
	
	
}