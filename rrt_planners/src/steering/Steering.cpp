#include <rrt_planners/steering/Steering.h>

RRT::Steering::Steering() {
	
	space_ = NULL;
	
	maxRange_ = 0.25;
	
	timeStep_ = 0.1; 
	minControlSteps_ = 5; //minTime = timeStep*minControlSteps
	maxControlSteps_ = 10;
	
	maxLinearAcc_ = 1.0;   // m/s²
	maxAngularAcc_ = 2.0; // rad/s²
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	steeringType_ = 2;
	motionCostType_ = 2;
}


RRT::Steering::Steering(StateSpace* sp) {
	
	space_ = sp;
	
	maxRange_ = 0.25; //setMaxRange is called later
	
	timeStep_ = 0.1; 
	minControlSteps_ = 5; //minTime = timeStep*minControlSteps
	maxControlSteps_ = 10;
	
	maxLinearAcc_ = 1.0;   // m/s²
	maxAngularAcc_ = 2.0; // rad/s²
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	steeringType_ = 2;
	motionCostType_ = 2;
	
	kp_ = space_->getMaxLinVel();
	kv_ = 3.0;
	ka_ = space_->getMaxAngVel()*4.0;
	ko_ = ka_/8.0;
}


RRT::Steering::Steering(StateSpace* sp, float max_range) 
	: maxRange_(max_range) {
	
	space_ = sp;
	
	timeStep_ = 0.1; 
	minControlSteps_ = 5; //minTime = timeStep*minControlSteps
	maxControlSteps_ = 10;
	
	maxLinearAcc_ = 1.0;   // m/s²
	maxAngularAcc_ = 2.0; // rad/s²
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	steeringType_ = 2;
	motionCostType_ = 2;
}


RRT::Steering::Steering(StateSpace* sp, float tstep, int minSteps, int maxSteps, float lAccMax, float aAccMax) 
	: timeStep_(tstep),
	minControlSteps_(minSteps), maxControlSteps_(maxSteps), 
	maxLinearAcc_(lAccMax), maxAngularAcc_(aAccMax) {
		
	space_ = sp;	
		
	maxRange_ = 0.0;
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	steeringType_ = 2;
	motionCostType_ = 2;
}	

RRT::Steering::Steering(StateSpace* sp, float max_range, float tstep, int minSteps, int maxSteps, float lAccMax, float aAccMax) 
	: space_(sp), maxRange_(max_range), timeStep_(tstep),
	minControlSteps_(minSteps), maxControlSteps_(maxSteps), 
	maxLinearAcc_(lAccMax), maxAngularAcc_(aAccMax) {
		
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	steeringType_ = 2;
	motionCostType_ = 2;
	
	kp_ = space_->getMaxLinVel();
	kv_ = 3.0;
	ka_ = space_->getMaxAngVel()*4.0;
	ko_ = ka_/8.0;
	
}



RRT::Steering::~Steering() {
	//delete space_;
}


void RRT::Steering::setSteeringParams(float kp, float kv, float ka, float ko)
{
	kp_ = kp;
	kv_ = kv;
	ka_ = ka;
	ko_ = ko;
}



RRT::State* RRT::Steering::simpleSteer(State* fromState, State* toState, std::vector<State>& istates)
{
	//State* newState = NULL;

	//Distance between points
	float dx = (toState->getX()-fromState->getX());
	float dy = (toState->getY()-fromState->getY());
	float dt = atan2(dy, dx);
	float dist = sqrt(dx*dx+dy*dy);
	
	float res = space_->getXYZresolution();
	unsigned int steps;
	if(dist >= maxRange_)
		steps = (int) floor(maxRange_/res + 0.5);
	else
		steps = (int) floor(dist/res + 0.5);

	//printf("Double distance: %.2f, steps: %u, res: %.2f\n", dist, steps, res); 
	
	State* aux = new State(fromState->getX(), fromState->getY());
	for(unsigned int i=0; i<steps; i++)
	{
		float newx = aux->getX() + res*cos(dt);
		float newy = aux->getY() + res*sin(dt);
		delete aux;
		aux = new State(newx, newy);
		if(space_->isStateValid(aux)) {
			//if(newState)
			//	delete newState;
			//newState = new State(newx, newy);
			//istates.push_back(*newState);
			istates.push_back(*aux);
		} else {
			delete aux;
			aux = NULL;
			break;
		}
	}
	if(aux)
		delete aux;

	State* newState = NULL;
	if(istates.size() > 0)
		newState = &(istates.at(istates.size()-1));
		
	return newState;
}







RRT::State* RRT::Steering::simple3dSteer(State* fromState, State* toState, std::vector<State>& istates)
{
	//State* newState = NULL;
	//printf("simple3dSteer. fromState z:%.2f, toState z:%.2f\n", fromState->getZ(), toState->getZ());

	//Distance between points
	float dx = (toState->getX()-fromState->getX());
	float dy = (toState->getY()-fromState->getY());
	float dz = (toState->getZ()-fromState->getZ());
	float dt = atan2(dy, dx);
	float dist = sqrt(dx*dx+dy*dy+dz*dz);
	
	float res = space_->getXYZresolution();
	unsigned int steps;
	if(dist >= maxRange_)
		steps = (int) floor(maxRange_/res + 0.5);
	else
		steps = (int) floor(dist/res + 0.5);

	//printf("Double distance: %.2f, steps: %u, res: %.2f\n", dist, steps, res); 
	
	State* aux = new State(fromState->getX(), fromState->getY(), fromState->getZ());
	for(unsigned int i=0; i<steps; i++)
	{
		float newx = aux->getX() + res*cos(dt);
		float newy = aux->getY() + res*sin(dt);
		float newz = aux->getZ();
		delete aux;
		aux = new State(newx, newy, newz);
		if(space_->getValid3dState(aux)) {
			//printf("Steering. simple3dSteer. state step [%u] valid!!!\n", i);
			//if(newState)
			//	delete newState;
			//newState = new State(newx, newy);
			//istates.push_back(*newState);
			istates.push_back(*aux);
		} else {
			//printf("Steering. simple3dSteer. state step [%u] NOT valid!!!\n", i);
			delete aux;
			aux = NULL;
			break;
		}
	}
	if(aux)
		delete aux;

	State* newState = NULL;
	if(istates.size() > 0) {
		newState = &(istates.at(istates.size()-1));
	} else
		//printf("Steering. simple3dSteer. istates is empty!!!\n");
	return newState;
}











bool RRT::Steering::simple3dCollisionFree(State* fromState, State* toState, std::vector<State>& istates)
{

	//Distance between points
	float dx = (toState->getX()-fromState->getX());
	float dy = (toState->getY()-fromState->getY());
	float dz = (toState->getZ()-fromState->getZ());
	float dt = atan2(dy, dx);
	float dist = sqrt(dx*dx+dy*dy+dz*dz);

	float res = space_->getXYZresolution();
	unsigned int steps = (int) floor(dist/res + 0.5);
	
	//Check the validity of the steps of the line from the initial state
	//to the max_range distance  
	State* aux = new State(fromState->getX(), fromState->getY(), fromState->getZ());
	for(unsigned int i=0; i<steps; i++)
	{
		float newx = aux->getX() + cos(dt)*res;
		float newy = aux->getY() + sin(dt)*res;
		float newz = aux->getZ();
		delete aux;
		aux = new State(newx, newy, newz);
		if(!space_->getValid3dState(aux)) {
			delete aux;
			return false;
		}
		istates.push_back(*aux);
	}
	if(aux)
		delete aux;
		
	return true;
}





bool RRT::Steering::simpleCollisionFree(State* fromState, State* toState, std::vector<State>& istates)
{

	//Distance between points
	float dx = (toState->getX()-fromState->getX());
	float dy = (toState->getY()-fromState->getY());
	//float dz = (toState->getZ()-fromState->getZ());
	float dt = atan2(dy, dx);
	float dist = sqrt(dx*dx+dy*dy);

	float res = space_->getXYZresolution();
	unsigned int steps = (int) floor(dist/res + 0.5);
	
	//Check the validity of the steps of the line from the initial state
	//to the max_range distance  
	State* aux = new State(fromState->getX(), fromState->getY());
	for(unsigned int i=0; i<steps; i++)
	{
		float newx = aux->getX() + cos(dt)*res;
		float newy = aux->getY() + sin(dt)*res;
		delete aux;
		aux = new State(newx, newy);
		if(!space_->isStateValid(aux)) {
			delete aux;
			return false;
		}
		istates.push_back(*aux);
	}
	if(aux)
		delete aux;
		
	return true;
}








//Steering used in KinoRRT (only 2 dimensions, and one action between states)
bool RRT::Steering::rrt_steer(Node* fromNode, Node* toNode, Node* newNode)
{
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		return false;
	}	
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	//Robot position
	float rx = fromNode->getState()->getX();
	float ry = fromNode->getState()->getY();
	float rth = fromNode->getState()->getYaw();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	//printf("xr:%.2f, yr:%.2f, xw:%.2f, yw:%.2f\n", rx, ry, wx, wy); 
	
	// Transform way-point into local robot frame and get desired x,y,theta
	float dx = (wx-rx)*cos(rth) + (wy-ry)*sin(rth);
	float dy =-(wx-rx)*sin(rth) + (wy-ry)*cos(rth);
	float dist = sqrt(dx*dx + dy*dy);
	float dt = atan2(dy, dx);
	
	
	//Velocities to command
	float lv = kp_ * exp(-fabs(dt))* tanh(3*dist);
	float av = space_->getMaxAngVel() * dt;
	
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	
	// linear vel
	if(fabs(prev_lv - lv) > max_lv_var_) {
		if(lv < prev_lv)
				lv = prev_lv - max_lv_var_;
			else
				lv = prev_lv + max_lv_var_;
	} 
	// angular vel
	if(fabs(prev_av - av) > max_av_var_) {
		if(av < prev_av)
				av = prev_av - max_av_var_;
			else
				av = prev_av + max_av_var_;
	} 
	
	if(lv > space_->getMaxLinVel())
		lv = space_->getMaxLinVel();
	else if(lv < space_->getMinLinVel())
		lv = space_->getMinLinVel();
	
	if(av > space_->getMaxAngVel())
		av = space_->getMaxAngVel();
	else if(av < (-space_->getMaxAngVel()))
		av = space_->getMaxAngVel()*(-1);
		
	//Dead areas
	/*if(fabs(lv) < 0.08)
		lv = 0.0;
	if(fabs(av) < 0.05)
		av = 0.0;
	*/
	
	
	State currentState = *fromNode->getState();
	
	int numSteps = 0;
	while(numSteps <= maxControlSteps_ && dist > space_->getGoalXYZTolerance()) 
	{
			
		State* newState = propagateStep(&currentState, lv, av);
		
		if(!space_->isStateValid(newState)) {
			delete newState;
			break;
		}
		
		currentState = *newState;
		delete newState;
		
		istates.push_back(currentState);
		numSteps++;
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
	}
	
	if(numSteps == 0) {
		return false;
	}
	Action action(lv, 0.0, av, numSteps);
	//Node* newNode = new Node(currentState, action); 
	newNode->setState(currentState);
	newNode->addAction(action);
	newNode->setIntermediateStates(istates);
	
	return true;
	
}


//Steering used in KinoRRT (only 2 dimensions, and one action between states)
bool RRT::Steering::rrt_3d_steer(Node* fromNode, Node* toNode, Node* newNode)
{
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. Start node is NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Final node is NULL\n"); 
		return false;
	}	
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	//Robot position
	float rx = fromNode->getState()->getX();
	float ry = fromNode->getState()->getY();
	float rz = fromNode->getState()->getZ();
	float rth = fromNode->getState()->getYaw();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wz = toNode->getState()->getZ();
	float wth = toNode->getState()->getYaw();
	
	//printf("xr:%.2f, yr:%.2f, xw:%.2f, yw:%.2f\n", rx, ry, wx, wy); 
	
	// Transform way-point into local robot frame and get desired x,y,theta
	float dx = (wx-rx)*cos(rth) + (wy-ry)*sin(rth);
	float dy =-(wx-rx)*sin(rth) + (wy-ry)*cos(rth);
	float dz = (wz-rz);
	float dist = sqrt(dx*dx + dy*dy + dz*dz);
	float dt = atan2(dy, dx);
	
	
	//Velocities to command
	float lv = kp_ * exp(-fabs(dt))* tanh(3*dist);
	float av = space_->getMaxAngVel() * dt;
	
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	
	// linear vel
	if(fabs(prev_lv - lv) > max_lv_var_) {
		if(lv < prev_lv)
				lv = prev_lv - max_lv_var_;
			else
				lv = prev_lv + max_lv_var_;
	} 
	// angular vel
	if(fabs(prev_av - av) > max_av_var_) {
		if(av < prev_av)
				av = prev_av - max_av_var_;
			else
				av = prev_av + max_av_var_;
	} 
	
	if(lv > space_->getMaxLinVel())
		lv = space_->getMaxLinVel();
	else if(lv < space_->getMinLinVel())
		lv = space_->getMinLinVel();
	
	if(av > space_->getMaxAngVel())
		av = space_->getMaxAngVel();
	else if(av < (-space_->getMaxAngVel()))
		av = space_->getMaxAngVel()*(-1);
		
	//Dead areas
	/*if(fabs(lv) < 0.08)
		lv = 0.0;
	if(fabs(av) < 0.05)
		av = 0.0;
	*/
	
	
	State currentState = *fromNode->getState();
	
	int numSteps = 0;
	while(numSteps <= maxControlSteps_ && dist > space_->getGoalXYZTolerance()) 
	{
			
		State* newState = propagateStep(&currentState, lv, av);
		
		//if(!space_->isStateValid(newState)) {
		if(!space_->getValid3dState(newState)) {
			delete newState;
			break;
		}
		
		currentState = *newState;
		delete newState;
		
		istates.push_back(currentState);
		numSteps++;
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()) 
		+ (wz-currentState.getZ())*(wz-currentState.getZ()) );
	}
	
	if(numSteps == 0) {
		return false;
	}
	Action action(lv, 0.0, av, numSteps);
	//Node* newNode = new Node(currentState, action); 
	newNode->setState(currentState);
	newNode->addAction(action);
	newNode->setIntermediateStates(istates);
	
	return true;
	
}




/*
//Steering used in KinoRRT (only 2 dimensions, and one action between states)
bool RRT::Steering::accompany_steer(Node* fromNode, Node* toNode, Node* newNode)
{
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		return false;
	}	
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	//Robot position
	float rx = fromNode->getState()->getX();
	float ry = fromNode->getState()->getY();
	float rth = fromNode->getState()->getYaw();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	//printf("xr:%.2f, yr:%.2f, xw:%.2f, yw:%.2f\n", rx, ry, wx, wy); 
	
	// Transform way-point into local robot frame and get desired x,y,theta
	float dx = (wx-rx)*cos(rth) + (wy-ry)*sin(rth);
	float dy =-(wx-rx)*sin(rth) + (wy-ry)*cos(rth);
	float dist = sqrt(dx*dx + dy*dy);
	float dt = atan2(dy, dx);
	
	
	//We need to know the target position at the same
	//time that the robot is in 'fromNode'
	//double x,y;
	//xxxx_->getTargetPosition(double t, x, y);
	//Calculate the distance between the robot and the target
	//float dist = sqrt((x-rx)*(x-rx) + (y-ry)*(y-ry));
	
	
	//Velocities to command
	float lv = space_->getMaxLinVel() * (dist/1.5); // *exp(-fabs(dt))
	float av = space_->getMaxAngVel() * dt;
	
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	
	// linear vel
	if(fabs(prev_lv - lv) > max_lv_var_) {
		if(lv < prev_lv)
				lv = prev_lv - max_lv_var_;
			else
				lv = prev_lv + max_lv_var_;
	} 
	// angular vel
	if(fabs(prev_av - av) > max_av_var_) {
		if(av < prev_av)
				av = prev_av - max_av_var_;
			else
				av = prev_av + max_av_var_;
	} 
	
	if(lv > space_->getMaxLinVel())
		lv = space_->getMaxLinVel();
	else if(lv < space_->getMinLinVel())
		lv = space_->getMinLinVel();
	
	if(av > space_->getMaxAngVel())
		av = space_->getMaxAngVel();
	else if(av < (-space_->getMaxAngVel()))
		av = space_->getMaxAngVel()*(-1);
		
	//Dead areas
	//if(fabs(lv) < 0.08)
	//	lv = 0.0;
	//if(fabs(av) < 0.05)
	//	av = 0.0;
	
	
	
	State currentState = *fromNode->getState();
	
	int numSteps = 0;
	while(numSteps <= maxControlSteps_ && dist > space_->getGoalXYZTolerance()) 
	{
			
		State* newState = propagateStep(&currentState, lv, av);
		
		if(!space_->isStateValid(newState)) {
			delete newState;
			break;
		}
		
		currentState = *newState;
		delete newState;
		
		istates.push_back(currentState);
		numSteps++;
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
	}
	
	if(numSteps == 0) {
		return false;
	}
	Action action(lv, 0.0, av, numSteps);
	//Node* newNode = new Node(currentState, action); 
	newNode->setState(currentState);
	newNode->addAction(action);
	newNode->setIntermediateStates(istates);
	
	return true;
	
}*/




bool RRT::Steering::rrt_collisionFree(Node* fromNode, Node* toNode, Node& out)
{
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		return false;
	}	
	
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	//Robot position
	float rx = fromNode->getState()->getX();
	float ry = fromNode->getState()->getY();
	float rth = fromNode->getState()->getYaw();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	//printf("xr:%.2f, yr:%.2f, xw:%.2f, yw:%.2f\n", rx, ry, wx, wy); 
	
	// Transform way-point into local robot frame and get desired x,y,theta
	float dx = (wx-rx)*cos(rth) + (wy-ry)*sin(rth);
	float dy =-(wx-rx)*sin(rth) + (wy-ry)*cos(rth);
	float dist = sqrt(dx*dx + dy*dy);
	float dt = atan2(dy, dx);
	
	
	//Velocities to command
	float lv = kp_ * exp(-fabs(dt))* tanh(3*dist);
	float av = space_->getMaxAngVel() * dt;
	
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	
	// linear vel
	if(fabs(prev_lv - lv) > max_lv_var_) {
		if(lv < prev_lv)
				lv = prev_lv - max_lv_var_;
			else
				lv = prev_lv + max_lv_var_;
	} 
	// angular vel
	if(fabs(prev_av - av) > max_av_var_) {
		if(av < prev_av)
				av = prev_av - max_av_var_;
			else
				av = prev_av + max_av_var_;
	} 
	
	if(lv > space_->getMaxLinVel())
		lv = space_->getMaxLinVel();
	else if(lv < space_->getMinLinVel())
		lv = space_->getMinLinVel();
	
	if(av > space_->getMaxAngVel())
		av = space_->getMaxAngVel();
	else if(av < (-space_->getMaxAngVel()))
		av = space_->getMaxAngVel()*(-1);
		
	//Dead areas
	/*if(fabs(lv) < 0.08)
		lv = 0.0;
	if(fabs(av) < 0.05)
		av = 0.0;
	*/
	
	
	//printf("printf: dt: %.2f, dist:%.2f, lv: %.2f, av: %.2f \n\n", dt, dist, lv, av);
	
	float max_dist_step = space_->getMaxLinVel() * timeStep_;
	float approx_steps = dist/max_dist_step;
	
	
	State newState = *fromNode->getState();
	
	int numSteps = 0;
	
	while (dist >= space_->getGoalXYZTolerance()) {
		
		//Check that the path to waypoint is not too long
		if(numSteps > ceil(approx_steps*2)) { 
			return false;
		}
		
		//newState = *propagateStep(&newState, lv, av);
		State* st = propagateStep(&newState, lv, av);
		
		//Check if the state is valid
		if(!space_->isStateValid(st)) {
			delete st;
			return false;
		}
		
		newState = *st;
		delete st;
		
		//printf("Step: %i. NewState x:%.2f, y:%.2f, th:%.2f\n", numSteps, newState->getX(), newState->getY(), newState->getYaw());
		istates.push_back(newState);
		numSteps++;
		dist = sqrt((wx-newState.getX())*(wx-newState.getX()) + (wy-newState.getY())*(wy-newState.getY()));
	}
	
	if(numSteps == 0) {
		return false;
	}
	
	Action newAction(lv, 0.0, av, numSteps);
	out.setState(newState);
	std::vector<Action> act;
	act.push_back(newAction);
	out.setAction(act);
	out.setIntermediateStates(istates);
	return true;
}






// Steering method in 2 dimensions (x, y) - Interpolation of the motion cost
bool RRT::Steering::steer2(Node* fromNode, Node* toNode, Node* newNode)
{
	//Node * newNode = NULL;
	//Node * prevNode = NULL;
	std::vector<Action> actions;
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		return false;
	}	
	
	
	//Max velocities variations in one time step
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	float incCost = 0.0;
	float AccCost = fromNode->getAccCost();
	
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	
	float lv = 0.0, av=0.0;
	
	float phi = 0.0;
	float dist = 100.0;
	int numSteps = 0;
	
	//Node currentNode = *fromNode;
	State currentState = *fromNode->getState();
	Node currentNode(currentState);
	currentNode.setCost(space_->getCost(currentNode.getState()));
	
	while(numSteps <= maxControlSteps_ && dist > space_->getGoalXYZTolerance())
	{
		
		// Transform way-point into local robot frame and get desired x,y,theta
		float dx = (wx-currentState.getX())*cos(currentState.getYaw()) + (wy-currentState.getY())*sin(currentState.getYaw());
		float dy =-(wx-currentState.getX())*sin(currentState.getYaw()) + (wy-currentState.getY())*cos(currentState.getYaw());
		if(numSteps == 0)
			dist = sqrt(dx*dx + dy*dy);  

		float alpha = atan2(dy, dx);
		
		//Astolfi
		//float lv = kp * dist;
		//float av = ka * alpha; // + ko * phi;
		
		if(steeringType_ == 1) {
			//POSQ
			lv = kp_ * tanh(kv_*dist);
			av = ka_ * alpha; 
		} else {
			//Improved-POSQ
			lv = kp_ * tanh(kv_*dist) * exp(-fabs(alpha));
			av = ka_ * alpha;
		}
		
		//Check velocities reacheability
		// linear vel
		if(fabs(prev_lv - lv) > max_lv_var_) {
			if(lv < prev_lv)
					lv = prev_lv - max_lv_var_;
				else
					lv = prev_lv + max_lv_var_;
		} 
		// angular vel
		if(fabs(prev_av - av) > max_av_var_) {
			if(av < prev_av)
					av = prev_av - max_av_var_;
				else
					av = prev_av + max_av_var_;
		} 
		
		//Check max and min velocities
		if(lv > space_->getMaxLinVel())
			lv = space_->getMaxLinVel();
		else if(lv < space_->getMinLinVel())
			lv = space_->getMinLinVel();
		
		if(av > space_->getMaxAngVel())
			av = space_->getMaxAngVel();
		else if(av < (-space_->getMaxAngVel()))
			av = space_->getMaxAngVel()*(-1);
			
		//Dead velocity ranges
		//if(fabs(lv) < 0.08)
		//	lv = 0.0;
		//if(fabs(av) < 0.05)
		//	av = 0.0;
		
		//Propagate the movement
		State* st = propagateStep(&currentState, lv, av);
		
		//Break the loop is the state is not valid
		if(!space_->isStateValid(st)) {
			delete st;
			break;
		}
		
		Node nextNode(*st);
		nextNode.setCost(space_->getCost(st));
		float mc = motionCost(&currentNode, &nextNode);
		incCost += mc;
		currentState = *st;
		delete st;
		currentNode = nextNode;
		currentNode.setIncCost(incCost);
		
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
		istates.push_back(currentState);
		Action a(lv, 0.0, av, 1);
		actions.push_back(a);
	
		prev_lv = lv;
		prev_av = av;
	
		numSteps++;
	}
	
	if(numSteps == 0){
		return false;
	}
		
	//currentNode.setAction(actions);
	//currentNode.setIntermediateStates(istates);
	//currentNode.setAccCost(AccCost + incCost);
	
	//newNode = new Node(currentState);
	newNode->setState(currentState);
	newNode->setAction(actions);
	newNode->setIntermediateStates(istates);
	newNode->setCost(currentNode.getCost());
	newNode->setIncCost(incCost);
	newNode->setAccCost(AccCost + incCost);
	
	return true;
}





//Steering for collision checking in two dimensions (x, y) - Interpolation of the motion cost
bool RRT::Steering::collisionFree2(Node* fromNode, Node* toNode, std::vector<Action>& acts, std::vector<State>& istates, float& motCost)
{
	std::vector<Action> actions;
	std::vector<State> inter_states;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		motCost = 0.0;
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		motCost = 0.0;
		return false;
	}	
	
	
	float incCost = 0.0;
	//float AccCost = fromNode->getAccCost();
	
	//Max velocities variations in one time step
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act->at(act.size()-1)->getVx();
	//float prev_av = act->at(act.size()-1)->getVth();
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	
	float lv = 0.0, av=0.0;
	
	float phi = 0.0;
	float xs = wx - fromNode->getState()->getX();
	float ys = wy - fromNode->getState()->getY();
	float init_dist = sqrt(xs*xs + ys*ys);  //10.0;
	float dist = init_dist;
	
	float max_dist_step = space_->getMaxLinVel() * timeStep_;
	float approx_steps = init_dist/max_dist_step;
	
	
	//Initial robot position
	State currentState = *fromNode->getState();
	//Node currentNode(currentState);
	Node currentNode(*fromNode);
	
	
	int numSteps = 0;
	
	while (dist >= space_->getGoalXYZTolerance())  
	{
		
		//Check that the path to waypoint is not too long
		if(numSteps > ceil(approx_steps*3)) { 
			//printf("Step number > %.1f\n", ceil(approx_steps*5));
			motCost = 0.0;	
			return false;
		}
		
		// Transform way-point into local robot frame and get desired x,y,theta
		float dx = (wx-currentState.getX())*cos(currentState.getYaw()) + (wy-currentState.getY())*sin(currentState.getYaw());
		float dy =-(wx-currentState.getX())*sin(currentState.getYaw()) + (wy-currentState.getY())*cos(currentState.getYaw());
		if(numSteps == 0)
			dist = sqrt(dx*dx + dy*dy);  

		float alpha = atan2(dy, dx);
		
		
		//Velocities to command
		if(steeringType_ == 1) {
			//POSQ
			lv = kp_ * tanh(kv_*dist);
			av = ka_ * alpha; // + ko * phi;
		} else {
			//Improved-POSQ
			lv = kp_ * tanh(kv_*dist) * exp(-fabs(alpha));
			av = ka_ * alpha; // + ko * phi;
		}
		
		
		//Check velocities reacheability
		
		// linear vel
		if(fabs(prev_lv - lv) > max_lv_var_) {
			if(lv < prev_lv)
					lv = prev_lv - max_lv_var_;
				else
					lv = prev_lv + max_lv_var_;
		} 
		// angular vel
		if(fabs(prev_av - av) > max_av_var_) {
			if(av < prev_av)
					av = prev_av - max_av_var_;
				else
					av = prev_av + max_av_var_;
		} 
		
		if(lv > space_->getMaxLinVel())
			lv = space_->getMaxLinVel();
		else if(lv < space_->getMinLinVel())
			lv = space_->getMinLinVel();
		
		if(av > space_->getMaxAngVel())
			av = space_->getMaxAngVel();
		else if(av < (-space_->getMaxAngVel()))
			av = space_->getMaxAngVel()*(-1);
			
		//Dead areas
		/*if(fabs(lv) < 0.08)
			lv = 0.0;
		if(fabs(av) < 0.05)
			av = 0.0;
		*/
		
		State* st = propagateStep(&currentState, lv, av);
		//Check if the state is valid
		if(!space_->isStateValid(st))  {
			motCost = 0.0;
			delete st;
			return false;
		}
		
		Node nextNode(*st);
		nextNode.setCost(space_->getCost(st));
		float mc = motionCost(&currentNode, &nextNode);
		incCost += mc;
		currentState = *st;
		delete st;
		currentNode = nextNode;
		
		numSteps++;
		
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
		
		inter_states.push_back(currentState);
		Action a(lv, 0.0, av, 1);
		actions.push_back(a);
		
		prev_lv = lv;
		prev_av = av;
		
	}
	if(numSteps == 0){
		return false;
	}
	
	
	acts = actions;
	motCost = incCost;
	istates = inter_states;
	
	return true;
	
}









float RRT::Steering::motionCost(Node* n1, Node* n2)
{
	
	switch(motionCostType_)
	{
		case 1:
			// avg_cost
			return ((n1->getCost() + n2->getCost()) / 2.0);
			
		case 2:
			// avg_cost * ecuclidean_dist
			return (((n1->getCost() + n2->getCost()) / 2.0) * distance(n1->getState(), n2->getState(), 2)); 
		
		case 3:
			// avg_cost * exp(dist)
			return (((n1->getCost() + n2->getCost()) / 2.0) * exp(distance(n1->getState(), n2->getState(), 2))); 
				
		case 4:
			// cost sum 
			return (n1->getCost() + n2->getCost());
			
		case 5:
			//avg_cost * (dist + orientation)
			return ((n1->getCost() + n2->getCost()) / 2.0) * distance(n1->getState(), n2->getState(), 4); 
		
		case 6:
			// avg_cost * (dist + angles_accu_diff)  (angles of the point orientation regarding the intersection line)
			return ((n1->getCost() + n2->getCost()) / 2.0) * distance(n1->getState(), n2->getState(), 5); 
			
		case 7:
			// avg_cost * distance function of paper IROS2015 "Feedback motion planning via non-holonomic RRT* for mobile robots"
			return ((n1->getCost() + n2->getCost()) / 2.0) * distance(n1->getState(), n2->getState(), 6); 
		
		default:
			// avg * ecuclidean_dist
			return (((n1->getCost() + n2->getCost()) / 2.0) * distance(n1->getState(), n2->getState(), 2)); 
	}
}




float RRT::Steering::distance(State* s1, State* s2, int type)
{
	float dx = s1->getX() - s2->getX();
	float dy = s1->getY() - s2->getY();
	float dz = s1->getZ() - s2->getZ();
	//float dist = sqrt(dx*dx + dy*dy);
	float dist = dx*dx + dy*dy + dz*dz;
	
	switch(type) {
		
		case 1:
			return dist;

		case 2:
			return sqrt(dist);

		/*case 3:
			if(space_->getDimensions() == 2)
				return sqrt(dist);
			else {
				// w1*|| Pi+1 - Pi|| + w2*(1-|Qi+1 * Qi|)²
				float euc_dist = sqrt(dist);
		
				tf::Quaternion q1 = tf::createQuaternionFromYaw(s1->getYaw());
				tf::Quaternion q2 = tf::createQuaternionFromYaw(s2->getYaw());
				float dot_prod = q1.dot(q2);
				float angle_dist =  (1 - fabs(dot_prod))*(1 - fabs(dot_prod));
				//printf("eu_dist: %.2f, angle_dist: %.3f, dist: %.3f\n", euc_dist, angle_dist, 0.8*euc_dist + 0.2*angle_dist);
				return 0.7*euc_dist + 0.3*angle_dist;
			}*/
			
		case 4:
			if(space_->getDimensions() == 2)
				return dist;
			else {
				// Another option
				/*
				First, transform the robot location into person location frame: 
											|cos(th)  sin(th)  0|
					Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
											|  0        0      1|
												 
					x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
					y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
				*/
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				return (0.8*sqrt(dist)+0.2*fabs(alpha));
			}
			
		case 5:  
			if(space_->getDimensions() == 2)
				return dist;
			else {
				//UPO. Dist + sum of the angles of both points regarding the intersection line
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float beta = s2->getYaw() - alpha;
				beta = normalizeAngle(beta, -M_PI, M_PI);
				return (0.6*sqrt(dist)+0.4*(fabs(alpha)+fabs(beta)));
			}
			
		case 6:  
			if(space_->getDimensions() == 2)
				return dist;
			else {
				//Paper IROS2015 "Feedback motion planning via non-holonomic RRT* for mobile robots"
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float phi = s2->getYaw() - alpha;
				phi = normalizeAngle(phi, -M_PI, M_PI);
				float ka = 0.5;
				float ko = ka/8.0;
				dist = sqrt(dist);
				// two options
				float alpha_prime = atan(-ko*phi);
				//float alpha_prime = atan(-ko*ko * phi/(dist*dist));
				float r = normalizeAngle((alpha-alpha_prime), -M_PI, M_PI);
				return (sqrt(dist*dist + ko*ko + phi*phi) + ka*fabs(r));
			}
			
		default:
			return dist;
	}

}



// Steering method in 3 dimensions (x, y, yaw) - Interpolation of the motion cost
bool RRT::Steering::steer3(Node* fromNode, Node* toNode, Node* newNode)
{
	//Node * newNode = NULL;
	//Node * prevNode = NULL;
	std::vector<Action> actions;
	std::vector<State> istates;
	
	if(fromNode == NULL) {
		printf("Steering. ERROR. Initial node is NULL\n"); 
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. ERROR. Final node is NULL\n"); 
		return false;
	}	
	
	
	//Max velocities variations in one time step
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	
	float incCost = 0.0;
	float AccCost = fromNode->getAccCost();
	
	std::vector<Action>* ac = fromNode->getAction();
	float prev_lv = ac->at(ac->size()-1).getVx(); 
	float prev_av = ac->at(ac->size()-1).getVth();
	//float prev_lv = fromNode->getState()->getLinVel();
	//float prev_av = fromNode->getState()->getAngVel();
	
	float prev_x = fromNode->getState()->getX();
	float prev_y = fromNode->getState()->getY();
	
	/*if(prev_x == 0.0 && prev_y ==0.0){
		printf("a_lv: %.2f, st_lv:%.2f\n", prev_lvA, prev_lv);
	}*/
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	
	float lv = 0.0, av=0.0;
	
	float phi = 0.0;
	float dist = 100.0;
	int numSteps = 0;
	
	Node currentNode(*fromNode);
	State currentState = *currentNode.getState();
	//Node currentNode(currentState);
	currentNode.setCost(space_->getCost(currentNode.getState()));
	
	bool first = true;
	
	while(numSteps <= maxControlSteps_ && (dist > space_->getGoalXYZTolerance() || fabs(phi) > space_->getGoalTHTolerance()))
	{
		
		// Transform way-point into local robot frame and get desired x,y,theta
		float dx = (wx-currentState.getX())*cos(currentState.getYaw()) + (wy-currentState.getY())*sin(currentState.getYaw());
		float dy =-(wx-currentState.getX())*sin(currentState.getYaw()) + (wy-currentState.getY())*cos(currentState.getYaw());
		if(numSteps == 0) {
			dist = sqrt(dx*dx + dy*dy);  
			phi = currentState.getYaw() - wth;
			//Normalize phi
			phi = space_->normalizeAngle(phi, -M_PI, M_PI);
		}

		float alpha = atan2(dy, dx);
		
		if(fabs(alpha) > 1.0) // 1.5r~86º  0.7~41º  0.79~45º
		{
			lv = 0.0;
			av = 0.3;
			if(alpha < 0.0)
				av = -0.3;
	
		} else {
		
			//Astolfi
			//float lv = kp * dist;
			//float av = ka * alpha; // + ko * phi;
			
			if(steeringType_ == 1) {
				//POSQ
				lv = kp_ * tanh(kv_*dist);
				av = ka_ * alpha + ko_ * phi;
			} else {
				//Improved-POSQ
				lv = kp_ * tanh(kv_*dist) * exp(-fabs(alpha));
				av = ka_ * alpha + ko_ * phi;
			}
			
			
			float ant_lv = lv;
			
			//Check velocities reacheability
			// linear vel
			if(fabs(prev_lv - lv) > max_lv_var_) {
				if(lv < prev_lv)
						lv = prev_lv - max_lv_var_;
					else
						lv = prev_lv + max_lv_var_;
			} 
			// angular vel
			if(fabs(prev_av - av) > max_av_var_) {
				if(av < prev_av)
						av = prev_av - max_av_var_;
					else
						av = prev_av + max_av_var_;
			} 
			
			
			//Check max and min velocities
			if(lv > space_->getMaxLinVel())
				lv = space_->getMaxLinVel();
			else if(lv < space_->getMinLinVel())
				lv = space_->getMinLinVel();
			
			if(av > space_->getMaxAngVel())
				av = space_->getMaxAngVel();
			else if(av < (-space_->getMaxAngVel()))
				av = space_->getMaxAngVel()*(-1);
					
			
			/*if(first && prev_x == 0.0 && prev_y == 0.0) {
				first = false;
				printf("prev_lv: %.2f  n_lv:%.2f, n_lv2:%.2f\n", prev_lv, ant_lv, lv); 
			}*/
			
			//Dead velocity ranges
			/*if(fabs(lv) < 0.08)
				lv = 0.0;
			if(fabs(av) < 0.05)
				av = 0.0;
			*/
		}
		
		//Propagate the movement
		State* st = propagateStep(&currentState, lv, av);
		
		//Break the loop is the state is not valid
		if(!space_->isStateValid(st)) {
			delete st;
			break;
		}
		
		Node nextNode(*st);
		nextNode.setCost(space_->getCost(st));
		float mc = motionCost(&currentNode, &nextNode);
		incCost += mc;
		currentState = *st;
		delete st;
		currentNode = nextNode;
		currentNode.setIncCost(incCost);
		
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
		phi = currentState.getYaw() - wth;
		//Normalize phi
		phi = space_->normalizeAngle(phi, -M_PI, M_PI);
		istates.push_back(currentState);
		Action a(lv, 0.0, av, 1);
		actions.push_back(a);
	
		prev_lv = lv;
		prev_av = av;
	
		numSteps++;
		
	}
	
	if(numSteps == 0){
		return false;
	}
	
	
	newNode->setState(currentState);
	newNode->setAction(actions);
	newNode->setIntermediateStates(istates);
	newNode->setCost(currentNode.getCost());
	newNode->setIncCost(incCost);
	newNode->setAccCost(AccCost + incCost);
	
	
	return true;
}





//Steering for collision checking in 3 dimensions (x, y, yaw) - Interpolation of the motion cost
bool RRT::Steering::collisionFree3(Node* fromNode, Node* toNode, std::vector<Action>& acts, std::vector<State>& istates, float& motCost)
{
	std::vector<Action> actions;
	std::vector<State> inter_states;
	
	if(fromNode == NULL) {
		printf("Steering. Nodo inicial igual a NULL\n"); 
		motCost = 0.0;
		return false;
	}
	if(toNode == NULL) {
		printf("Steering. Nodo final igual a NULL\n"); 
		motCost = 0.0;
		return false;
	}	
	
	
	float incCost = 0.0;
	//float AccCost = fromNode->getAccCost();
	
	//Max velocities variations in one time step
	max_lv_var_ = maxLinearAcc_ * timeStep_;
	max_av_var_ = maxAngularAcc_ * timeStep_;
	
	//std::vector<Action*> act = fromNode->getAction();
	//float prev_lv = act.at(act.size()-1)->getVx();
	//float prev_av = act.at(act.size()-1)->getVth();
	float prev_lv = fromNode->getState()->getLinVel();
	float prev_av = fromNode->getState()->getAngVel();
	
	//waypoint to reach
	float wx = toNode->getState()->getX();
	float wy = toNode->getState()->getY();
	float wth = toNode->getState()->getYaw();
	
	
	float lv = 0.0, av=0.0;
	
	float phi = 0.0;
	float xs = wx - fromNode->getState()->getX();
	float ys = wy - fromNode->getState()->getY();
	float init_dist = sqrt(xs*xs + ys*ys);  //10.0;
	float dist = init_dist;
	
	float max_dist_step = space_->getMaxLinVel() * timeStep_;
	float approx_steps = init_dist/max_dist_step;
	
	
	//Initial robot position
	Node currentNode(*fromNode);
	State currentState = *currentNode.getState();
	
	int numSteps = 0;
	
	while (dist >= space_->getGoalXYZTolerance() || fabs(phi) > space_->getGoalTHTolerance())  
	{
		
		//Check that the path to waypoint is not too long
		if(numSteps > ceil(approx_steps*3)) { 
			//printf("Step number > %.1f\n", ceil(approx_steps*5));
			motCost = 0.0;	
			return false;
		}
		
		
		
		// Transform way-point into local robot frame and get desired x,y,theta
		float dx = (wx-currentState.getX())*cos(currentState.getYaw()) + (wy-currentState.getY())*sin(currentState.getYaw());
		float dy =-(wx-currentState.getX())*sin(currentState.getYaw()) + (wy-currentState.getY())*cos(currentState.getYaw());
		if(numSteps == 0) {
			dist = sqrt(dx*dx + dy*dy);  
			phi = currentState.getYaw() - wth;
			//Normalize phi
			phi = space_->normalizeAngle(phi, -M_PI, M_PI);
		} 

		float alpha = atan2(dy, dx);
		
		
		//Velocities to command
		if(steeringType_ == 1) {
			//POSQ
			lv = kp_ * tanh(kv_*dist);
			av = ka_ * alpha + ko_ * phi;
		} else {
			//Improved-POSQ
			lv = kp_ * tanh(kv_*dist) * exp(-fabs(alpha));
			av = ka_ * alpha + ko_ * phi;
		}
		
		
		//Check velocities reacheability
		
		// linear vel
		if(fabs(prev_lv - lv) > max_lv_var_) {
			if(lv < prev_lv)
					lv = prev_lv - max_lv_var_;
				else
					lv = prev_lv + max_lv_var_;
		} 
		// angular vel
		if(fabs(prev_av - av) > max_av_var_) {
			if(av < prev_av)
					av = prev_av - max_av_var_;
				else
					av = prev_av + max_av_var_;
		} 
		
		if(lv > space_->getMaxLinVel())
			lv = space_->getMaxLinVel();
		else if(lv < space_->getMinLinVel())
			lv = space_->getMinLinVel();
		
		if(av > space_->getMaxAngVel())
			av = space_->getMaxAngVel();
		else if(av < (-space_->getMaxAngVel()))
			av = space_->getMaxAngVel()*(-1);
			
		//Dead areas
		/*if(fabs(lv) < 0.08)
			lv = 0.0;
		if(fabs(av) < 0.05)
			av = 0.0;
		*/
		
		//---------------------------------
		State* st = propagateStep(&currentState, lv, av);
		//Check if the state is valid
		if(!space_->isStateValid(st))  {
			motCost = 0.0;
			delete st;
			return false;
		}
		
		Node nextNode(*st);
		nextNode.setCost(space_->getCost(st));
		float mc = motionCost(&currentNode, &nextNode);
		incCost += mc;
		currentState = *st;
		delete st;
		currentNode = nextNode;
		
		numSteps++;
		
		dist = sqrt((wx-currentState.getX())*(wx-currentState.getX()) + (wy-currentState.getY())*(wy-currentState.getY()));
		phi = currentState.getYaw() - wth;
		//Normalize phi
		phi = space_->normalizeAngle(phi, -M_PI, M_PI);
		
		inter_states.push_back(currentState);
		Action a(lv, 0.0, av, 1);
		actions.push_back(a);
		
		prev_lv = lv;
		prev_av = av;
		
	}
	
	if(numSteps == 0){
		return false;
	}
	
	acts = actions;
	motCost = incCost;
	istates = inter_states;
	
	return true;
	
}


//Propagate one step
RRT::State* RRT::Steering::propagateStep(State* st, float lv, float av)
{
	float lin_dist = lv * timeStep_;
	float ang_dist = av * timeStep_;
	float th = st->getYaw() + ang_dist;
	//Normalize th
	th = space_->normalizeAngle(th, -M_PI, M_PI);
	float x = st->getX() + lin_dist*cos(th);
	float y = st->getY() + lin_dist*sin(th); 
	float z = st->getZ();
	State* result = new State(x, y, z, th, lv, av);
	return result;
}



float RRT::Steering::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}




