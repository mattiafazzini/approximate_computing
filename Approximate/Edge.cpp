#include "Edge.h"

using namespace std;

//constructor
Edge::Edge(Instruction *s, Instruction *d, Probability *p){
	src=s;
	dst=d;
	prob=p;
}

Edge::~Edge(){
	delete prob;
}

Instruction* Edge::getSrc(){
	return src;
}

Instruction* Edge::getDst(){
	return dst;
}

Probability* Edge::getProb(){
	return prob;
}
