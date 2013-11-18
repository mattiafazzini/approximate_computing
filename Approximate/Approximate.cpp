#include "llvm/Pass.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Constants.h"
#include "llvm/Support/raw_ostream.h"
#include "Edge.h"

using namespace llvm;

namespace {
	struct Approximate : public FunctionPass {
		static char ID;
		std::map<int,Probability*> inst_prob_map;
		Approximate() : FunctionPass(ID) {}
	    virtual bool runOnFunction(Function &function) {
	    	//getting pointer to function
	    	Function *func=&function;
	    	//remove after algorithm is implemented
	    	if(!func->getName().equals("compute")){
	    		return false;
	    	}

	    	//creating vector of edges
	    	std::vector<Edge*> *edges_vector = new std::vector<Edge*>;

	    	//iterating over basic blocks
	    	for (Function::iterator block_it = func->begin(), block_end = func->end(); block_it != block_end; ++block_it){
	    		BasicBlock *bb = block_it;
	    		//iterating over instructions
	    		for (BasicBlock::iterator inst_it = bb->begin(), inst_end = bb->end(); inst_it != inst_end; ++inst_it){
	    			Instruction *inst = inst_it;
					//errs() << "Type:" << inst->getOpcode() << "\n";
					//errs() << "Inst:" << *inst << "\n";
	    			//iterating over uses of def for this instruction
	    			for(Value::use_iterator use_it = inst->use_begin(), use_end = inst->use_end(); use_it != use_end; ++use_it){
	    				User *user = *use_it;
	    				if (Instruction *curr_inst = dyn_cast<Instruction>(user)) {
	    					//adding to vector
	    					edges_vector->push_back(new Edge(inst, curr_inst, new Probability()));
	    				}
	    			}
	    		}
	    	}


	    	//algorithm

	    	//iterating over basic blocks
	    	for (Function::iterator block_it = func->begin(), block_end = func->end(); block_it != block_end; ++block_it){
	    		BasicBlock *bb = block_it;
	    		//iterating over instructions
	    		for (BasicBlock::iterator inst_it = bb->begin(), inst_end = bb->end(); inst_it != inst_end; ++inst_it){
	    			Instruction *inst = inst_it;
    				if(inst->getOpcode()==9){
    					//handle add inst
    					handleFaddInst(edges_vector, inst);
    				}
    				if(inst->getOpcode()==13){
    					//handle fmul inst
    					handleFmulInst(edges_vector, inst);
    				}
	    		}
	    	}


	    	//printing results
	    	errs() << "FUNC:" << func->getName() << "\n";
	    	for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
	    		Edge *edge = *it;
	    		errs() << "EDGE:" << *(edge->getSrc()) << " ->" << *(edge->getDst()) << " # " << "\n"
	    				<< "minProb:" << edge->getProb()->getMinProb() << "\n"
	    				<< "minMean:" << edge->getProb()->getMinMean() << "\n"
	    				<< "minVari:" << edge->getProb()->getMinVari() << "\n"
	    				<< "avgProb:" << edge->getProb()->getAvgProb() << "\n"
	    				<< "avgMean:" << edge->getProb()->getAvgMean() << "\n"
	    				<< "avgVari:" << edge->getProb()->getAvgVari() << "\n"
	    				<< "maxProb:" << edge->getProb()->getMaxProb() << "\n"
	    				<< "maxMean:" << edge->getProb()->getMaxMean() << "\n"
	    				<< "maxVari:" << edge->getProb()->getMaxVari() << "\n";
	    		delete edge;
	    	}
	    	delete edges_vector;

	    	return false;
	    }

	    void handleFaddInst(std::vector<Edge*> *edges_vector, Instruction *inst){
	    	if(inst->getNumOperands()!=2){
	    		errs() << "Can not handle fadd instruction with unusual number of operands\n";
	    		return;
	    	}

	    	Probability *first = new Probability();
	    	Probability *second = new Probability();
	    	Probability *operation = new Probability();

	    	//getting probability of first operand
	    	Value *first_operand = inst->getOperand(0);
	    	if (isa<Instruction>(first_operand)){

	    		bool found = false;
	    		//get the probability of the first edge the contains first_operand as source and inst as dst
	    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
	    			Edge *edge = *it;
	    			if(edge->getSrc()==first_operand && edge->getDst()==inst){
	    				found=true;
	    				first->setProbability(edge->getProb());
	    				break;
	    			}
	    		}
	    		if(!found){
	    			errs() << "Did not find first operand probability for fadd\n";
	    			return;
	    		}
	    	}

	    	//getting probability of first operand
	    	Value *second_operand = inst->getOperand(1);
	    	if (isa<Instruction>(second_operand)){

	    		bool found = false;
	    		//get the probability of the first edge the contains first_operand as source and inst as dst
	    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
	    			Edge *edge = *it;
	    			if(edge->getSrc()==second_operand && edge->getDst()==inst){
	    				found=true;
	    				second->setProbability(edge->getProb());
	    				break;
	    			}
	    		}
	    		if(!found){
	    			errs() << "Did not find second operand probability for fadd\n";
	    			return;
	    		}
	    	}

	    	//getting probability of operation
	    	std::map<int, Probability*>::iterator map_it = inst_prob_map.find(inst->getOpcode());
	    	if(inst_prob_map.end() != map_it) {
	    		//probability of operator inside operator map
	    		operation->setProbability(map_it->second);
	    	}

	    	//new probabilities computation
	    	Probability *result = computeProbabilityFadd(first, second, operation);

	    	//set result in each of the edges in which inst is src
    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
    			Edge *edge = *it;
    			if(edge->getSrc()==inst){
    				edge->getProb()->setProbability(result);
    			}
    		}


	    	//iterate over all edges that have this inst as src and set the computed probability
	    	delete first;
	    	delete second;
	    	delete operation;
	    	delete result;
	    }

	    //computeProbabilityFmul(first, second, operation, first_min, first_avg, first_max, second_min, second_avg, second_max);
	    Probability* computeProbabilityFadd(Probability *first, Probability *second, Probability *operation){
	    	Probability *result = new Probability();

	    	//min computation, at least one error and error is greater than zero
	    	std::vector<double> min_probabilities;
	    	std::vector<double> min_means;
	    	std::vector<double> min_variances;

	    	//000
	    	double min_probability_000 = (1-first->getMinProb())*(1-second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_000);
	    	double min_mean_000 = 0;
	    	min_means.push_back(min_mean_000);
	    	double min_variance_000 = 0;
	    	min_variances.push_back(min_variance_000);
	    	//001
	    	double min_probability_001 = (1-first->getMinProb())*(1-second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_001);
	    	double min_mean_001 = operation->getMinMean();
	    	min_means.push_back(min_mean_001);
	    	double min_variance_001 = operation->getMinVari();
	    	min_variances.push_back(min_variance_001);
	    	//010
	    	double min_probability_010 = (1-first->getMinProb())*(second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_010);
	    	double min_mean_010 = second->getMinMean();
	    	min_means.push_back(min_mean_010);
	    	double min_variance_010 = second->getMinVari();
	    	min_variances.push_back(min_variance_010);
	    	//011
	    	double min_probability_011 = (1-first->getMinProb())*(second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_011);
	    	double min_mean_011 = second->getMinMean()+operation->getMinMean();
	    	min_means.push_back(min_mean_011);
	    	double min_variance_011 = second->getMinVari()+operation->getMinVari();
	    	min_variances.push_back(min_variance_011);
	    	//100
	    	double min_probability_100 = (first->getMinProb())*(1-second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_100);
	    	double min_mean_100 = first->getMinMean();
	    	min_means.push_back(min_mean_100);
	    	double min_variance_100 = first->getMinVari();
	    	min_variances.push_back(min_variance_100);
	    	//101
	    	double min_probability_101 = (first->getMinProb())*(1-second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_101);
	    	double min_mean_101 = first->getMinMean()+operation->getMinMean();
	    	min_means.push_back(min_mean_101);
	    	double min_variance_101 = first->getMinVari()+operation->getMinVari();
	    	min_variances.push_back(min_variance_101);
	    	//110
	    	double min_probability_110 = (first->getMinProb())*(second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_110);
	    	double min_mean_110 = first->getMinMean()+second->getMinMean();
	    	min_means.push_back(min_mean_110);
	    	double min_variance_110 = first->getMinVari()+second->getMinVari();
	    	min_variances.push_back(min_variance_110);
	    	//111
	    	double min_probability_111 = (first->getMinProb())*(second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_111);
	    	double min_mean_111 = first->getMinMean()+second->getMinMean()+operation->getMinVari();
	    	min_means.push_back(min_mean_111);
	    	double min_variance_111 = first->getMinVari()+second->getMinVari()+operation->getMinVari();
	    	min_variances.push_back(min_variance_111);

	    	//getting min that is not zero
	    	int min_index=-1;
	    	double min_variance = std::numeric_limits<double>::max();
	    	for(unsigned int i=0; i<min_variances.size();i++){
	    		if(min_variances[i]<min_variance && min_variances[i]>0){
	    			min_index = i;
	    			min_variance = min_variances[i];
	    		}
	    	}

	    	double min_probability = min_probabilities[min_index];
	    	double min_mean = min_means[min_index];
	    	//setting result for min
	    	result->setMinProb(min_probability);
	    	result->setMinMean(min_mean);
	    	result->setMinVari(min_variance);


	    	//avg computation, variance
	    	std::vector<double> avg_probabilities;
	    	std::vector<double> avg_means;
	    	std::vector<double> avg_variances;
	    	std::vector<double> avg_differences;

	    	//000
	    	double avg_probability_000 = (1-first->getAvgProb())*(1-second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_000);
	    	double avg_mean_000 = 0;
	    	avg_means.push_back(avg_mean_000);
	    	double avg_variance_000 = 0;
	    	avg_variances.push_back(avg_variance_000);
	    	//001
	    	double avg_probability_001 = (1-first->getAvgProb())*(1-second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_001);
	    	double avg_mean_001 = operation->getAvgMean();
	    	avg_means.push_back(avg_mean_001);
	    	double avg_variance_001 = operation->getAvgVari();
	    	avg_variances.push_back(avg_variance_001);
	    	//010
	    	double avg_probability_010 = (1-first->getAvgProb())*(second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_010);
	    	double avg_mean_010 = second->getAvgMean();
	    	avg_means.push_back(avg_mean_010);
	    	double avg_variance_010 = second->getAvgVari();
	    	avg_variances.push_back(avg_variance_010);
	    	//011
	    	double avg_probability_011 = (1-first->getAvgProb())*(second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_011);
	    	double avg_mean_011 = second->getAvgMean()+operation->getAvgMean();
	    	avg_means.push_back(avg_mean_011);
	    	double avg_variance_011 = second->getAvgVari()+operation->getAvgVari();
	    	avg_variances.push_back(avg_variance_011);
	    	//100
	    	double avg_probability_100 = (first->getAvgProb())*(1-second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_100);
	    	double avg_mean_100 = first->getAvgMean();
	    	avg_means.push_back(avg_mean_100);
	    	double avg_variance_100 = first->getAvgVari();
	    	avg_variances.push_back(avg_variance_100);
	    	//101
	    	double avg_probability_101 = (first->getAvgProb())*(1-second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_101);
	    	double avg_mean_101 = first->getAvgMean()+operation->getAvgMean();
	    	avg_means.push_back(avg_mean_101);
	    	double avg_variance_101 = first->getAvgVari()+operation->getAvgVari();
	    	avg_variances.push_back(avg_variance_101);
	    	//110
	    	double avg_probability_110 = (first->getAvgProb())*(second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_110);
	    	double avg_mean_110 = first->getAvgMean()+second->getAvgMean();
	    	avg_means.push_back(avg_mean_110);
	    	double avg_variance_110 = first->getAvgVari()+second->getAvgVari();
	    	avg_variances.push_back(avg_variance_110);
	    	//111
	    	double avg_probability_111 = (first->getAvgProb())*(second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_111);
	    	double avg_mean_111 = first->getAvgMean()+second->getAvgMean()+operation->getAvgMean();
	    	avg_means.push_back(avg_mean_111);
	    	double avg_variance_111 = first->getAvgVari()+second->getAvgVari()+operation->getAvgVari();
	    	avg_variances.push_back(avg_variance_111);

	    	/*double expected_avg_mean =
	    			(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*second->getAvgMean()+
	    			(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*first->getAvgMean()+
	    			(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*operation->getAvgMean();*/

	    	double expected_avg_variace =
	    			(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*second->getAvgMean()+
	    			(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*first->getAvgMean()+
	    			(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*operation->getAvgMean();



	    	int avg_index=-1;
	    	double avg_difference= std::numeric_limits<double>::max();
	    	for(unsigned int i=0; i<avg_variances.size();i++){
	    		double difference = expected_avg_variace-avg_variances[i];
	    		if(difference<0){
	    			difference=-difference;
	    		}

	    		if(difference<avg_difference && avg_variances[i]>0){
	    			avg_index = i;
	    			avg_difference = difference;
	    		}
	    	}

	    	double avg_probability = avg_probabilities[avg_index];
	    	double avg_mean = avg_means[avg_index];
	    	double avg_variance = avg_variances[avg_index];
	    	result->setAvgProb(avg_probability);
	    	result->setAvgMean(avg_mean);
	    	result->setAvgVari(avg_variance);

	    	//max computation, biggest variance
	    	std::vector<double> max_probabilities;
	    	std::vector<double> max_means;
	    	std::vector<double> max_variances;

	    	//000
	    	double max_probability_000 = (1-first->getMaxProb())*(1-second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_000);
	    	double max_mean_000 = 0;
	    	max_means.push_back(max_mean_000);
	    	double max_variance_000 = 0;
	    	max_variances.push_back(max_variance_000);
	    	//001
	    	double max_probability_001 = (1-first->getMaxProb())*(1-second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_001);
	    	double max_mean_001 = operation->getMaxMean();
	    	max_means.push_back(max_mean_001);
	    	double max_variance_001 = operation->getMaxVari();
	    	max_variances.push_back(max_variance_001);
	    	//010
	    	double max_probability_010 = (1-first->getMaxProb())*(second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_010);
	    	double max_mean_010 = second->getMaxMean();
	    	max_means.push_back(max_mean_010);
	    	double max_variance_010 = second->getMaxVari();
	    	max_variances.push_back(max_variance_010);
	    	//011
	    	double max_probability_011 = (1-first->getMaxProb())*(second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_011);
	    	double max_mean_011 = second->getMaxMean()+operation->getMaxMean();
	    	max_means.push_back(max_mean_011);
	    	double max_variance_011 = second->getMaxVari()+operation->getMaxVari();
	    	max_variances.push_back(max_variance_011);
	    	//100
	    	double max_probability_100 = (first->getMaxProb())*(1-second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_100);
	    	double max_mean_100 = first->getMaxMean();
	    	max_means.push_back(max_mean_100);
	    	double max_variance_100 = first->getMaxVari();
	    	max_variances.push_back(max_variance_100);
	    	//101
	    	double max_probability_101 = (first->getMaxProb())*(1-second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_101);
	    	double max_mean_101 = first->getMaxMean()+operation->getMaxMean();
	    	max_means.push_back(max_mean_101);
	    	double max_variance_101 = first->getMaxVari()+operation->getMaxVari();
	    	max_variances.push_back(max_variance_101);
	    	//110
	    	double max_probability_110 = (first->getMaxProb())*(second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_110);
	    	double max_mean_110 = first->getMaxMean()+second->getMaxMean();
	    	max_means.push_back(max_mean_110);
	    	double max_variance_110 = first->getMaxVari()+second->getMaxVari();
	    	max_variances.push_back(max_variance_110);
	    	//111
	    	double max_probability_111 = (first->getMaxProb())*(second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_111);
	    	double max_mean_111 = first->getMaxMean()+second->getMaxMean()+operation->getMaxMean();
	    	max_means.push_back(max_mean_111);
	    	double max_variance_111 = first->getMaxVari()+second->getMaxVari()+operation->getMaxVari();
	    	max_variances.push_back(max_variance_111);

	    	//getting max
	    	int max_index=-1;
	    	double max_variance = 0;
	    	for(unsigned int i=0; i<max_variances.size();i++){
	    		if(max_variances[i]>max_variance){
	    			max_index = i;
	    			max_variance = max_variances[i];
	    		}
	    	}

	    	double max_probability = max_probabilities[max_index];
	    	double max_mean = max_means[max_index];
	    	//setting result for max
	    	result->setMaxProb(max_probability);
	    	result->setMaxMean(max_mean);
	    	result->setMaxVari(max_variance);

	    	return result;
	    }

	    void handleFmulInst(std::vector<Edge*> *edges_vector, Instruction *inst){
	    	if(inst->getNumOperands()!=2){
	    		errs() << "Can not handle fmul instruction with unusual number of operands\n";
	    		return;
	    	}

	    	Probability *first = new Probability();
	    	double first_min = 1;
	    	double first_avg = 1;
	    	double first_max = 1;
	    	Probability *second = new Probability();
	    	double second_min = 1;
	    	double second_avg = 1;
	    	double second_max = 1;
	    	Probability *operation = new Probability();

	    	//getting probability of first operand
	    	Value *first_operand = inst->getOperand(0);
	    	if (isa<Instruction>(first_operand)){
	    		//TODO change, should not be hard coded
		    	first_min = 0;
		    	first_avg = 0.5;
		    	first_max = 1;

	    		bool found = false;
	    		//get the probability of the first edge the contains first_operand as source and inst as dst
	    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
	    			Edge *edge = *it;
	    			if(edge->getSrc()==first_operand && edge->getDst()==inst){
	    				found=true;
	    				first->setProbability(edge->getProb());
	    				break;
	    			}
	    		}
	    		if(!found){
	    			errs() << "Did not find first operand probability for fmul\n";
	    			return;
	    		}
	    	}
	    	else{
	    		//get constant value
	    		if(isa<ConstantFP>(first_operand)){
	    			ConstantFP *operand = (ConstantFP*)first_operand;
	    			double constant_value = operand->getValueAPF().convertToDouble();
	    			first_min = constant_value;
	    			first_avg = constant_value;
	    			first_max = constant_value;

	    		}

	    		if(isa<ConstantInt>(first_operand)){
	    			ConstantInt *operand = (ConstantInt*)first_operand;
	    			int constant_value = (int) operand->getSExtValue();
	    			first_min = (double) constant_value;
	    			first_avg = (double) constant_value;
	    			first_max = (double) constant_value;
	    		}
	    	}

	    	//getting probability of first operand
	    	Value *second_operand = inst->getOperand(1);
	    	if (isa<Instruction>(second_operand)){
	    		//TODO change, should not be hard coded
		    	second_min = 0;
		    	second_avg = 0.5;
		    	second_max = 1;

	    		bool found = false;
	    		//get the probability of the first edge the contains first_operand as source and inst as dst
	    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
	    			Edge *edge = *it;
	    			if(edge->getSrc()==second_operand && edge->getDst()==inst){
	    				found=true;
	    				second->setProbability(edge->getProb());
	    				break;
	    			}
	    		}
	    		if(!found){
	    			errs() << "Did not find second operand probability for fmul\n";
	    			return;
	    		}
	    	}
	    	else{
	    		//get constant value
	    		if(isa<ConstantFP>(second_operand)){
	    			ConstantFP *operand = (ConstantFP*)second_operand;
	    			double constant_value = operand->getValueAPF().convertToDouble();
	    			second_min = constant_value;
	    			second_avg = constant_value;
	    			second_max = constant_value;
	    		}

	    		if(isa<ConstantInt>(second_operand)){
	    			ConstantInt *operand = (ConstantInt*)second_operand;
	    			int constant_value = (int) operand->getSExtValue();
	    			second_min = (double) constant_value;
	    			second_avg = (double) constant_value;
	    			second_max = (double) constant_value;
	    		}
	    	}

	    	//getting probability of operation
	    	std::map<int, Probability*>::iterator map_it = inst_prob_map.find(inst->getOpcode());
	    	if(inst_prob_map.end() != map_it) {
	    		//probability of operator inside operator map
	    		operation->setProbability(map_it->second);
	    	}

	    	//new probabilities computation
	    	Probability *result = computeProbabilityFmul(first, second, operation, first_min, first_avg, first_max, second_min, second_avg, second_max);

	    	//set result in each of the edges in which inst is src
    		for(std::vector<Edge*>::iterator it = edges_vector->begin(); it != edges_vector->end(); ++it){
    			Edge *edge = *it;
    			if(edge->getSrc()==inst){
    				edge->getProb()->setProbability(result);
    			}
    		}


	    	//iterate over all edges that have this inst as src and set the computed probability
	    	delete first;
	    	delete second;
	    	delete operation;
	    	delete result;
	    }

	    //computeProbabilityFmul(first, second, operation, first_min, first_avg, first_max, second_min, second_avg, second_max);
	    Probability* computeProbabilityFmul(Probability *first, Probability *second, Probability *operation, double first_min, double first_avg, double first_max, double second_min, double second_avg, double second_max){
	    	Probability *result = new Probability();

	    	//min computation, at least one error and error is greater than zero
	    	std::vector<double> min_probabilities;
	    	std::vector<double> min_means;
	    	std::vector<double> min_variances;

	    	//000
	    	double min_probability_000 = (1-first->getMinProb())*(1-second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_000);
	    	double min_mean_000 = 0;
	    	min_means.push_back(min_mean_000);
	    	double min_variance_000 = 0;
	    	min_variances.push_back(min_variance_000);
	    	//001
	    	double min_probability_001 = (1-first->getMinProb())*(1-second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_001);
	    	double min_mean_001 = operation->getMinMean();
	    	min_means.push_back(min_mean_001);
	    	double min_variance_001 = operation->getMinVari();
	    	min_variances.push_back(min_variance_001);
	    	//010
	    	double min_probability_010 = (1-first->getMinProb())*(second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_010);
	    	double min_mean_010 = first_min*second->getMinMean();
	    	min_means.push_back(min_mean_010);
	    	double min_variance_010 = (first_min*first_min*second->getMinVari());
	    	min_variances.push_back(min_variance_010);
	    	//011
	    	double min_probability_011 = (1-first->getMinProb())*(second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_011);
	    	double min_mean_011 = (first_min*second->getMinMean())+operation->getMinMean();
	    	min_means.push_back(min_mean_011);
	    	double min_variance_011 = (first_min*first_min*second->getMinVari())+(operation->getMinVari());
	    	min_variances.push_back(min_variance_011);
	    	//100
	    	double min_probability_100 = (first->getMinProb())*(1-second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_100);
	    	double min_mean_100 = second_min*first->getMinMean();
	    	min_means.push_back(min_mean_100);
	    	double min_variance_100 = (second_min*second_min*first->getMinVari());
	    	min_variances.push_back(min_variance_100);
	    	//101
	    	double min_probability_101 = (first->getMinProb())*(1-second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_101);
	    	double min_mean_101 = second_min*first->getMinMean()+operation->getMinMean();
	    	min_means.push_back(min_mean_101);
	    	double min_variance_101 = (second_min*second_min*first->getMinVari())+(operation->getMinVari());
	    	min_variances.push_back(min_variance_101);
	    	//110
	    	double min_probability_110 = (first->getMinProb())*(second->getMinProb())*(1-operation->getMinProb());
	    	min_probabilities.push_back(min_probability_110);
	    	double min_mean_110 = second_min*first->getMinMean()+first_min*second->getMinMean()+first->getMinMean()*second->getMinMean();
	    	min_means.push_back(min_mean_110);
	    	double min_variance_110 = (first_min*first_min*second->getMinVari())+(second_min*second_min*first->getMinVari())+(first->getMinMean()*first->getMinMean()*second->getMinVari())+(second->getMinMean()*second->getMinMean()*first->getMinVari())+(first->getMinVari()*second->getMinVari());
	    	min_variances.push_back(min_variance_110);
	    	//111
	    	double min_probability_111 = (first->getMinProb())*(second->getMinProb())*(operation->getMinProb());
	    	min_probabilities.push_back(min_probability_111);
	    	double min_mean_111 = (first_min*second->getMinMean())+(second_min*first->getMinMean())+(first->getMinMean()*second->getMinMean())+(operation->getMinMean());
	    	min_means.push_back(min_mean_111);
	    	double min_variance_111 = (first_min*first_min*second->getMinVari())+(second_min*second_min*first->getMinVari())+(first->getMinMean()*first->getMinMean()*second->getMinVari())+(second->getMinMean()*second->getMinMean()*first->getMinVari())+(first->getMinVari()*second->getMinVari())+(operation->getMinVari());
	    	min_variances.push_back(min_variance_111);

	    	//getting min that is not zero
	    	int min_index=-1;
	    	double min_variance = std::numeric_limits<double>::max();
	    	for(unsigned int i=0; i<min_variances.size();i++){
	    		if(min_variances[i]<min_variance && min_variances[i]>0){
	    			min_index = i;
	    			min_variance = min_variances[i];
	    		}
	    	}

	    	double min_probability = min_probabilities[min_index];
	    	double min_mean = min_means[min_index];
	    	//setting result for min
	    	result->setMinProb(min_probability);
	    	result->setMinMean(min_mean);
	    	result->setMinVari(min_variance);


	    	//avg computation, variance
	    	std::vector<double> avg_probabilities;
	    	std::vector<double> avg_means;
	    	std::vector<double> avg_variances;
	    	std::vector<double> avg_differences;

	    	//000
	    	double avg_probability_000 = (1-first->getAvgProb())*(1-second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_000);
	    	double avg_mean_000 = 0;
	    	avg_means.push_back(avg_mean_000);
	    	double avg_variance_000 = 0;
	    	avg_variances.push_back(avg_variance_000);
	    	//001
	    	double avg_probability_001 = (1-first->getAvgProb())*(1-second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_001);
	    	double avg_mean_001 = operation->getAvgMean();
	    	avg_means.push_back(avg_mean_001);
	    	double avg_variance_001 = operation->getAvgVari();
	    	avg_variances.push_back(avg_variance_001);
	    	//010
	    	double avg_probability_010 = (1-first->getAvgProb())*(second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_010);
	    	double avg_mean_010 = first_avg*second->getAvgMean();
	    	avg_means.push_back(avg_mean_010);
	    	double avg_variance_010 = (first_avg*first_avg*second->getAvgVari());
	    	avg_variances.push_back(avg_variance_010);
	    	//011
	    	double avg_probability_011 = (1-first->getAvgProb())*(second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_011);
	    	double avg_mean_011 = (first_avg*second->getAvgMean())+operation->getAvgMean();
	    	avg_means.push_back(avg_mean_011);
	    	double avg_variance_011 = (first_avg*first_avg*second->getAvgVari())+(operation->getAvgVari());
	    	avg_variances.push_back(avg_variance_011);
	    	//100
	    	double avg_probability_100 = (first->getAvgProb())*(1-second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_100);
	    	double avg_mean_100 = second_avg*first->getAvgMean();
	    	avg_means.push_back(avg_mean_100);
	    	double avg_variance_100 = (second_avg*second_avg*first->getAvgVari());
	    	avg_variances.push_back(avg_variance_100);
	    	//101
	    	double avg_probability_101 = (first->getAvgProb())*(1-second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_101);
	    	double avg_mean_101 = second_avg*first->getAvgMean()+operation->getAvgMean();
	    	avg_means.push_back(avg_mean_101);
	    	double avg_variance_101 = (second_avg*second_avg*first->getAvgVari())+(operation->getAvgVari());
	    	avg_variances.push_back(avg_variance_101);
	    	//110
	    	double avg_probability_110 = (first->getAvgProb())*(second->getAvgProb())*(1-operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_110);
	    	double avg_mean_110 = second_avg*first->getAvgMean()+first_avg*second->getAvgMean()+first->getAvgMean()*second->getAvgMean();
	    	avg_means.push_back(avg_mean_110);
	    	double avg_variance_110 = (first_avg*first_avg*second->getAvgVari())+(second_avg*second_avg*first->getAvgVari())+(first->getAvgMean()*first->getAvgMean()*second->getAvgVari())+(second->getAvgMean()*second->getAvgMean()*first->getAvgVari())+(first->getAvgVari()*second->getAvgVari());
	    	avg_variances.push_back(avg_variance_110);
	    	//111
	    	double avg_probability_111 = (first->getAvgProb())*(second->getAvgProb())*(operation->getAvgProb());
	    	avg_probabilities.push_back(avg_probability_111);
	    	double avg_mean_111 = (first_avg*second->getAvgMean())+(second_avg*first->getAvgMean())+(first->getAvgMean()*second->getAvgMean())+(operation->getAvgMean());
	    	avg_means.push_back(avg_mean_111);
	    	double avg_variance_111 = (first_avg*first_avg*second->getAvgVari())+(second_avg*second_avg*first->getAvgVari())+(first->getAvgMean()*first->getAvgMean()*second->getAvgVari())+(second->getAvgMean()*second->getAvgMean()*first->getAvgVari())+(first->getAvgVari()*second->getAvgVari())+(operation->getAvgVari());
	    	avg_variances.push_back(avg_variance_111);

	    	/*double expected_avg_mean =
	    			(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*first_avg*second->getAvgMean()+
	    			(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*second_avg*first->getAvgMean()+
	    			(avg_probability_110+avg_probability_111)*first->getAvgMean()*second->getAvgMean()+
	    			(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*operation->getAvgMean();*/

	    	double expected_avg_variace =
	    			(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*(avg_probability_010+avg_probability_011+avg_probability_110+avg_probability_111)*(first_avg)*(first_avg)*(second->getAvgVari())+
	    			(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*(avg_probability_100+avg_probability_101+avg_probability_110+avg_probability_111)*(second_avg)*(second_avg)*(first->getAvgVari())+
	    			(avg_probability_110+avg_probability_111)*(avg_probability_110+avg_probability_111)*(first->getAvgMean())*(first->getAvgMean())*(second->getAvgVari())+
	    			(avg_probability_110+avg_probability_111)*(avg_probability_110+avg_probability_111)*(second->getAvgMean())*(second->getAvgMean())*(first->getAvgVari())+
	    			(avg_probability_110+avg_probability_111)*(avg_probability_110+avg_probability_111)*(first->getAvgVari())*(second->getAvgVari())+
	    			(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*(avg_probability_001+avg_probability_011+avg_probability_101+avg_probability_111)*(operation->getAvgVari());



	    	int avg_index=-1;
	    	double avg_difference= std::numeric_limits<double>::max();
	    	for(unsigned int i=0; i<avg_variances.size();i++){
	    		double difference = expected_avg_variace-avg_variances[i];
	    		if(difference<0){
	    			difference=-difference;
	    		}

	    		if(difference<avg_difference && avg_variances[i]>0){
	    			avg_index = i;
	    			avg_difference = difference;
	    		}
	    	}

	    	double avg_probability = avg_probabilities[avg_index];
	    	double avg_mean = avg_means[avg_index];
	    	double avg_variance = avg_variances[avg_index];
	    	result->setAvgProb(avg_probability);
	    	result->setAvgMean(avg_mean);
	    	result->setAvgVari(avg_variance);

	    	//max computation, biggest variance
	    	std::vector<double> max_probabilities;
	    	std::vector<double> max_means;
	    	std::vector<double> max_variances;

	    	//000
	    	double max_probability_000 = (1-first->getMaxProb())*(1-second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_000);
	    	double max_mean_000 = 0;
	    	max_means.push_back(max_mean_000);
	    	double max_variance_000 = 0;
	    	max_variances.push_back(max_variance_000);
	    	//001
	    	double max_probability_001 = (1-first->getMaxProb())*(1-second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_001);
	    	double max_mean_001 = operation->getMaxMean();
	    	max_means.push_back(max_mean_001);
	    	double max_variance_001 = operation->getMaxVari();
	    	max_variances.push_back(max_variance_001);
	    	//010
	    	double max_probability_010 = (1-first->getMaxProb())*(second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_010);
	    	double max_mean_010 = first_max*second->getMaxMean();
	    	max_means.push_back(max_mean_010);
	    	double max_variance_010 = (first_max*first_max*second->getMaxVari());
	    	max_variances.push_back(max_variance_010);
	    	//011
	    	double max_probability_011 = (1-first->getMaxProb())*(second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_011);
	    	double max_mean_011 = (first_max*second->getMaxMean())+operation->getMaxMean();
	    	max_means.push_back(max_mean_011);
	    	double max_variance_011 = (first_max*first_max*second->getMaxVari())+(operation->getMaxVari());
	    	max_variances.push_back(max_variance_011);
	    	//100
	    	double max_probability_100 = (first->getMaxProb())*(1-second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_100);
	    	double max_mean_100 = second_max*first->getMaxMean();
	    	max_means.push_back(max_mean_100);
	    	double max_variance_100 = (second_max*second_max*first->getMaxVari());
	    	max_variances.push_back(max_variance_100);
	    	//101
	    	double max_probability_101 = (first->getMaxProb())*(1-second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_101);
	    	double max_mean_101 = second_max*first->getMaxMean()+operation->getMaxMean();
	    	max_means.push_back(max_mean_101);
	    	double max_variance_101 = (second_max*second_max*first->getMaxVari())+(operation->getMaxVari());
	    	max_variances.push_back(max_variance_101);
	    	//110
	    	double max_probability_110 = (first->getMaxProb())*(second->getMaxProb())*(1-operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_110);
	    	double max_mean_110 = second_max*first->getMaxMean()+first_max*second->getMaxMean()+first->getMaxMean()*second->getMaxMean();
	    	max_means.push_back(max_mean_110);
	    	double max_variance_110 = (first_max*first_max*second->getMaxVari())+(second_max*second_max*first->getMaxVari())+(first->getMaxMean()*first->getMaxMean()*second->getMaxVari())+(second->getMaxMean()*second->getMaxMean()*first->getMaxVari())+(first->getMaxVari()*second->getMaxVari());
	    	max_variances.push_back(max_variance_110);
	    	//111
	    	double max_probability_111 = (first->getMaxProb())*(second->getMaxProb())*(operation->getMaxProb());
	    	max_probabilities.push_back(max_probability_111);
	    	double max_mean_111 = (first_max*second->getMaxMean())+(second_max*first->getMaxMean())+(first->getMaxMean()*second->getMaxMean())+(operation->getMaxMean());
	    	max_means.push_back(max_mean_111);
	    	double max_variance_111 = (first_max*first_max*second->getMaxVari())+(second_max*second_max*first->getMaxVari())+(first->getMaxMean()*first->getMaxMean()*second->getMaxVari())+(second->getMaxMean()*second->getMaxMean()*first->getMaxVari())+(first->getMaxVari()*second->getMaxVari())+(operation->getMaxVari());
	    	max_variances.push_back(max_variance_111);

	    	//getting max
	    	int max_index=-1;
	    	double max_variance = 0;
	    	for(unsigned int i=0; i<max_variances.size();i++){
	    		if(max_variances[i]>max_variance){
	    			max_index = i;
	    			max_variance = max_variances[i];
	    		}
	    	}

	    	double max_probability = max_probabilities[max_index];
	    	double max_mean = max_means[max_index];
	    	//setting result for max
	    	result->setMaxProb(max_probability);
	    	result->setMaxMean(max_mean);
	    	result->setMaxVari(max_variance);

	    	return result;
	    }

	    virtual bool doInitialization(Module &module){
	    	Probability *fmul_prob = new Probability();
	    	fmul_prob->setMinProb(0.1);
	    	fmul_prob->setMinMean(0);
	    	fmul_prob->setMinVari(0.02);
	    	fmul_prob->setAvgProb(0.1);
	    	fmul_prob->setAvgMean(0);
	    	fmul_prob->setAvgVari(0.02);
	    	fmul_prob->setMaxProb(0.1);
	    	fmul_prob->setMaxMean(0);
	    	fmul_prob->setMaxVari(0.02);
	    	inst_prob_map.insert(std::pair<int,Probability*>(13, fmul_prob));
	    	Probability *add_prob = new Probability();
	    	add_prob->setMinProb(0.1);
	    	add_prob->setMinMean(0);
	    	add_prob->setMinVari(0.02);
	    	add_prob->setAvgProb(0.1);
	    	add_prob->setAvgMean(0);
	    	add_prob->setAvgVari(0.02);
	    	add_prob->setMaxProb(0.1);
	    	add_prob->setMaxMean(0);
	    	add_prob->setMaxVari(0.02);
	    	inst_prob_map.insert(std::pair<int,Probability*>(9, add_prob));
	    	return false;
	    }
	};
}

char Approximate::ID = 0;
static RegisterPass<Approximate> X("approximate", "Approximate Pass", true, true);
