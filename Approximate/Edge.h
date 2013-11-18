#include "llvm/IR/Instruction.h"
#include "Probability.h"

using namespace llvm;

class Edge {

	private:
		Instruction *src;
		Instruction *dst;
		Probability *prob;

	public:
		//constructor
		Edge(Instruction *src, Instruction *dst, Probability *prob);
		~Edge();

		Instruction* getSrc();
		Instruction* getDst();
		Probability* getProb();
};
