using namespace std;

class Probability {

	private:
		double minProb;
		double avgProb;
		double maxProb;
		double minMean;
		double minVari;
		double avgMean;
		double avgVari;
		double maxMean;
		double maxVari;

	public:
		//constructor
		Probability();
		~Probability();

		void setProbability(Probability *p);
		double getMinProb();
		void setMinProb(double minp);
		double getAvgProb();
		void setAvgProb(double avgp);
		double getMaxProb();
		void setMaxProb(double maxp);
		double getMinMean();
		double getMinVari();
		void setMinMean(double minm);
		void setMinVari(double minv);
		double getAvgMean();
		double getAvgVari();
		void setAvgMean(double avgm);
		void setAvgVari(double avgv);
		double getMaxMean();
		double getMaxVari();
		void setMaxMean(double maxm);
		void setMaxVari(double maxv);
};
