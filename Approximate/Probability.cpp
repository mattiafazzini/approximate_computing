#include "Probability.h"

using namespace std;

//constructor
Probability::Probability(){
	minProb = 0;
	avgProb = 0;
	maxProb = 0;
	minMean = 0;
	minVari = 0;
	avgMean = 0;
	avgVari = 0;
	maxMean = 0;
	maxVari = 0;
}

Probability::~Probability(){
}

void Probability::setProbability(Probability *p){
	minProb = p->getMinProb();
	avgProb = p->getAvgProb();
	maxProb = p->getMaxProb();
	minMean = p->getMinMean();
	minVari = p->getMinVari();
	avgMean = p->getAvgMean();
	avgVari = p->getAvgVari();
	maxMean = p->getMaxMean();
	maxVari = p->getMaxVari();
}

double Probability::getMinProb(){
	return minProb;
}

void Probability::setMinProb(double minp){
	minProb = minp;
}

double Probability::getAvgProb(){
	return avgProb;
}

void Probability::setAvgProb(double avgp){
	avgProb = avgp;
}

double Probability::getMaxProb(){
	return maxProb;
}

void Probability::setMaxProb(double maxp){
	maxProb = maxp;
}

double Probability::getMinMean(){
	return minMean;
}

double Probability::getMinVari(){
	return minVari;
}

void Probability::setMinMean(double minm){
	minMean = minm;
}

void Probability::setMinVari(double minv){
	minVari = minv;
}

double Probability::getAvgMean(){
	return avgMean;
}

double Probability::getAvgVari(){
	return avgVari;
}

void Probability::setAvgMean(double avgm){
	avgMean = avgm;
}

void Probability::setAvgVari(double avgv){
	avgVari = avgv;
}

double Probability::getMaxMean(){
	return maxMean;
}

double Probability::getMaxVari(){
	return maxVari;
}

void Probability::setMaxMean(double maxm){
	maxMean = maxm;
}

void Probability::setMaxVari(double maxv){
	maxVari = maxv;
}

