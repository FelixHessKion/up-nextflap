#ifndef METRIC_FF_H
#define METRIC_FF_H

#include "../utils/utils.h"
#include "../utils/priorityQueue.h"
#include "../sas/sasTask.h"
#include "../planner/state.h"

class METRIC_FF_Condition : public PriorityQueueItem {
public:
	TVariable var;
	TValue value;
    int level;
    METRIC_FF_Condition(TVariable v, TValue val, int l) {
		var = v;
		value = val;
		level = l;
	}
    inline int compare(PriorityQueueItem* other) {
        return ((METRIC_FF_Condition*)other)->level - level;
    }
    virtual ~METRIC_FF_Condition() { }
};

class METRIC_FF_VarValue {
public:
	TVariable var;
    TValue value;
    METRIC_FF_VarValue(TVariable var, TValue value);
};

class METRIC_FF {
private:
	SASTask* task;
	std::vector< std::vector<int> > literalLevels;
    std::vector<int> actionLevels;
    unsigned int numLevels;
    std::vector<METRIC_FF_VarValue>* lastLevel;
    std::vector<METRIC_FF_VarValue>* newLevel;
    std::vector<TVarValue> reachedValues;
    
    void initialize();
    void addEffects(SASAction* a);
    void addEffect(TVariable var, TValue value);
	void expand();
	void addSubgoals(std::vector<TVarValue>* goals, PriorityQueue* openConditions);
	void addSubgoal(TVariable var, TValue value, PriorityQueue* openConditions);
	void addSubgoals(SASAction* a, PriorityQueue* openConditions);
	uint16_t getDifficulty(SASAction* a);
	uint16_t getDifficulty(SASCondition* c);
	void addTILactions(std::vector<SASAction*>* tilActions);
	uint16_t computeHeuristic(PriorityQueue* openConditions);
	void resetReachedValues();
	bool isExecutable(SASAction* a);

public:
	std::vector<SASAction*> relaxedPlan;

    METRIC_FF(TState* fs, std::vector<SASAction*>* tilActions, SASTask* task);
	uint16_t evaluate();
};

#endif
