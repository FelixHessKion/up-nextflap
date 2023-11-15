

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Main method: parses the command-line arguments and   */
/* launches the planner.                                */
/********************************************************/
#include <iostream>
#include <string>
#include <sstream>

#include "utils/utils.h"
#include "parser/parsedTask.h"
#include "parser/parser.h"
#include "preprocess/preprocess.h"
#include "grounder/grounder.h"
#include "sas/sasTranslator.h"
#include "planner/plannerSetting.h"
#include "planner/z3Checker.h"
#include "planner/printPlan.h"

using namespace std;

// Parses the domain and problem files
ParsedTask *parseStage(char *domainFileName, char *problemFileName)
{
    clock_t t = clock();
    Parser parser;
    ParsedTask *parsedTask = parser.parseDomain(domainFileName);
    parser.parseProblem(problemFileName);
    float time = toSeconds(t);
    cout << ";Parsing time: " << time << endl;
    return parsedTask;
}

// Preprocesses the parsed task
PreprocessedTask *preprocessStage(ParsedTask *parsedTask)
{
    Preprocess preprocess;
    PreprocessedTask *prepTask = preprocess.preprocessTask(parsedTask);
    return prepTask;
}

// Grounder stage of the preprocessed task
GroundedTask *groundingStage(PreprocessedTask *prepTask)
{
    Grounder grounder;
    GroundedTask *gTask = grounder.groundTask(prepTask, false);
    if (gTask != nullptr && debugFile != nullptr)
        *debugFile << gTask->toString() << endl;
    return gTask;
}

// SAS translation stage
SASTask *sasTranslationStage(GroundedTask *gTask)
{
    SASTranslator translator;
    SASTask *sasTask = translator.translate(gTask, false, false, false);
    return sasTask;
}

// Sequential calls to the main planning stages
string startPlanning(SASTask *sTask, ParsedTask *parsedTask, bool durativePlan)
{
    PlannerSetting planner(sTask);
    Plan *solution;
    float bestMakespan = FLOAT_INFINITY;
    int bestNumSteps = MAX_UINT16;
    do {
        solution = planner.plan(bestMakespan, parsedTask);
        if (solution != nullptr) {
            Z3Checker checker;
            TControVarValues cvarValues;
            float solutionMakespan;
            if (checker.checkPlan(solution, true, &cvarValues)) {
                solutionMakespan = PrintPlan::getMakespan(solution);
                if (solutionMakespan < bestMakespan
                    || (abs(solutionMakespan - bestMakespan) < EPSILON
                        && solution->g < bestNumSteps)) {
                    bestMakespan = solutionMakespan;
                    bestNumSteps = solution->g;
                    return PrintPlan::print(solution, &cvarValues, durativePlan);
                }
            }
        }
    } while (solution != nullptr);
    return "No plan";
}

// Main method
int main(int argc, char *argv[])
{
    int param = 1;
    char *domainFileName = nullptr;
    char *problemFileName = nullptr;
    bool generateGroundedDomain = false;
    bool keepStaticData = false;
    bool noSAS = false;
    bool generateMutexFile = false;



    while (param < argc) {
        if (argv[param][0] != '-') {
            if (domainFileName == nullptr)
                domainFileName = argv[param];
            else if (problemFileName == nullptr)
                problemFileName = argv[param];
            else {
                domainFileName = nullptr;
                break;
            }
        } else {
            if (compareStr(argv[param], "-ground"))
                generateGroundedDomain = true;
            else if (compareStr(argv[param], "-static"))
                keepStaticData = true;
            else if (compareStr(argv[param], "-nsas"))
                noSAS = true;
            else if (compareStr(argv[param], "-mutex"))
                generateMutexFile = true;
            else {
                domainFileName = nullptr;
                break;
            }
        }
        param++;
    }
    if (domainFileName == nullptr || problemFileName == nullptr) {
    } else {
        ParsedTask *parsedTask = parseStage(domainFileName, problemFileName);
        PreprocessedTask *prepTask = preprocessStage(parsedTask);
        GroundedTask *gTask = groundingStage(prepTask);
        SASTask *sasTask = sasTranslationStage(gTask);
        string plan = startPlanning(sasTask, parsedTask, true);
        std::stringstream ss(plan);
        std::string token;

        // use getline to split the string by ':'
        while (std::getline(ss, token, ':')) {
            std::cout << token << ":" << std::endl;
        }
        }


    return 0;
}
