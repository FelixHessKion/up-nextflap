cmake_minimum_required(VERSION 3.5)  
  
project(NextFLAP)  
  
# Set the C++ standard to use  
set(CMAKE_CXX_STANDARD 20)  

set(CMAKE_BUILD_TYPE Debug)
  
# Add source files  
set(SOURCES  
   nextflap/grounder/grounder.cpp  
   nextflap/grounder/groundedTask.cpp  
   nextflap/heuristics/evaluator.cpp  
   nextflap/heuristics/hFF.cpp  
   nextflap/heuristics/hLand.cpp  
   nextflap/heuristics/landmarks.cpp  
   nextflap/heuristics/numericRPG.cpp  
   nextflap/heuristics/rpg.cpp  
   nextflap/heuristics/temporalRPG.cpp  
   nextflap/parser/parsedTask.cpp  
   nextflap/parser/parser.cpp  
   nextflap/parser/syntaxAnalyzer.cpp  
   nextflap/planner/intervalCalculations.cpp  
   nextflap/planner/linearizer.cpp  
   nextflap/planner/plan.cpp  
   nextflap/planner/planBuilder.cpp  
   nextflap/planner/planComponents.cpp  
   nextflap/planner/planEffects.cpp  
   nextflap/planner/planner.cpp  
   nextflap/planner/plannerSetting.cpp  
   nextflap/planner/printPlan.cpp  
   nextflap/planner/selector.cpp  
   nextflap/planner/state.cpp  
   nextflap/planner/successors.cpp  
   nextflap/planner/z3Checker.cpp  
   nextflap/preprocess/preprocess.cpp  
   nextflap/preprocess/preprocessedTask.cpp  
   nextflap/sas/mutexGraph.cpp  
   nextflap/sas/sasTask.cpp  
   nextflap/sas/sasTranslator.cpp  
   nextflap/utils/utils.cpp  
)  
  
 # Find Z3 library    
find_library(Z3_LIBRARY z3)  
  
# If Z3 library is not found, print an error message  
if(NOT Z3_LIBRARY)  
    message(FATAL_ERROR "Z3 library not found.")  
endif()   

  
# Create the executable target  
add_library(${PROJECT_NAME} SHARED ${SOURCES})  

add_executable(nextflap_planner nextflap/nextflap.cpp
                               ${SOURCES})  
  
# Link necessary libraries  
target_link_libraries(nextflap_planner  ${Z3_LIBRARY})  
  
# Include necessary directories  
target_include_directories(${PROJECT_NAME} PRIVATE  
    ${Z3_INCLUDE_DIR}  
)  
