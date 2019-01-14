#ifndef SIMULATOR_BOARD_H
#define SIMULATOR_BOARD_H

#include <regex>
#include "Utils.h"
#include "ExprGenerator.h"
#include <iostream>
#include "Cell.h" // Not really necessary but easier to debug
#include "BoardObject.h"
#include <ostream>

// Definition of the simulation bord composing all cells and sources
struct Simulator
{
public:
	Simulator(
		const std::string rowExpr, std::string columnExpr,
		const int speedOnConduct/*, // Should we have different speeds ??? between 4 and 2
		const int speedWithoutConduct*/);
	
	// Used by events system to add / remove sources
	// Check the result if the operation was successful
	bool addSource(const TablePos& tablePos, const SourceInfo& source);
	bool modifySource(const TablePos& tablePos, const SourceInfo& source);
	bool removeSource(const TablePos& tablePos);

	// Simulates and outputs result to a file
	bool autoSimulate(const int numSteps, int minPower, int maxPower, const char* resultsFileName);
	void doStepByStepSimulation(const bool writeHelperOutput, std::istream& inStream, std::ostream& outStream);
	
	// Prints the board on screen
	void printBoard(std::ostream& outStream);

	// Save/Load board to file
	bool saveBoard(const char* fileNameToSave);

	void reorganize(); // Called to optimize the tree
	bool initialize_random(int maxDepth); // Called to initialize the tree with random nodes by the given specification
	bool initialize_fromFile(const char* fileToInitializeFrom);

	// Serial (deterministic) simulation of data flow
	void doDataFlowSimulation_serial(const bool withReconfiguration, std::ostream& output);

	void doUnitTests();
	void printBoard();

	void simulateOptimalReconfigurationScenarios(const int numAttempts, const char* filename);

	// Running for numScnearios * sampleTicks
	// ticksBetweenSourceEvent - self descriptive, a new source event will randomly spawn at this interval
	// ticksToReconfigureRoot - the number of ticks the root of the subtree being cut can't send further its flow
	void simulateOptimalVsRandomFlowScenario(const int numScenarios, const int sampleCount, const int sampleTicks, const int ticksBetweenSourceEvent, const int ticksToReconfigureRoot);


private:
	void gatherAllDistinctSymbols(std::vector<char>& symbols);

	// For evaluation purposes:
	// generate an optimal board and a random start board and check how far we can get to the optimal flow value after reconfigurations
	// After the initial step, if we modify the sources locations, how well does the reconfiguration behaves in time in comparison with the static version 
	void generateOptimalAndRandomBoard(BoardObject& outOptimalBoard, BoardObject& outRandomBoard);

	// Simulates a flow scenario considering re-configuration if allowed, sample count and each sample tick
	static void simulateFlowScenario(BoardObject& board, const float probForSourceEvent, const bool allowReconfiguration, const int sampleCount, const int sampleTicks, float& outAvgFlow);

	// Checks for a source event based on a given probability
	// Returns true in sourcesModified if an operation was performed
	static void checkSourceModifyEvent(BoardObject& board, const float probForSourceEvent, bool& sourcesModified);

	// Returns true if board is valid
	bool validateBoard()
	{
		// TODO
		return true;
	}

	void addBoardInHistory(const BoardObject* board);
	void undoBoard();

	BoardObject m_board; // The board at the current moment

	// When doing step by step simulation we store the history of the boards such that you can undo
	BoardObject m_history[SIMULATION_BOARD_HISTORY_SIZE];
	int m_historyIndex;

	Cell* m_root;
	std::string m_rowStrExpr;
	std::string m_colStrExpr;
	Expression_Generator m_rowGenerator;
	Expression_Generator m_colGenerator;

	int m_speedOnConduct;
	//int m_speedWithoutConduct;
};


#endif
