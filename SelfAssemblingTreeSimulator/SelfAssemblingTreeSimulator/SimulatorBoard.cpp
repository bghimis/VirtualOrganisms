#include "SimulatorBoard.h"
#include <fstream>
#include <float.h>
#include <sstream>

using namespace std;

extern int g_simulationTicksForDataFlowEstimation;
extern int g_numSourcesOnRandomBoard;
extern int g_depthForAutoInitialization;
extern float g_energyLossThreshold;

extern bool g_verboseBestGatheredSolutions;
extern bool g_verboseLocalSolutions;
extern int g_minPowerForWirelessSource;
extern int g_maxPowerForWirelessSource;
extern int g_247eModelRootCol;
extern int g_247eModelRootRow;

extern int g_useEModel;
extern int FIXED_SEED;
std::vector<char> g_allSymbolsSet;
extern float g_costPerResource[];
extern std::ostream* g_debugLogOutput;

Simulator::Simulator(const std::string rowExpr, std::string columnExpr, const int speedOnConduct)
	: m_speedOnConduct(speedOnConduct)
	//, m_speedWithoutConduct(speedWithoutConduct)
	, m_rowStrExpr(rowExpr)
	, m_colStrExpr(columnExpr)
	, m_root(nullptr)
	, m_historyIndex(0)
{
	// Initialize the global language expression generators for rows and columns
	std::vector<char> symbols;
	m_rowGenerator.init(m_rowStrExpr, symbols);
	m_colGenerator.init(m_colStrExpr, symbols);

	gatherAllDistinctSymbols(symbols);
}

void Simulator::printBoard(std::ostream& outStream)
{
	m_board.printBoard(outStream);
}

bool Simulator::saveBoard(const char* fileNameToSave)
{
	ofstream outFile(fileNameToSave);

	if (!outFile.is_open())
	{
		assert(false && "The given input file can't be opened for writing !. Is it opened somewhere ??");
		return false;
	}

	for (uint i = 0; i < MAX_ROWS; i++)
	{
		for (uint j = 0; j < MAX_COLS; j++)
		{
			if (m_board(i, j).isFree())
			{
				outFile << BOARD_SKIP_CHARACTER;
			}
			else
			{
				outFile << m_board(i, j).m_symbol;
			}
		}

		outFile << std::endl;
	}

	outFile << m_board.m_posToSourceMap.size() << endl;
	for (auto& it : m_board.m_posToSourceMap)
	{
		const TablePos& pos = it.first;
		const SourceInfo& srcInfo = it.second;

		outFile << pos.row <<" " << pos.col <<" " << srcInfo.getPower() << endl;
	}
	outFile << endl;

	return true;
}

// Initialize from the given expressions
bool Simulator::initialize_random(int maxBranchDepth)
{
	m_board = BoardObject(); // Force initialize
	m_board.setRowAndColGenerators(&m_rowGenerator, &m_colGenerator);

	const bool res = m_board.generateRandomBoard(maxBranchDepth, g_numSourcesOnRandomBoard);

	m_root = m_board.getRootCell();

	m_board.setAvailableSymbols(g_allSymbolsSet);

	return res;
}

bool Simulator::initialize_fromFile(const char* fileToInitializeFrom)
{
	m_board.reset();

	ifstream inFile(fileToInitializeFrom);

	if (!inFile.is_open())
	{
		assert(false && "The given input file can't be opened for reading !");
		return false;
	}

	std::vector<char> symbols;
	m_rowGenerator.init(m_rowStrExpr, symbols);
	m_colGenerator.init(m_colStrExpr, symbols);
	gatherAllDistinctSymbols(symbols);

	// Read the root location if E model is used
	if (g_useEModel)
	{
		inFile >> g_247eModelRootRow >> g_247eModelRootCol;
	}

	// Step 1: read board and create the links
	char buffLine[MAX_COLS + 1];
	for (int i = 0; i < MAX_ROWS; i++)
	{
		for(int j = 0; j < MAX_COLS; j++)
		{		
			inFile >>buffLine[j];
			if (buffLine[j]=='\0' || buffLine[j] =='\r' || buffLine[j] == '\n')
			{
				j = -1;
				continue;
			}
		}

		buffLine[MAX_COLS] = 0;

		assert(strlen(buffLine) == MAX_COLS && "The size of the input is incorrect !");
		for (int j = 0; j < MAX_COLS; j++)
		{
			if (buffLine[j] == BOARD_SKIP_CHARACTER)
				continue;

			m_board(i, j).setSymbol(buffLine[j]);
		}
	}
	
	const int rootColumn = g_247eModelRootCol;
	const int rootRow = g_247eModelRootRow;

	m_board.setAvailableSymbols(g_allSymbolsSet);
	m_board.setRootLocation(rootRow, rootColumn);
	m_board.updateBoardAfterSymbolsInit();

	m_root = &m_board(rootRow, rootColumn);

	// Step 2: Populate the sources
	int numSources = 0;
	inFile >> numSources;
	for (int i = 0; i < numSources; i++)
	{
		TablePos pos;
		SourceInfo src;
		float srcPower = 0;
		inFile >> pos.row >> pos.col >> srcPower;
		src.overridePower(srcPower);
		addSource(pos, src);
	}

	m_root->initFlowStatistics(g_simulationTicksForDataFlowEstimation);

	return true;
}

bool Simulator::checkBoardLanguageConstraints() const
{
	TablePos firstWrongPos;
	if (!m_board.isCompliantWithRowColPatterns(INVALID_POS, INVALID_POS, &firstWrongPos))
	{
		(*g_debugLogOutput) << " !!!!!!!!!! Incorrect Row / COlumn patters identified !! !!!!!!!!!!!!" << std::endl;
		(*g_debugLogOutput) << "The first wrong pos is (" << firstWrongPos.row << "," << firstWrongPos.col <<")" << std::endl;

		return false;
	}

	return true;
}

void Simulator::reorganize()
{
	m_board.reorganize(*g_debugLogOutput);
}

/// Start Source management at simulation level
//----------------------------------------------------
// We don't need to propagate these events as messages, we just 'hack' the sources on local tables (shared memory - direct write, distributed - message)
// That's because Cells will subtract energy from where the source are at every moment
bool Simulator::addSource(const TablePos& tablePos, const SourceInfo& source)
{
	const bool res = m_board.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, tablePos, source, false); 
	//assert(res && "didn't succeed to add the source");
	return res;
}

bool Simulator::removeSource(const TablePos& tablePos)
{
	const bool res = m_board.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, tablePos, SourceInfo(), false);
	//assert(res && "didn't succeed to remove the source");
	return res;

}

bool Simulator::modifySource(const TablePos& tablePos, const SourceInfo& source)
{
	const bool res = m_board.propagateSourceEvent(Cell::EVENT_SOURCE_MODIFY, tablePos, source, false);
	assert(res && "didn't succeed to remove the source");
	return res;
}

//----------------------------------------------------

void Simulator::doStepByStepSimulation(const bool writeHelperOutput, std::istream& inStream, std::ostream& outStream)
{
	if (writeHelperOutput)
	{
		outStream << "Options:\nE - Data flow simulation to determine flow per unit of time \n";
		outStream << "O - Restructure then same as option E (to see restructure affects data flow\n";
		outStream << "Add/Remove/Modify source: S \n";
		outStream << "Reorganize: R\n";
		outStream << "Debug 24e7 model: 1 - Run GB 2-expand external 3-expand internal 4-optimize membrane G 5 - optimize by row/column cut  6 - optimize by corner cut\n";
		outStream << "Undo: U \n";
		outStream << "Save: V   |   Load: W\n";
		outStream << "Print current board: P\n";
		outStream << "Quit: Q \n";
	}

	int requestCounter = 0;

	while (true)
	{
		checkBoardLanguageConstraints();

		// Read an input
		char ev;

		if (inStream.eof())
			break;
		
		requestCounter++;
		if (writeHelperOutput)
		{
			outStream << "Your option: ";
		}
		else
		{
			outStream << endl << "##########################################################";
			outStream << endl << "Output for line " << requestCounter << " request below" << endl;
			outStream << "##########################################################" << endl;
		}

		inStream >> ev;

		switch (ev)
		{
			case 'E': // Nothing:
			case 'e':
			{
				doDataFlowSimulation_serial(false, outStream);
				outStream << endl << endl;
			}
			break;

			case 'O':
			case 'o':
			{
				doDataFlowSimulation_serial(true, outStream);
				addBoardInHistory(&m_board);
				m_root->onRootMsgReorganize();
				printBoard(outStream);
			}
			break;

			case 's':
			case 'S':
			{
				addBoardInHistory(&m_board);

				TablePos pos;
				if (writeHelperOutput) { outStream << "Source ROW = "; }
				inStream >> pos.row;

				if (writeHelperOutput) { outStream << "Source COL = "; }
				inStream >> pos.col;

				char innerEv = 'A';
				bool result = false;
				if (writeHelperOutput) { outStream << "event type: M for modify existing, A to add, R to remove: "; }
				inStream >> innerEv;

				switch (innerEv)
				{
					case 'R':
					case 'r':
					{
						result |= removeSource(pos);
					}
					break;

					case 'A':
					case 'a':
					{
						SourceInfo sourceInfo;
						if (writeHelperOutput) { outStream << "Source power = "; }
						float srcPower;
						inStream >> srcPower;  
						sourceInfo.overridePower(srcPower);

						result |= addSource(pos, sourceInfo);
					}
					break;

					case 'M':
					case 'm':
					{
						SourceInfo sourceInfo;
						if (writeHelperOutput) { outStream << "New Source power = "; }
						float srcPower;
						inStream >> srcPower;
						sourceInfo.overridePower(srcPower);

						result |= modifySource(pos, sourceInfo);
					}
					break;
				}
				
				outStream << (result ? "Your operation was done !" : "Your operation failed. Check the messages above") << endl;

				printBoard(outStream);
			}
			break;

			case 'Q':
			{
				outStream << "Exit! "; return;
			}
			break;

			case 'R':
			case 'r':
			{
				addBoardInHistory(&m_board);				
				m_board.reorganize(outStream);
				printBoard(outStream);
			}
			break;

			case '1':
			{
				addBoardInHistory(&m_board);

				m_board.runGarbageCollector(g_energyLossThreshold, outStream);

				printBoard(outStream);
			}
			break;
			case '2':
			{
				addBoardInHistory(&m_board);

				m_board.expandExternalTrees();
			}
			break;
			case '3':
			{
				addBoardInHistory(&m_board);

				m_board.expandInternalTrees();
			}
			break;
			case '4':
			{
				addBoardInHistory(&m_board);

				m_board.optimizeMembrane();

				printBoard(outStream);
			}break;

			case '5':
			{
				addBoardInHistory(&m_board);
				m_board.optimizeMembrane_byCutRowCols();
				printBoard(outStream);
			}break;

			case '6':
			{
				addBoardInHistory(&m_board);
				m_board.optimizeMembrane_byCutCorners();
				printBoard(outStream);
			}break;

			case 'U':
			{
				undoBoard();
				printBoard(outStream);
			}
			break;

			case 'V':
			{
				static char fileNameBuff[2048];
				if (writeHelperOutput) { outStream << "File name: "; }
				inStream >> fileNameBuff;
				saveBoard(fileNameBuff);
			}
			break;

			case 'W':
			{
				static char fileNameBuff[2048];
				if (writeHelperOutput) { outStream << "File name: "; }
				inStream >> fileNameBuff;

				initialize_fromFile(fileNameBuff);
			}
			break;

			case 'P':
			{
				printBoard(outStream);
			}
			break;

			default:
			{
				outStream << "Invalid option !" << endl;
			}
			break;
		}
	}
}

struct LogStep
{
	LogStep(int _id) :id(_id) {}

	int ev = -1;
	bool isRemovedSource = false;
	bool isAddedSource = false;
	TablePos sourcePos;
	bool isModfiedSource = false;
	bool isReorganization = false;
	float sourceNewPower = 0;
	int id = -1;

	void printLog(ostream& outStream)
	{
		outStream << "=============== Event:  " << id << "  (ev: " << ev << ") ===============" << endl;

		if (isRemovedSource)
			outStream << "Removed source (" << sourcePos.row << ", " << sourcePos.col << ")" << endl;

		if (isAddedSource)
			outStream << "Added source (" << sourcePos.row << ", " << sourcePos.col << ")" << " power " << sourceNewPower << endl;

		if (isModfiedSource)
			outStream << "Modified source (" << sourcePos.row << ", " << sourcePos.col << ")" << " power " << sourceNewPower << endl;

		if (isReorganization)
			outStream << "Reorganization called !!!";

		outStream << endl;
	}
};

bool Simulator::autoSimulate(const int numSteps, int minPower, int maxPower, const char* resultsFileName)
{
	ofstream outFile("result.txt", std::ofstream::out);
	if (outFile.is_open() == false)
	{
		assert("can't open the results file ! Is it opened or something ?");
		return false;
	}

	for (int i = 0; i < numSteps; i++)
	{
		LogStep logStep(i);

		// TODO: generate events 
		const int choice = randRange(1, 10);
		logStep.ev = choice;
		
		// 30% to do nothing or data flow analysis
		if (choice <= 2)
		{
			if (choice <= 0)
			{
				doDataFlowSimulation_serial(false, outFile);
			}
			continue;
		}			
		// 40% for reorganization
		else if (choice <= 7)
		{
			m_root->onRootMsgReorganize();
			logStep.isReorganization = true;
		}
		else // 30% source events
		{
			auto& mapOfSources = m_board.m_posToSourceMap;
			if (mapOfSources.empty() || choice <= 8)
			{
				SourceInfo src;
				float newPower = (float)randRange(minPower, maxPower);
				src.overridePower(newPower);
				TablePos pos(randRange(0, MAX_ROWS-1), randRange(0, MAX_COLS-1));
				
				addSource(pos, src);

				logStep.isAddedSource = true;
				logStep.sourcePos = pos;
				logStep.sourceNewPower = src.getPower();
			}
			else if (choice <= 9)
			{
				SourceInfo src;
				const float newPower = (float)randRange(minPower, maxPower);
				src.overridePower(newPower);

				const TablePos selectedPos = m_board.selectRandomSource();
				if (selectedPos.col != INVALID_POS && selectedPos.row != INVALID_POS)
				{
					modifySource(selectedPos, src);

					logStep.isModfiedSource = true;
					logStep.sourcePos = selectedPos;
					logStep.sourceNewPower = src.getPower();
				}
			}
			else if (choice <= 10)
			{
				const TablePos selectedPos = m_board.selectRandomSource();
				if (selectedPos.col != INVALID_POS && selectedPos.row != INVALID_POS)
				{
					removeSource(selectedPos);

					logStep.isRemovedSource = true;
					logStep.sourcePos = selectedPos;
				}
			}
		}

		logStep.printLog(outFile);
		printBoard(outFile);
	}

	return true;
}

void Simulator::addBoardInHistory(const BoardObject* board)
{
	assert(m_historyIndex >= 0 && m_historyIndex < SIMULATION_BOARD_HISTORY_SIZE);
	m_history[m_historyIndex] = *board;
	m_historyIndex = (m_historyIndex + 1) % SIMULATION_BOARD_HISTORY_SIZE;
}

void Simulator::undoBoard()
{
	m_historyIndex--;
	m_board = m_history[m_historyIndex];
	m_board.updateInternalCellsInfo();

	m_root->onRootMsgBroadcastStructure(&m_board);
}

void Simulator::doDataFlowSimulation_serial(const bool withReconfiguration, std::ostream& output)
{
	output << "\n\n\n======= Performing data flow simulation =========== \n";


	// Simulate
	if (withReconfiguration)
	{
		output << "reconfiguring first...\n";
		addBoardInHistory(&m_board);
		m_root->onRootMsgReorganize();
		printBoard(output);
	}

	m_board.doDataFlowSimulation_serial(g_simulationTicksForDataFlowEstimation, true);
	const float flowRes = m_board.getLastSimulationAvgDataFlowPerUnit();
	output << "Per unit of time there is an avg flow of: " << flowRes << endl;
}

void Simulator::doUnitTests()
{
	{
		BoardObject copyBoard = m_board;
		SubtreeInfo outSubtree;
		const int optionTestRow = 0;
		const int optionTestCol = 3;
		copyBoard.cutSubtree(optionTestRow, optionTestCol, outSubtree);
		cout << "BOard without subtree : " << endl;
		copyBoard.printBoard(cout);
		AvailablePositionsToMove positions;
		int bestOptionIndex = INVALID_POS;

		copyBoard.evaluatePositionsToMove(optionTestRow, optionTestCol, outSubtree, positions, bestOptionIndex);

		cout << "Test 1 res: " << endl;
		cout << "Best option index: " << bestOptionIndex << endl;
		int optionIter = 0;
		for (const AvailablePosInfoAndDeltaScore& posAndScore : positions)
		{
			cout << "Option " << optionIter <<": " << posAndScore << " ";
			optionIter++;
		}
	}
}

void Simulator::generateOptimalAndRandomBoard(BoardObject& outOptimalBoard, BoardObject& outRandomBoard)
{
	outOptimalBoard.setRowAndColGenerators(&m_rowGenerator, &m_colGenerator);
	outRandomBoard.setRowAndColGenerators(&m_rowGenerator, &m_colGenerator);

	// Step 1: generate an optimal board
	//--------------------------------------
	// Actually generate a random board then find the optimal sources positions for this board
	const bool resOptimal = outOptimalBoard.generateRandomBoard(g_depthForAutoInitialization, g_numSourcesOnRandomBoard);
	if (resOptimal == false)
	{
		assert(false && "Couldn't generate the optimal board");
	}

	// Try all possible combinations of 2 sources and keep the best
	// For now, hardcoded to only 2 sources
	assert(g_numSourcesOnRandomBoard == 2 && "TODO: hardcoded for 2 sources only. do a backtracking here instead of this for please ");

	SourceInfo s1Info; s1Info.overridePower(789);
	SourceInfo s2Info; s2Info.overridePower(345);

	TablePos bestS1, bestS2;
	float maxFlow = MIN_SCORE;
	//for (int s1Row = 0; s1Row < MAX_ROWS; s1Row++)
		//for (int s1Col = 0; s1Col < MAX_COLS; s1Col++)
			//for (int s2Row = 0; s2Row < MAX_ROWS; s2Row++)
				//for (int s2Col = 0; s2Col < MAX_COLS; s2Col++)
				// {

	for (int tryy = 0; tryy < 10; tryy++)
	{
		TablePos pos1 = getRandomTablePos();
		TablePos pos2 = getRandomTablePos();
		int s1Row = pos1.row;
		int s2Row = pos2.row;
		int s1Col = pos1.col;
		int s2Col = pos2.col;

				{
					// Remove all sources and add these two
					outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, TablePos (0, 0), SourceInfo(), true);

					// Add the new sources
					TablePos s1Pos(s1Row, s1Col);
					TablePos s2Pos(s2Row, s2Col);
					outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, s1Pos, s1Info, false);
					outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, s2Pos, s2Info, false);

					outOptimalBoard.doDataFlowSimulation_serial(1);
					const float flow = outOptimalBoard.getLastSimulationAvgDataFlowPerUnit();

					if (flow > maxFlow)
					{
						maxFlow = flow;
						bestS1 = s1Pos;
						bestS2 = s2Pos;
					}
				}
		}

	// Apply the best option for this board
	outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, TablePos(), SourceInfo(), true);
	outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, bestS1, s1Info, false);
	outOptimalBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, bestS2, s2Info, false);
	outOptimalBoard.reorganizeMaxFlow(nullptr);

	// Step 2: copy the optimal board and modify sources then reorganize for a random number of frames. 
	// Repeat for a couple of times 
	//--------------------------------------
	outRandomBoard = outOptimalBoard;
	// TODO: move these parameters outside in ini file
	const int numAttemptsToModifyStructure = 5;
	const int numReorganizations = 6;

	for (int attemptIter = 0; attemptIter < numAttemptsToModifyStructure; attemptIter++)
	{
		// Remove existent sources
		outRandomBoard.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, TablePos(), SourceInfo(), true);

		for (int sourceAddIter = 0; sourceAddIter < g_numSourcesOnRandomBoard; sourceAddIter++)
		{
			TablePos pos = getRandomTablePos();
			SourceInfo s; s.overridePower((float)randRange(g_minPowerForWirelessSource, g_maxPowerForWirelessSource));
			outRandomBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, pos, s, false);
		}

		// Call reorganize
		float prevAvgFlow = 0;
		for (int reorganizeIter = 0; reorganizeIter < numReorganizations; reorganizeIter++)
		{
			outRandomBoard.reorganize(*g_debugLogOutput);
			outRandomBoard.doDataFlowSimulation_serial(1);
			const float avgFlow = outRandomBoard.getLastSimulationAvgDataFlowPerUnit();
			
			if (avgFlow > prevAvgFlow)
				prevAvgFlow = avgFlow;
			else
				break;
		}
	}

	// Copy the sources from the optimal board to the random one
	outRandomBoard.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, TablePos(), SourceInfo(), true);
	for (auto& it : outOptimalBoard.m_posToSourceMap)
	{
		outRandomBoard.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, it.first, it.second, false);
	}
}

void Simulator::simulateOptimalReconfigurationScenarios(const int numScenarios, const char* filename)
{
	// Disable the verbose on this
	g_verboseLocalSolutions = false;
	g_verboseBestGatheredSolutions = false;

	const bool writeOutput = filename != nullptr;
	std::ofstream outputStream;
	if (writeOutput)
	{
		outputStream.open(filename, std::ofstream::out);
	}

	struct OptimalVsInitialReconfigFlow
	{
		OptimalVsInitialReconfigFlow(const int _optFlow, const int _after, const int _configsCount)
			: optimalFlow(_optFlow)
			, afterConfigFlow(_after)
			, numReconfigMade(_configsCount)
		{

		}

		int optimalFlow = 0;
		int afterConfigFlow = 0;
		int numReconfigMade = 0;
	};

	std::vector<OptimalVsInitialReconfigFlow> simStats;

	for (int scenario = 0; scenario < numScenarios; scenario++)
	{
		// Generate an optimal and a random board
		BoardObject optimalBoard;
		BoardObject randomBoard;
		Simulator::generateOptimalAndRandomBoard(optimalBoard, randomBoard);
		optimalBoard.doDataFlowSimulation_serial(1);

		//if (writeOutput)
		//{
		outputStream << " ===== Scenario " << scenario << "=====" << endl;
		const float optimalBoardMaxFlow = optimalBoard.getLastSimulationAvgDataFlowPerUnit();
		outputStream << "A. Optimal board with flow " << optimalBoardMaxFlow << " : " << endl;
		optimalBoard.printBoard(outputStream);
		outputStream << "B. Random initial board: " << endl;
		randomBoard.printBoard(outputStream);
		//}

		// Check how much the random board get to the optima board and after how many reconfiguration
		int numReorganizationsMade = 0;
		randomBoard.reorganizeMaxFlow(&numReorganizationsMade);

		const float randomBoardMaxFlow = randomBoard.getLastSimulationAvgDataFlowPerUnit();
		outputStream << "C. Board after reconfigurations has flow " << randomBoardMaxFlow << endl;
		randomBoard.printBoard(outputStream);

		simStats.push_back(OptimalVsInitialReconfigFlow((int)optimalBoardMaxFlow, (int)randomBoardMaxFlow, numReorganizationsMade));
	}

	ofstream resStatsFile("resultsStats.txt", std::ofstream::out);

	float avgFlowPercent = 0.0f;
	float avgReconfigsMade = 0.0f;
	for (const OptimalVsInitialReconfigFlow& entry : simStats)
	{
		const float percent = (float)entry.afterConfigFlow / (float)entry.optimalFlow;
		avgFlowPercent += std::min(percent, 1.0f);
		avgReconfigsMade += entry.numReconfigMade;
	}

	avgFlowPercent /= simStats.size();
	avgReconfigsMade /= simStats.size();

	resStatsFile << "Avg percent to optimal after reconfigurations: " << avgFlowPercent <<endl;
	resStatsFile << "Avg num reconfigurations done to max flow: " << avgReconfigsMade << endl;

	int scenarioIter = 0;
	for (const OptimalVsInitialReconfigFlow& entry : simStats)
	{		
		resStatsFile << "Scenario " << scenarioIter << " -  optimal flow: " << entry.optimalFlow << " after reconfig: " << entry.afterConfigFlow << " num reconfigs done: " << entry.numReconfigMade << endl;
		scenarioIter++;
	}
}

void Simulator::checkSourceModifyEvent(BoardObject& board, const float probForSourceEvent, bool& sourcesModified)
{
	sourcesModified = false;
	// Check for source event generation
	{
		const bool generateSourceEvent = randUniform() < probForSourceEvent ? true : false;
		if (generateSourceEvent)
		{
			// Select one, clear it then create a new one
			std::vector<TablePos> allSources(board.m_posToSourceMap.size());
			for (auto it : board.m_posToSourceMap)
			{
				allSources.push_back(it.first);
			}

			const int sourceToDelete = randRange(0, (int)allSources.size() - 1);
			if (sourceToDelete >= 0)
			{
				board.propagateSourceEvent(Cell::EVENT_SOURCE_REMOVE, allSources[sourceToDelete], SourceInfo(), false);

				const TablePos newSourcePos = getRandomTablePos();
				const SourceInfo newSourceInfo = getRandomSourceInfo();
				board.propagateSourceEvent(Cell::EVENT_SOURCE_ADD, newSourcePos, newSourceInfo, false);

				sourcesModified = true;
			}
			else
			{
				assert(false && "This source doesn't exist. Failed to generate event !!!");
			}
		}
	}
}

void Simulator::simulateFlowScenario(BoardObject& board, const float probForSourceEvent, const bool allowReconfiguration, const int sampleCount, const int sampleTicks, float& outAvgFlow, std::ostream& outStream)
{
	// Dynamic board simulation 
	{
		bool shouldReorganize = true; // Start with reorganizations from beginning since this is a random board
		float prevFlow = 0.0f;

		Cell* rootCell = board.getRootCell();
		for (int sampleIter = 0; sampleIter < sampleCount; sampleIter++)
		{
			rootCell->beginSimulation();
			for (int tickIter = 0; tickIter < sampleTicks; tickIter++)
			{
				bool sourcesModified = false;
				Simulator::checkSourceModifyEvent(board, probForSourceEvent, sourcesModified);

				// If configuration is not allowed don't do it !
				if (allowReconfiguration)
				{
					if (sourcesModified)
					{
						// Signal reorganization needed
						shouldReorganize = true;
						prevFlow = MIN_SCORE;
					}

					// Check reorganization 
					{
						if (shouldReorganize)
						{
							board.reorganize(*g_debugLogOutput);

							//board.printBoard(outStream);

							const float newFlow = board.getLastSimulationAvgDataFlowPerUnit();

							// If the flow doesn't improve, stop reorganizing. Otherwise, reorganize on the next tick too
							if (newFlow <= prevFlow)
							{
								shouldReorganize = false;
							}
							else
							{
								int test = 0;
								test++;
							}
						}
					}
				}

				// Simulate a single tick
				board.simulateTick_serial();
			}

			const float currFlow = rootCell->getAvgFlow();
			outAvgFlow += currFlow;
		}

		outAvgFlow /= sampleCount;
	}
}

void Simulator::simulateOptimalVsRandomFlowScenario(const char* fileToInitializeModel, const int numScenarios, const int sampleCount, const int sampleTicks, const int ticksBetweenSourceEvent, const int ticksToReconfigureRoot, std::ostream& outStream)
{
	// Disable the verbose on this
	g_verboseLocalSolutions = false;
	g_verboseBestGatheredSolutions = false;

	float avgFlowStatic = 0.0f;
	float avgFlowDynamic = 0.0f;
	const float probForSourceEvent = ticksBetweenSourceEvent / (float)sampleTicks;

	for (int scenarioIter = 0; scenarioIter < numScenarios; scenarioIter++)
	{
		BoardObject* staticBoard = new BoardObject();
		BoardObject* dynamicBoard = new BoardObject();
		
		// Tree collector
		if (g_useEModel == false)
		{
			generateOptimalAndRandomBoard(*staticBoard, *dynamicBoard);
		}
		else
		{
			// In the E model we initialize a model from file since we don't have yet a random generation procedure
			initialize_fromFile(fileToInitializeModel);
			*staticBoard = m_board;
			*dynamicBoard = m_board;
		}


		float scenarioFlowStatic = 0.0f;
		float scenarioFlowDynamic = 0.0f;
		std::srand(FIXED_SEED);
		simulateFlowScenario(*staticBoard, probForSourceEvent, false, sampleCount, sampleTicks, scenarioFlowStatic, outStream);
		std::srand(FIXED_SEED);
		simulateFlowScenario(*dynamicBoard, probForSourceEvent, true, sampleCount, sampleTicks, scenarioFlowDynamic, outStream);

		avgFlowStatic += scenarioFlowStatic;
		avgFlowDynamic += scenarioFlowDynamic;

		checkBoardLanguageConstraints();

		delete staticBoard;
		delete dynamicBoard;
	}

	avgFlowStatic /= numScenarios;
	avgFlowDynamic /= numScenarios;

	ofstream resStatsFile("resultsStats.txt", std::ofstream::out);

	ostream* streamsToWriteResults[2] = { &resStatsFile, &std::cout};

	for (int i = 0; i < 2; i++)
	{
		ostream& streamTarget = *streamsToWriteResults[i];
		streamTarget << " ========= RESULTS ========== " << std::endl;

		streamTarget << "Avg flow using static structure: " << avgFlowStatic << endl;
		streamTarget << "Avg flow using dynamic structure: " << avgFlowDynamic << endl;

	}
}

void Simulator::gatherAllDistinctSymbols(std::vector<char>& symbols)
{
	bool isSymbolSet[256] = { false };
	for (const char s : symbols)
	{
		isSymbolSet[s] = true;
	}

	g_allSymbolsSet.clear();
	for (int i = 0; i < 256; i++)
	{
		if (isSymbolSet[i])
		{
			g_allSymbolsSet.push_back(i);

			if (g_costPerResource[i] == INVALID_COST_PER_RESOURCE)
			{
				std::ostringstream strErr;
				strErr << "Symbol " << (char)i << "doesn't have a valid cost associated Overriding it to a large value !!!";
				assert(false && strErr.str().c_str());

				g_costPerResource[i] = FLT_MAX;
			}
		}
	}
}


