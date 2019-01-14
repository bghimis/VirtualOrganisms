#include "Cell.h"
#include "BoardObject.h"
#include <regex>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdlib.h>

extern bool g_verboseLocalSolutions;
extern bool g_verboseBestGatheredSolutions;
extern int g_maxFlowPerCell;
extern int g_ticksToDelayDataFlowCaptureOnRestructure;
extern int g_simulationTicksForDataFlowEstimation;
extern bool g_elasticModelEnabled;
extern float g_benefitPerUnitOfFlow;
extern float g_costPerResource[];
extern int g_maxResourcesToRent;
extern std::vector<char> g_allSymbolsSet;

extern bool g_verboseElasticModel_All;
extern bool g_verboseElasticModel_Results;

extern std::ostream* g_debugLogOutput;

extern std::regex g_colRXExpr;
extern std::regex g_rowRXExpr;

const TablePos Cell::DIR_OFFSET[DIR_COUNT]=
{
	TablePos(0, -1),
	TablePos(1, 0),
	TablePos(0, 1),
	TablePos(-1, 0)
};

float getCostForResource(char symbol) { return g_costPerResource[symbol]; }
void Cell::UniversalHash2D::reset() { memset(cellsHash, false, sizeof(cellsHash)); }

bool SimulationContext::getLeafNodeCapture(const TablePos& leafPos, float& outValue) const
{
	outValue = 0.0f;
	auto it = mLeafNodeToCaptureValue.find(leafPos);
	if (it == mLeafNodeToCaptureValue.end())
	{
		assert(false && "Couldn't find cached data for this leaf in the simulation context");
		return false;
	}

	outValue = it->second;
	return true;
}

Cell::Cell() : m_bufferedData((float)g_maxFlowPerCell), m_flowStatistics(nullptr)
{
	reset();
}

std::ostream& operator<<(std::ostream& out, const AvailablePosInfoAndDeltaScore& sol)
{
	out << "|(" << sol.row << "," << sol.col << ") " << " " << std::setprecision(3) << (int)sol.score << " |";
	return out;
}

Cell::~Cell()
{
	if (m_flowStatistics)
	{
		delete m_flowStatistics;
		return;
	}
}

void Cell::operator=(const Cell& other)
{
	if (other.m_flowStatistics)
	{
		m_flowStatistics = other.m_flowStatistics->duplicate();
	}
	else
	{
		if (m_flowStatistics)
			delete m_flowStatistics;

		m_flowStatistics = nullptr;
	}
	
	symbol = other.symbol;
	m_distanceToRoot = other.symbol;
	m_isEmpty = other.m_isEmpty;
	m_row = other.m_row;
	m_column = other.m_column;
	m_isRented = other.m_isRented;

#if RUNMODE == DIRECTIONAL_MODE
	m_cellType = other.m_cellType;
#endif

	m_boardView = nullptr;
#if RUNMODE != DIRECTIONAL_MODE
	m_parent = nullptr;
#endif
	m_left = m_down = m_right = m_up = nullptr;
	m_prevLeft = m_prevRight = m_prevDown = m_prevUp = nullptr;
}

void Cell::resetLinks()
{
#if RUNMODE != DIRECTIONAL_MODE
	m_parent = nullptr;
#endif

	m_left = m_down = m_right = m_up = nullptr;
	m_prevLeft = m_prevRight = m_prevDown = m_prevUp = nullptr;
}

void Cell::reset()
{
	symbol = ' ';
	m_distanceToRoot = -1;
	resetLinks();
	//m_isSource = false;
	m_isEmpty = true;
	m_row = -1;
	m_column = -1;
	m_boardView = nullptr;
	m_remainingTicksToDelayDataFlowCapture = 0;
	m_isRented = false;

#if RUNMODE == DIRECTIONAL_MODE
	m_cellType = CELL_NOTSET;
#endif

	/*
	if (m_flowStatistics)
	{
		delete m_flowStatistics;
		m_flowStatistics = nullptr;
	}*/
}

void Cell::onMsgBroadcastStructure(BoardObject* structure)
{
#if RUNMODE == DIRECTIONAL_MODE
	// Follow previous links. Can't get a cycle so no need to protect the flood fill
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const int rowOffset = DIR_OFFSET[dirIter].row;
		const int colOffset = DIR_OFFSET[dirIter].col;

		Cell* prev = *m_previousByDir[dirIter];
		if (prev)
			prev->onMsgBroadcastStructure(structure);
	}
#else
	if (m_left)
		m_left->onMsgBroadcastStructure(structure);

	if (m_down)
		m_down->onMsgBroadcastStructure(structure);

#endif

	// Update the local blackboard
	if (m_boardView == nullptr)
		m_boardView = new BoardObject();

	*m_boardView = *structure;
}

void Cell::onRootMsgBroadcastStructure(BoardObject* structure)
{
#if RUNMODE == DIRECTIONAL_MODE
	// Get the cell below and send it the stuff. It will send further the structure
	// Can get it from board since Root is using the main (real) board while the others have a private copy. 
	Cell* prev = &structure->m_board[m_row + 1][m_column];
	assert(prev->m_up == this); // Sanity check to be sure that we take it from the right board...
	prev->onMsgBroadcastStructure(structure);
#else 
	if (m_left)
		m_left->onMsgBroadcastStructure(structure);

	if (m_down)
		m_down->onMsgBroadcastStructure(structure);
#endif

	m_boardView = structure;
}

void Cell::onMsgDiscoverStructure(int currRow, int currCol, int depth)
{
	m_distanceToRoot = depth;
	m_row = currRow;
	m_column = currCol;

#if RUNMODE == DIRECTIONAL_MODE
	if (isRoot())
	{
		// Start from below and follow previous links
		onMsgDiscoverStructure(currRow + 1, currCol, depth + 1);
	}
	else
	{
		// Follow previous links. Can't get a cycle so no need to protect the flood fill
		for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
		{
			const int rowOffset = DIR_OFFSET[dirIter].row;
			const int colOffset = DIR_OFFSET[dirIter].col;

			Cell* prev = *m_previousByDir[dirIter];
			if (prev)
				prev->onMsgDiscoverStructure(currRow + rowOffset, currCol + colOffset, depth + 1 );
		}
	}
#else
	if (m_left)
		m_left->onMsgDiscoverStructure(currRow, currCol - 1, depth + 1);

	if (m_down)
		m_down->onMsgDiscoverStructure(currRow + 1, currCol, depth + 1);
#endif
}

void Cell::onRootMsgReorganize()
{
	assert(m_column == MAX_COLS - 1 && m_row == 0);	// Just a check for sanity :)

													// Allocate an array for results - TODO: optimize we should know easily how many nodes are in the tree 
	std::vector<AvailablePosInfoAndDeltaScore> localBestResults;
	localBestResults.reserve(MAX_COLS * MAX_ROWS);

	// Send message to children first
	if (m_left)
		m_left->onMsgReorganizeStart(localBestResults);

	if (m_down)
		m_down->onMsgReorganizeStart(localBestResults);

	// Get the best result and send the decision further
	int maxIndex = INVALID_POS;
	float maxScore = MIN_SCORE;
	for (uint i = 0; i < localBestResults.size(); i++)
	{
		if (maxIndex == -1 || localBestResults[i].score > maxScore)
		{
			maxIndex = i;
			maxScore = localBestResults[i].score;
		}
	}

	if (g_verboseBestGatheredSolutions && maxIndex != INVALID_POS)
	{
		// Check the current flow against the best found by any possible move
		m_boardView->doDataFlowSimulation_serial(1);
		const float currentScore = (float)m_boardView->getLastSimulationAvgDataFlowPerUnit();

		std::ostringstream strOut;
		strOut << "\n-----  Best solutions on root !!:  ----- \n";
		const AvailablePosInfoAndDeltaScore& selectedOpt = localBestResults[maxIndex];
		strOut << "ROOT: Local best option from children -  " << " Cell: (" << selectedOpt.selectedRow << ", " << selectedOpt.selectedColumn << ") " << selectedOpt << "\n";
		strOut << "Other options: ";

		for (const AvailablePosInfoAndDeltaScore& availablePos : localBestResults)
		{
			strOut << " Cell: (" << availablePos.selectedRow << ", " << availablePos.selectedColumn << ") " << availablePos;
		}

		strOut << "\n PREVIOUS AVG FLOW: " << (int)currentScore;

		// If not better, invalidate result for code below
		if (maxScore <= currentScore)
		{
			maxIndex = INVALID_POS;
			maxScore = MIN_SCORE;

			strOut << " !! NOT APPLYING ANY CHANGE BECAUSE CURRENT FLOW IS BETTER THAN EVERYTHING" << std::endl;
		}
		else
		{
			strOut << " !! NEW AVG FLOW: " << (int)maxScore << std::endl;
		}

		*g_debugLogOutput << strOut.str() << " \n\n";
	}

	if (maxIndex == INVALID_POS)
	{
		// Send empty decision further
		AvailablePosInfoAndDeltaScore dummy;
		onMsgReorganizeEnd(INVALID_POS, INVALID_POS, dummy);
	}
	else
	{
		const AvailablePosInfoAndDeltaScore& bestRes = localBestResults[maxIndex];
		assert(isCoordinateValid(bestRes.selectedRow, bestRes.selectedColumn) && "It looks like the selected row/column for restructuring has failed to fill correctly. There is a bug !");
		onMsgReorganizeEnd(bestRes.selectedRow, bestRes.selectedColumn, bestRes);
	}
}

void Cell::onMsgReorganizeStart(std::vector<AvailablePosInfoAndDeltaScore>& output)
{
	// Send message to children first
	if (m_left)
		m_left->onMsgReorganizeStart(output);

	if (m_down)
		m_down->onMsgReorganizeStart(output);

	// Copy the global structure and delete from it this subtree
	BoardObject boardWithoutMySubtree = *m_boardView;
	SubtreeInfo subTreeCut;
	boardWithoutMySubtree.cutSubtree(m_row, m_column, subTreeCut);

	// Get and evaluate the available positions to move this cut subtree
	AvailablePositionsToMove localOptions;
	int localBestOptionIndex = INVALID_POS;

	boardWithoutMySubtree.evaluatePositionsToMove(m_row, m_column, subTreeCut, localOptions, localBestOptionIndex);

	// No local option ?
	if (localBestOptionIndex == INVALID_POS)
	{
		// Send empty decision from this node
	}
	else // Send my best option then
	{
		const AvailablePosInfoAndDeltaScore& bestOption = localOptions[localBestOptionIndex];
		output.push_back(bestOption);

		if (g_verboseLocalSolutions)
		{
			std::ostringstream outMsg;
			outMsg << "Cell (" << m_row << ", " << m_column << ") ";
			outMsg << "Best: " << bestOption << "Curr Sc: " << bestOption.score << " others: ";
			for (const AvailablePosInfoAndDeltaScore& sol : localOptions)
			{
				outMsg << " " << sol;
			}
			
			outMsg << "\n";
			*g_debugLogOutput << outMsg.str();
		}
	}
}

// The decision has been made: move the subtree rooted at selectedRow and selectedCol to targetPosAndDir
void Cell::onMsgReorganizeEnd(int selectedRow, int selectedCol, const AvailablePosInfoAndDeltaScore& targetPosAndDir)
{
	// Update the local table then broadcast the discover and structure broadcast message to update everyone table
	std::string outCutSubtree;

	// Do re-structuring only if needed
	bool isBoardChanged = false;
	if (isCoordinateValid(selectedRow, selectedCol))
	{
		SubtreeInfo subtreeToMove;
		m_boardView->cutSubtree(selectedRow, selectedCol, subtreeToMove);
		const bool res = m_boardView->tryApplySubtree(targetPosAndDir.row, targetPosAndDir.col, subtreeToMove, true, true); // double check
		assert(res);

		isBoardChanged = true;
	}

	if (g_elasticModelEnabled)
	{
		*g_debugLogOutput << "Board AFTER reconfiguration: \n";
		m_boardView->printBoard(*g_debugLogOutput);
	}

	isBoardChanged |= analyzeElasticModel(*g_debugLogOutput);

	// If the board was changed, discover the structure and broadcast it to everyone
	if (isBoardChanged)
	{
		onMsgDiscoverStructure(m_row, m_column, 0);
		onMsgBroadcastStructure(m_boardView);
	}
	
	// If I´m the cell selected for move, i will have to delay some ticks data buffering since i have to change my parent
	if (selectedRow == m_row && selectedCol == m_column)
	{
		m_remainingTicksToDelayDataFlowCapture = g_ticksToDelayDataFlowCaptureOnRestructure;
	}
}

void Cell::captureDataFlow(const SimulationContext& simContext)
{
	if (m_remainingTicksToDelayDataFlowCapture > 0)
	{
		m_remainingTicksToDelayDataFlowCapture--;
		return; 
	}
	
	// Captures as much as it can from environment (if leaf) or from children if internal node	

	// Doesn't matter if parents / or children update in parallel stuff, the most important thing is to not violate the flow constraint
	// Better than blocking everything
	const float capRemaining = 0.0f + g_maxFlowPerCell - m_bufferedData.getCurrentCap();
	if (isLeaf())
	{
		// Take data from simulation context
		float leafCaptureValue = 0.0f;
		const bool succeded = simContext.getLeafNodeCapture(TablePos(m_row, m_column), leafCaptureValue);
		assert(capRemaining >= leafCaptureValue && "Incorect capture value. Static simulation is wrong !!");
		m_bufferedData.add(leafCaptureValue);
	}
	else
	{
		const float leftChildCap = (m_left ? m_left->getCurrentBufferedCap() : 0.0f);
		const float rightChildCap = (m_down ? m_down->getCurrentBufferedCap() : 0.0f);
		const float totalChildrenCap = leftChildCap + rightChildCap;
		
		// compute how much we can take from left/right children
		float takeLeft = leftChildCap;
		float takeRight = rightChildCap;
		
		// If not enough cap take from children proportionally to their input and my remaining cap
		if (totalChildrenCap > capRemaining)
		{
			const float ratioCapture = ((float)(capRemaining))/totalChildrenCap;
			takeLeft = floor(ratioCapture * takeLeft);  
			takeRight = floor(ratioCapture * takeRight);
		}
		
		// Subtract the right amount from children then add to me
		if (m_left)
			m_left->subtractData(takeLeft);

		if (m_down)
			m_down->subtractData(takeRight);
			
		m_bufferedData.add(takeLeft);
		m_bufferedData.add(takeRight);

	}
}

// In the serial simulation the update order is deterministic (this is usefully to have proper evaluation of results under low number of real
// physical processes).
void Cell::simulateTick_serial(const SimulationContext& simContext)
{
	if (m_left)
		m_left->simulateTick_serial(simContext);

	if (m_down)
		m_down->simulateTick_serial(simContext);

	captureDataFlow(simContext);

	if (isRoot())
	{
		const float currRootsDataSize = m_bufferedData.getCurrentCap();
		m_flowStatistics->addStat(currRootsDataSize);
		
		// Use the captured data here then clear it
		m_bufferedData.subtract(currRootsDataSize);
	}
}

float Cell::getRemainingCap() const 
{ 
	const float cap = (0.0f + g_maxFlowPerCell - m_bufferedData.getCurrentCap()); 
	assert(cap >= 0.0f); 
	return cap; 
}

void Cell::gatherNewResourcesPos(Cell* cell, std::vector<TablePos>& outPositions, UniversalHash2D& hash)
{
	if (cell == nullptr)
		return;

	const int currRow = cell->m_row;
	const int currCol = cell->m_column;

	if (cell->isLeaf())
	{
		TablePos nextPosLeft(currRow, currCol - 1);
		if (isCoordinateValid(nextPosLeft) && hash.isCellSet(nextPosLeft) == false && m_boardView->isPosFree(nextPosLeft))
		{
			outPositions.push_back(nextPosLeft);
		}
		
		TablePos nextPosDown(currRow + 1, currCol);
		if (isCoordinateValid(nextPosDown) && hash.isCellSet(nextPosDown) == false && m_boardView->isPosFree(nextPosDown))
		{
			outPositions.push_back(nextPosDown);
		}
	}
	else
	{
		if (isCoordinateValid(currRow, currCol)
			&& hash.isCellSet(currRow, currCol) == false)
		{
			outPositions.push_back(TablePos(currRow, currCol));
			hash.setCell(currRow, currCol);
		}

		gatherNewResourcesPos(cell->m_left, outPositions, hash);
		gatherNewResourcesPos(cell->m_down, outPositions, hash);
	}
}

void Cell::elasticBoardCompare(BoardObject& copyBoard, const bool isResourceAdded, const char symbolOfResource, const TablePos& resourcePos, ElasticResourceEval& outResult, const float oldAvgFlow, std::ostream& outDebugStream)
{
	float costForResource = getCostForResource(symbolOfResource);
	copyBoard.doDataFlowSimulation_serial(1);
	const float oldBenefitValue = oldAvgFlow * g_benefitPerUnitOfFlow;
	const float newAvgFlow = copyBoard.getLastSimulationAvgDataFlowPerUnit();
	const float newResourceBenefit = (newAvgFlow * g_benefitPerUnitOfFlow) + (isResourceAdded ? -costForResource : +costForResource);
	const float newResourceBenefitDiff = newResourceBenefit - oldBenefitValue;
	outResult.augment(copyBoard, symbolOfResource, newResourceBenefitDiff, resourcePos);

	if (g_verboseElasticModel_All)
	{
		outDebugStream << " Trying resource Symbol-" << symbolOfResource << " at position " << resourcePos.row<<","<<resourcePos.col << " and benefit diff to before is " << newResourceBenefitDiff <<"(flow before: " << oldAvgFlow <<", after: " << newAvgFlow << "\n";
	}
}

bool Cell::root_checkAddResources(std::ostream& outDebugStream)
{
	// Debug stuff
	if (g_verboseElasticModel_Results)
	{
		outDebugStream << "ADD RESOURCES - Analyze start\n";
	}

	assert(isRoot());
	std::vector<TablePos> potentialAddPos;
	potentialAddPos.reserve(MAX_ROWS * MAX_ROWS / 2);// just a priori allocation

	// Find the potential positions where resources can be added
	UniversalHash2D hash;
	gatherNewResourcesPos(this, potentialAddPos, hash);

	// Find the current board's flow and benefit value
	m_boardView->doDataFlowSimulation_serial(1);
	const float currentAvgFlow = m_boardView->getLastSimulationAvgDataFlowPerUnit();
	const float oldBenefitValue = currentAvgFlow * g_benefitPerUnitOfFlow;

	// Over all resource symbols and positions where we can apply this resource find the best possible place to add a new resource
	// TODO: there is some redundancy here but i'm too lazy to optimize - basically if you need that it failed under some conditions with a previous symbol, it will fail for the next ones too so we can reduce CPU cycles with this
	ElasticResourceEval bestResult;
	bestResult.bestBoard_onlySymbols = new ((BoardObject*)alloca(sizeof(BoardObject))) BoardObject();
	for (char symbolToAdd : g_allSymbolsSet)
	{
		// For each potential position try to cut / move subtree if needed
		for (const TablePos& addPos : potentialAddPos)
		{
			// Is the position close to a leaf actually ?
			if (m_boardView->isPosFree(addPos))
			{
				BoardObject copyBoard = *m_boardView;
				Cell& targetCell = copyBoard(addPos.row, addPos.col); 
				targetCell.setEmpty();
				targetCell.setSymbol(symbolToAdd);	// We already know from pos list if this cell is a valid one or not

				// If the board is complaint with the language for row and column update internal links and compare against best result
				if (copyBoard.isCompliantWithRowColPatterns(addPos.row, addPos.col))
				{
					copyBoard.updateInternalCellsInfo();
					elasticBoardCompare(copyBoard, true, symbolToAdd, addPos, bestResult, currentAvgFlow, outDebugStream);
				}
			}
			else
			{
				// Horizontal shift (left side) and vertical shift (down)
				constexpr int numDirs = 2;
				int offsetsPerDirection[numDirs][2] = { {0,-1}, {1,0} }; // for each direction put the offsets of row/col

				BoardObject baseBoard = *m_boardView;
				SubtreeInfo subtree;
				baseBoard.cutSubtree(addPos.row, addPos.col, subtree);
				Cell& targetCell = baseBoard(addPos.row, addPos.col);
				targetCell.setEmpty();
				targetCell.setSymbol(symbolToAdd);

				// Check compliance first
				if (baseBoard.isCompliantWithRowColPatterns(addPos.row, addPos.col))
				{
					for (int dirIter = 0; dirIter < numDirs; dirIter++)
					{
						BoardObject copyBoard = baseBoard;

						const int offsetRow = offsetsPerDirection[dirIter][0];
						const int offsetCol = offsetsPerDirection[dirIter][1];
						TablePos newSubtreeRoot(addPos.row + offsetRow, addPos.col + offsetCol);
						const bool res = copyBoard.tryApplySubtree(newSubtreeRoot.row, newSubtreeRoot.col, subtree, true, true);
						if (res)
						{
							copyBoard.doDataFlowSimulation_serial(1);
							copyBoard.getLastSimulationAvgDataFlowPerUnit();
							elasticBoardCompare(copyBoard, true, symbolToAdd, addPos, bestResult, currentAvgFlow, outDebugStream);
						}
					}
				}
			}
		}
	}

	// Debug stuff
	if (g_verboseElasticModel_Results)
	{
		outDebugStream << bestResult;
	}
	//--------

	// If the result is valid apply the optimal board and move on.
	if (bestResult.isValid())
	{
		BoardObject* boardViewAddressCopy = m_boardView;	// This will be canceled after copyJustCells call
		m_boardView->copyJustCells(*bestResult.bestBoard_onlySymbols);
		assert(m_boardView == nullptr || m_boardView == boardViewAddressCopy);

		m_boardView = boardViewAddressCopy;
		m_boardView->addRentedResource(bestResult.symbolAdded, bestResult.pos);
		m_boardView->updateInternalCellsInfo();

		return true;
	}

	return false;
}

bool Cell::root_checkRemoveResources(std::ostream& outDebugStream)
{
	// Debug stuff
	if (g_verboseElasticModel_Results)
	{
		outDebugStream << "REMOVE RESOURCES - Analyze start\n";
	}

	assert(isRoot());
	auto& rentedResources = m_boardView->m_rentedResources;

	// Find the current board's flow and benefit value
	m_boardView->doDataFlowSimulation_serial(1);
	const float currentAvgFlow = m_boardView->getLastSimulationAvgDataFlowPerUnit();
	const float oldBenefitValue = currentAvgFlow * g_benefitPerUnitOfFlow;

	ElasticResourceEval bestResult;
	bestResult.bestBoard_onlySymbols = new ((BoardObject*)alloca(sizeof(BoardObject))) BoardObject();
	for (auto it = rentedResources.begin(); it != rentedResources.end(); it++)
	{
		const RentedResourceInfo& rcRented = *it;
		const TablePos& rcPos = rcRented.pos;

		// If we have something below and on the left side we can shift those subtrees
		// to the deleted rented position
		constexpr int MAX_NUM_TRIES = 2;
		int numValidSubtreesToShift = 0;
		TablePos validSubtreesToShift[MAX_NUM_TRIES];
		TablePos subtreesAttempts[MAX_NUM_TRIES] = {
			TablePos(rcPos.row + 1, rcPos.col), // Down pos
			TablePos(rcPos.row, rcPos.col - 1), // Left pos
		};

		for (int attemptIter = 0; attemptIter < MAX_NUM_TRIES; attemptIter++)
		{
			const TablePos& subtreePos = subtreesAttempts[attemptIter];

			// Check if there is something there and the coordinate is valid
			if (isCoordinateValid(subtreePos) && m_boardView->isPosFree(subtreePos) == false)
			{
				validSubtreesToShift[numValidSubtreesToShift++] = subtreePos;
			}
		}

		// Now bring the best subtree here (if any available). 
		// Otherwise compare the benefit of the current configuration
		if (numValidSubtreesToShift > 0)
		{
			for (int validSubtreeIter = 0; validSubtreeIter < numValidSubtreesToShift; validSubtreeIter++)
			{
				const TablePos& subtreePos = validSubtreesToShift[validSubtreeIter];

				BoardObject copyBoardBase = *m_boardView;
				copyBoardBase(rcPos.row, rcPos.col).setEmpty();
				copyBoardBase.updateInternalCellsInfo();

				SubtreeInfo outSubtree;
				copyBoardBase.cutSubtree(subtreePos.row, subtreePos.col, outSubtree);
				if (copyBoardBase.tryApplySubtree(rcPos.row, rcPos.col, outSubtree, true, true))
				{
					elasticBoardCompare(copyBoardBase, false, rcRented.symbol, rcPos, bestResult, currentAvgFlow, outDebugStream);
				}
			}
		}
		else
		{
			BoardObject copyBoardBase = *m_boardView;
			copyBoardBase(rcPos.row, rcPos.col).setEmpty();
			copyBoardBase.updateInternalCellsInfo();

			elasticBoardCompare(copyBoardBase, false, rcRented.symbol, rcPos, bestResult, currentAvgFlow, outDebugStream);
		}
	}

	// Debug stuff
	if (g_verboseElasticModel_Results)
	{
		outDebugStream << bestResult;
	}
	//--------

	// If the result is valid apply the optimal board and move on.
	if (bestResult.isValid())
	{
		m_boardView->removeRentedSource(bestResult.pos);

		BoardObject* boardViewAddressCopy = m_boardView;	// This will be canceled after copyJustCells call
		m_boardView->copyJustCells(*bestResult.bestBoard_onlySymbols);
		m_boardView = boardViewAddressCopy;
		m_boardView->updateInternalCellsInfo();

		return true;
	}

	return false;
}

bool Cell::analyzeElasticModel(std::ostream& outDebugStream)
{
	if (!g_elasticModelEnabled)
		return false;

	const bool anyElasticModelDebugEnabled = g_verboseElasticModel_All || g_verboseElasticModel_Results;
	if (anyElasticModelDebugEnabled)
		outDebugStream << "\n----- Elastic model analysis BEGIN ----- num resources left to rent: " << m_boardView->getNumAvailableResourcesToRent() << "\n";

	// While we can still add resources try to add if score would be better
	bool anyResourceAdded = false;
	bool anyResourceRemoved = false;
	while (m_boardView->getNumAvailableResourcesToRent() > 0)
	{
		const bool res = root_checkAddResources(outDebugStream);
		anyResourceAdded |= res;

		// Don't continue if the benefit formula doesn't improve
		// otherwise keep adding as long as we improve the benefits
		if (res == false)
			break;
	}

	// If we didn't added any resource check maybe we can actually improve the score by giving the resource back
	if (anyResourceAdded == false)
	{
		if (anyElasticModelDebugEnabled)
			outDebugStream << "NO RESOURCES COULD BE ADDED GOING TO TRY AND REMOVE SOME\n";

		while (m_boardView->m_rentedResources.empty() == false)
		{
			const bool res = root_checkRemoveResources(outDebugStream);
			anyResourceRemoved |= res;

			// Don't try remove anymore if no improvement 
			if (res == false)
				break;
		}

		if (anyElasticModelDebugEnabled)
		{
			if (anyResourceRemoved == false)
			{
				outDebugStream << "NO RESOURCES COULD BE REMOVED \n";
			}
		}
	}

	if (anyElasticModelDebugEnabled)
		outDebugStream << "\n----- END Elastic model analysis -----\n\n";

	return (anyResourceAdded || anyResourceRemoved);
}

void ElasticResourceEval::augment(const BoardObject& copyBoard, const char _symbolAdded, const float _benefit, const TablePos& _pos)
{
	if (_benefit > benefit)
	{
		benefit = _benefit;
		symbolAdded = _symbolAdded;
		pos = _pos;

		bestBoard_onlySymbols->copyJustCells(copyBoard);
	}
}

std::ostream& operator <<(std::ostream& out, const ElasticResourceEval& data)
{
	out << "\n";
	if (data.isValid() == false)
		out << "No valid results found for resources\n";
	else
		out << "Best found result found "<< "Symbol- " << data.symbolAdded << " Pos- " << data.pos.row << "," << data.pos.col << " diff benefit " << data.benefit << "\n";

	return out;
}
