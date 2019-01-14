#include "BoardObject.h"
#include "Utils.h"
#include <unordered_map>
#include <assert.h>
#include "ExprGenerator.h"
#include <algorithm>
#include <stack>
#include <algorithm>
#include <set>

#define NOMINMAX
#include <windows.h>

using namespace std;

extern int g_speedOnConduct;
extern int g_minPowerForWirelessSource;
extern int g_maxPowerForWirelessSource;
extern int g_simulationTicksForDataFlowEstimation;
extern int g_minNodesOnRandomTree;
extern int g_powerChangeFrequency;
extern float g_maxPowerVelocityPerTick;
extern int g_maxFlowPerCell;
extern bool variableSourcesPower;
extern std::regex g_colRXExpr;
extern std::regex g_rowRXExpr;
extern int g_maxResourcesToRent;

extern bool g_verboseLocalSolutions;
extern bool g_verboseBestGatheredSolutions;
extern float g_energyLossThreshold;;

extern std::ostream* g_debugLogOutput;

BoardObject::~BoardObject()
{
}

bool BoardObject::addSource(const TablePos& pos, const SourceInfo& sourceInfo)
{
	auto it = m_posToSourceMap.find(pos);
	if (it != m_posToSourceMap.end())
	{
		//assert(false && "This source is already added !");
		// Just change the power
		it->second.overridePower(sourceInfo.getPower());
		return true;
	}

	m_posToSourceMap.insert(std::make_pair(pos, sourceInfo));

	return true;
}

bool BoardObject::modifySource(const TablePos& pos, const SourceInfo& sourceInfo)
{
	auto it = m_posToSourceMap.find(pos);
	if (it == m_posToSourceMap.end())
	{
		assert(false && "This source doesn't exist can't update !");
		return false;
	}

	it->second = sourceInfo;
	return true;
}

bool BoardObject::removeSource(const TablePos& pos, const bool allSources)
{
	if (allSources)
	{
		m_posToSourceMap.clear();
	}
	else
	{
		auto it = m_posToSourceMap.find(pos);
		if (it == m_posToSourceMap.end())
		{
			//assert(false && "This source doesn't exist can't delete it !");
			return false;
		}

		m_posToSourceMap.erase(it);
	}

	return true;
}

TablePos BoardObject::selectRandomSource() const
{
	if (m_posToSourceMap.empty())
	{
		assert(false);
		TablePos invalidPos(INVALID_POS, INVALID_POS);
		return invalidPos;
	}
	// Choose a bucket
	int bucket, bucket_size;
	do
	{
		bucket = (int)randRange(0, (int)m_posToSourceMap.bucket_count() - 1);
	} while ((bucket_size = (int)m_posToSourceMap.bucket_size(bucket)) == 0);

	// Normally this should be very small if the hash function is working properly
	auto element = std::next(m_posToSourceMap.begin(bucket), randRange(0, bucket_size - 1));
	return element->first;
}

float BoardObject::computeScoreForLeafAndSource(const TablePos& leafPos, const TablePos& srcPos, const SourceInfo& srcInfo) const
{
	// Score part 1: sum of signal(src) / distTo(src) , for all src on board 
	float scorePart1 = 0.0f;

	float distSqr = (float)manhattanDist(srcPos, leafPos);
	distSqr *= distSqr;

	const float scoreToAdd = ((float)(srcInfo.getPower())) / distSqr;
	scorePart1 += scoreToAdd;

	/*
	// Score part 2:   (1.0f / distanceToRoot) * speedOnConduct
	const float scorePart2 = (onlyFlowScore ? 0.0f : (1.0f / distanceToRoot) * g_speedOnConduct);
	const float totalScore = scorePart1 + scorePart2;
	*/
	return scorePart1;
}

int BoardObject::getNumNeighboors(const TablePos& pos) const
{
	int numNeighb = 0;
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const TablePos neighbPos = Cell::DIR_OFFSET[dirIter] + pos;
		if (!isCoordinateValid(neighbPos))
			continue;

		numNeighb += m_board[neighbPos.row][neighbPos.col].isFree() ? 0 : 1;
	}

	return numNeighb;
}

void BoardObject::updateBoardAfterSymbolsInit()
{
	setRootLocation(m_rootRow, m_rootCol);

	// Set the root first on board
	Cell* rootCell = &m_board[m_rootRow][m_rootCol];
	rootCell->m_row = m_rootRow;
	rootCell->m_column = m_rootCol;

	updateInternalCellsInfo();

	// a) discover structure message - root (fixed position) sends a message to help every cell find its position. When the information come back to root we do step b)
	// b) broadcast entire structure to everyone
	// After these two steps everybody knows the distance to root and the number of branches below
	rootCell->onRootMsgBroadcastStructure(this); // Root uses the global board !!!
	rootCell->onMsgDiscoverStructure(m_rootRow, m_rootCol, 0);
}

void BoardObject::updateInternalCellsInfo()
{
	// Reset all links first
	for (int i = 0; i < MAX_ROWS; i++)
		for (int j = 0; j < MAX_COLS; j++)
		{
			for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
			{
				(*m_board[i][j].m_followersByDir[dirIter]) = nullptr;
				(*m_board[i][j].m_previousByDir[dirIter]) = nullptr;
			}
		}


	// Start from root node
	Cell* rootCell = getRootCell();
	rootCell->m_distanceToRoot = 0;

	rootCell->m_row = m_rootRow;
	rootCell->m_column = m_rootCol;

	assert(rootCell->isRented() == false); // Jesus, i hope not :)
	m_rentedResources.clear();

#if RUNMODE == DIRECTIONAL_MODE
	// Points where the direction is changed
	std::vector<TablePos> inflexionPoints;
	getInflexionPointAndConnectMembrane(inflexionPoints);

	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);
	updateMembraneBounds(membraneCells);

#if 1

	// Go through each inflexion points and try to find/create links of trees appending to the membrane
	const TablePos* prevInflexionPoint = nullptr;
	for (const TablePos& pos : inflexionPoints)
	{
		if (prevInflexionPoint == nullptr)
		{
			prevInflexionPoint = &pos;
			continue;
		}


		// Is it vertical or horizontal direction ?
		const bool isHorizontal = prevInflexionPoint->row == pos.row;
		if (isHorizontal)
		{
			int colStart = prevInflexionPoint->col;
			int colEnd   = pos.col;
			if (colStart > colEnd)
				std::swap(colStart, colEnd);

			internalCreateColsLinks(pos.row, colStart, colEnd);
		}
		else
		{
			int rowStart = prevInflexionPoint->row;
			int rowEnd = pos.row;
			if (rowStart > rowEnd)
				std::swap(rowStart, rowEnd);

			internalCreateRowsLinks(pos.col, rowStart, rowEnd);
		}

		prevInflexionPoint = &pos;
	}
#endif

#else
	internalCreateLinks(rootCell, DIR_LEFT, 0, MAX_COLS - 2);
	internalCreateLinks(rootCell, DIR_DOWN, 1, MAX_COLS - 1);
#endif

	Cell* rootNode = getRootCell();
	rootNode->m_row = m_rootRow;
	rootNode->m_column = m_rootCol;
}

CellType BoardObject::decideCellType(const Cell* startCell, const DIRECTION dir)
{
	if (startCell == nullptr)
	{
		assert(false); // Because I'm not using this use case. if using, remove assert
		return CELL_MEMBRANE;
	}

	return getCellTypeRelativeToMembrane(startCell->m_row + Cell::DIR_OFFSET[dir].row, startCell->m_column + Cell::DIR_OFFSET[dir].col);

	/*
	if (startCell->m_cellType == CELL_MEMBRANE)
	{
		if ((dir == DIR_LEFT && startCell->m_symbol == '2')
			|| (dir == DIR_UP && startCell->m_symbol == '4')
			|| (dir == DIR_RIGHT && startCell->m_symbol == '7')
			|| (dir == DIR_DOWN && startCell->m_symbol == 'e'))
		{
			return CELL_EXTERIOR;
		}
		else if ((dir == DIR_RIGHT && startCell->m_symbol == '2')
			|| (dir == DIR_DOWN && startCell->m_symbol == '4')
			|| (dir == DIR_LEFT && startCell->m_symbol == '7')
			|| (dir == DIR_UP && startCell->m_symbol == 'e'))
		{
			return CELL_INTERIOR;
		}
	}
	else
	{
		return startCell->m_cellType;
	}

	assert(false && "type not found");

	return CELL_NOTSET;
	*/
}

#if RUNMODE == DIRECTIONAL_MODE
void BoardObject::getInflexionPointAndConnectMembrane(std::vector<TablePos>& outInflexionPositions, const bool justGetInflexionPoints)
{
	// Iterate over membrane - property : it has only straightforward direction until we get back to the root
	TablePos rootPos(m_rootRow, m_rootCol);
	TablePos currPos(m_rootRow, m_rootCol);
	assert(m_board[m_rootRow][m_rootCol].m_symbol == '4' && "I think you didn't specified the root correctly !");
	DIRECTION currDir = Cell::getDirectionFromSymbol(m_board[m_rootRow][m_rootCol].m_symbol);
	DIRECTION prevDir = currDir;

	outInflexionPositions.clear();
	outInflexionPositions.push_back(TablePos(m_rootRow, m_rootCol));

	Cell* prevCell = nullptr;
	bool alreadyMetRoot = false;
	
	do
	{
		Cell* thisCell = &m_board[currPos.row][currPos.col];
		thisCell->m_row = currPos.row;
		thisCell->m_column = currPos.col;
		// Reset prevs and follow links
	

		thisCell->m_cellType = CELL_MEMBRANE;

		if (!justGetInflexionPoints)
		{
			if (prevCell)
			{
				switch (prevDir)
				{
				case DIR_LEFT:
				{
					thisCell->m_prevRight = prevCell;
					prevCell->m_left = thisCell;
				}
				break;
				case DIR_RIGHT:
				{
					thisCell->m_prevLeft = prevCell;
					prevCell->m_right = thisCell;
				}
				break;
				case DIR_DOWN:
				{
					thisCell->m_prevUp = prevCell;
					prevCell->m_down = thisCell;
				}
				break;
				case DIR_UP:
				{
					thisCell->m_prevDown = prevCell;
					prevCell->m_up = thisCell;
				}
				break;
				}
			}
		}

		if (rootPos == currPos)
		{
			if (alreadyMetRoot)
				break;

			alreadyMetRoot = true;
		}

		prevDir = currDir;
		currPos += Cell::DIR_OFFSET[currDir];
		const DIRECTION newDir = Cell::getDirectionFromSymbol(m_board[currPos.row][currPos.col].m_symbol);
		if (newDir != currDir)
		{
			
			currDir = newDir;
			outInflexionPositions.push_back(currPos);
		}

		prevCell = thisCell;
	} while (true); // rootPos != currPos); // When we get back to the root, stop

	// Clear all prev items from root. It shouldn't have any prevs !
	if (!justGetInflexionPoints)
	{
		Cell* rootCell = &m_board[rootPos.row][rootPos.col];
		for (int i = 0; i < DIR_COUNT; i++)
			(*rootCell->m_previousByDir)[i] = nullptr;
	}
}

void BoardObject::internalCreateRowsLinks(const int middleCol, const int rowStart, const int rowEnd)
{
	// Try each row from start to end and see how many items are in the left and right sides
	// Then go recursively on each
	for (int row = rowStart; row <= rowEnd; row++)
	{
		Cell* startCell = &m_board[row][middleCol];
		startCell->m_row = row;
		startCell->m_column = middleCol;

		//assert(startCell->symbol == '2' || startCell->symbol == '7');

		const int numItemsInLeft	= getOccupiedItemsOnRow(row, middleCol, true, true);
		const int numItemsInRight	= getOccupiedItemsOnRow(row, middleCol, false, true);

		// Solve Left (4 symbol)
		{
			Cell* cellIter = startCell;

			for (int leftIter = 1; leftIter <= numItemsInLeft; leftIter++)
			{
				Cell* newCell = &m_board[row][middleCol - leftIter];
				assert(newCell->m_symbol == '4' || newCell->m_symbol == EMPTY_SYMBOL);
				newCell->m_right = cellIter;
				cellIter->m_prevLeft = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_LEFT);
			}
		}

		// Generate right (e symbol)
		{
			Cell* cellIter = startCell;
			for (int rightIter = 1; rightIter <= numItemsInRight; rightIter++)
			{
				Cell* newCell = &m_board[row][middleCol + rightIter];
				assert(newCell->m_symbol == 'e' || newCell->m_symbol == EMPTY_SYMBOL);
				newCell->m_left = cellIter;
				cellIter->m_prevRight = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_RIGHT);
			}
		}

		// Recursively generate columns over the newly left and right sequences
		internalCreateColsLinks(row, middleCol - numItemsInLeft, middleCol - 1);
		internalCreateColsLinks(row, middleCol + 1, middleCol + numItemsInRight);
	}
}

void BoardObject::internalCreateColsLinks(const int middleRow, const int colMin, const int colMax)
{
	// Go through each column and check the items filled in each
	for (int col = colMin; col <= colMax; col++)
	{
		Cell* startCell = &m_board[middleRow][col];
		startCell->m_row = middleRow;
		startCell->m_column = col;
		//assert(startCell->symbol == '4' || startCell->symbol == 'e');

		const int numItemsUp	= getOccupiedItemsOnCol(col, middleRow, false, true);
		const int numItemsDown	= getOccupiedItemsOnCol(col, middleRow, true, true);

		// Generate Up (7 symbol)
		{
			Cell* cellIter = startCell;
			for (int upIter = 1; upIter <= numItemsUp; upIter++)
			{
				Cell* newCell = &m_board[middleRow - upIter][col];
				assert(newCell->m_symbol == '7' || newCell->m_symbol == EMPTY_SYMBOL);
				newCell->m_down = cellIter;
				cellIter->m_prevUp = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_UP);
			}
		}

		// Generate down (2 symbol)
		{
			Cell* cellIter = startCell;
			for (int downIter = 1; downIter <= numItemsDown; downIter++)
			{
				Cell* newCell = &m_board[middleRow + downIter][col];
				assert(newCell->m_symbol == '2' || newCell->m_symbol == EMPTY_SYMBOL);
				newCell->m_up = cellIter;
				cellIter->m_prevDown = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_DOWN);
			}
		}

		internalCreateRowsLinks(col, middleRow - numItemsUp, middleRow - 1);
		internalCreateRowsLinks(col, middleRow + 1, middleRow + numItemsDown);
	}
}

#else
void BoardObject::internalCreateLinks(Cell* prevCell, const DIRECTION dir, const int row, const int col)
{
	if (!isCoordinateValid(row, col))
		return;

	Cell* currCell = &m_board[row][col];
	if (currCell->isFree())
		return;

	// If rented, mark it accordingly
	if (currCell->isRented())
	{
		addRentedResource(currCell->m_symbol, TablePos(row, col));
	}

#if RUNMODE == DIRECTIONAL_MODE
	assert(false);
	/*
	if (dir == DIR_LEFT)
		prevCell->m_left = currCell;
	else if (dir == DIR_RIGHT)
		prevCell->m_right = currCell;
	else if (dir == DIR_UP)
		prevCell->m_ 
	else if (dir == DIR_DOWN)
	*/

#else
	currCell->m_parent = prevCell;
	if (dir == DIR_LEFT)
		prevCell->m_left = currCell;
	else
		prevCell->m_down = currCell;
#endif

	currCell->m_distanceToRoot = prevCell->m_distanceToRoot + 1;
	currCell->m_row = row;
	currCell->m_column = col;

	// THey must have the same board view now if it was reset
	if (currCell->m_boardView == nullptr)
	{
		currCell->m_boardView = prevCell->m_boardView;
	}

#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	internalCreateLinks(currCell, DIR_LEFT, row, col - 1);
	internalCreateLinks(currCell, DIR_DOWN, row + 1, col);
}
#endif

void BoardObject::copyDataFrom(const BoardObject& other)
{
	// If this is the same object, don't do anything !
	for (int i = 0; i < MAX_ROWS; i++)
		for (int j = 0; j < MAX_ROWS; j++)
		{
			m_board[i][j] = other.m_board[i][j];

			// Set the parent board
			m_board[i][j].m_boardView = this;
		}

#if RUNMODE == DIRECTIONAL_MODE
	setRootLocation(other.m_rootRow, other.m_rootCol);
#endif

	getRootCell()->initFlowStatistics(g_simulationTicksForDataFlowEstimation);

	updateInternalCellsInfo();	// Without updating the links, old ones will remain in place which is totally wrong :)

	m_garbageCollectedResources = other.m_garbageCollectedResources;

	if (this != &other)
	{
		m_posToSourceMap.clear();
	}

	m_posToSourceMap.insert(other.m_posToSourceMap.begin(), other.m_posToSourceMap.end());

	m_rowGenerator = other.m_rowGenerator;
	m_colGenerator = other.m_colGenerator;
	m_numTicksRemainingToUpdateSources = other.m_numTicksRemainingToUpdateSources;
}

BoardObject::BoardObject()
	: m_colGenerator(nullptr)
	, m_rowGenerator(nullptr)
	, m_numTicksRemainingToUpdateSources(0)
{
#if RUNMODE == DIRECTIONAL_MODE
	const int rootColumn = rand() % (MAX_COLS / 3);
	const int rootRow = rand() % (MAX_ROWS / 3);
	setRootLocation(rootRow, rootColumn);
#endif

}

BoardObject::BoardObject(const BoardObject& other)
{
	copyDataFrom(other);
}

void BoardObject::operator=(const BoardObject& other)
{
	copyDataFrom(other);
}

void BoardObject::resetCells(const bool resetSymbolsToo /*= true*/)
{
	for (int i = 0; i < MAX_ROWS; i++)
	{
		for (int j = 0; j < MAX_COLS; j++)
		{
			m_board[i][j].reset(resetSymbolsToo);
		}
	}
}

void BoardObject::reset(const bool withoutStatistics, const bool resetSymbolsToo)
{
	resetCells(resetSymbolsToo);

	m_posToSourceMap.clear();
	m_numTicksRemainingToUpdateSources = g_powerChangeFrequency;

	for (auto& keyValue : m_garbageCollectedResources)
	{
		m_garbageCollectedResources[keyValue.first] = 0;
	}

	if (!withoutStatistics)
	{
		getRootCell()->initFlowStatistics(g_simulationTicksForDataFlowEstimation);
	}
}

float BoardObject::doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(const int ticksToSimulate)
{
	this->doDataFlowSimulation_serial(1, false);
	const float baselineCurrentFlow = this->getLastSimulationAvgDataFlowPerUnit();
	this->retractLastSimTickFlowRecord(); // Retract it since we don't propagate changes

#if RUNMODE == DIRECTIONAL_MODE
	Cell& cellBelowRoot = getRootCell()->m_boardView->m_board[m_rootRow - 1][m_rootCol];
	cellBelowRoot.resetCapacityUsedInSubtree();
#else
	assert(false && " Not implemented yet");
#endif

	return baselineCurrentFlow;
}

void BoardObject::doDataFlowSimulation_serial(const int ticksToSimulate, const bool isRealTick, const bool considerForStatistics /*=true*/)
{
	Cell* root = getRootCell();
	root->beginSimulation();

	for (int i = 0; i < ticksToSimulate; i++)
	{
		simulateTick_serial(considerForStatistics);

		if (isRealTick)
		{
			updateSourcesPower();
		}
	}
}

void BoardObject::getMembraneCellsChildren(std::vector<Cell *>& outList, const CellType targetCellType /* = CELL_NOTSET */)
{
	outList.clear();

	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);

	for (Cell* memCell : membraneCells)
	{
		Cell* childrenList[DIR_COUNT];
		memCell->fillChildrenList(childrenList);

		// Iterate over its external children and simulate tick to capture the flow out of them since they contains the leaf nodes
		for (int childIter = 0; childIter < DIR_COUNT; childIter++)
		{
			Cell* child = childrenList[childIter];
			if (child == nullptr || (targetCellType != CELL_NOTSET && child->m_cellType != targetCellType))
				continue;

			outList.push_back(child);
		}
	}
}

void BoardObject::collectAllNodesFromRoot(const Cell* subtreeRoot, std::vector<Cell *>& outList, const CellType targetCellType /* = CELL_NOTSET */)
{

	Cell* childrenList[DIR_COUNT];
	subtreeRoot->fillChildrenList(childrenList);

	// Iterate over its external children and simulate tick to capture the flow out of them since they contains the leaf nodes
	for (int childIter = 0; childIter < DIR_COUNT; childIter++)
	{
		Cell* child = childrenList[childIter];
		if (child == nullptr || (targetCellType != CELL_NOTSET && (child->m_cellType != targetCellType)))
			continue;

		outList.push_back(child);
		collectAllNodesFromRoot(child, outList, targetCellType);
	}
}

void BoardObject::getMembraneCells(std::vector<Cell*>& membraneCells)
{
	std::vector<TablePos> inflexionPoints;
	getInflexionPointAndConnectMembrane(inflexionPoints, true);
	for (int inflexionIter = 0; inflexionIter < inflexionPoints.size() - 1; inflexionIter++)
	{
		const TablePos& A = inflexionPoints[inflexionIter];
		const TablePos& B = inflexionPoints[inflexionIter + 1];
		const TablePos dir = get2DNormDir(A, B);

		// Iterating between all cells inside membrane (along inflexion points)
		for (TablePos iter = A; iter != B; iter += dir)
		{
			Cell* memCell = &m_board[iter.row][iter.col];
			membraneCells.push_back(memCell);
		}
	}
}

void BoardObject::simulateTick_serial(const bool considerForStatistics /* = true */)
{
	// Step 0: Find how much flow each leaf (capture) cell will get from the available sources
	SimulationContext simContext;
	fillSimulationContext(simContext);

	// If not directional mode, easy step: capture from root only
#if RUNMODE != DIRECTIONAL_MODE
	Cell* root = getRootCell();
	root->simulateTick_serial(simContext);
#else
	// In directional mode:
	float totalDonatedFlow = 0.0f;

	std::set<Cell*> interiorSubtrees; // THis will store all interior sub-trees roots

	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);

	// Step 1: take membrane nodes and find how much each can capture from their attached external trees
	for (Cell* memCell : membraneCells)
	{
		// Get the child list of the membrane cell
		{
			Cell* childrenList[DIR_COUNT];
			memCell->fillChildrenList(childrenList);

			// Iterate over its external children and simulate tick to capture the flow out of them since they contains the leaf nodes
			for (int childIter = 0; childIter < DIR_COUNT; childIter++)
			{
				Cell* child = childrenList[childIter];
				if (child == nullptr || child->m_cellType != CELL_EXTERIOR)
					continue;

				child->simulateTick_serial(simContext);
			}

			// Now each external child has some buffered captured data. Take proportionally from each
			// and add the flow to this node
			float maxCapAllowed = memCell->getRemainingCap();
			memCell->captureFromChildren(maxCapAllowed, CELL_EXTERIOR);
		}		

		// If no cap to donate further, continue
		{
			if (floatEqual(memCell->getCurrentBufferedCap(), 0.0f))
				continue;
		}

		// Step 2: for each membrane node that has a positive flow, iterate circularly until we either consume the flow
		// by sending it to interior cells or when it gets back to that node.
		Cell *nextCell = memCell;
		do
		{
			Cell* childrenList[DIR_COUNT];
			nextCell->fillChildrenList(childrenList);

			// Donate the flow as much as possible to the internal children nodes
			for (int childIter = 0; childIter < DIR_COUNT; childIter++)
			{
				Cell* child = childrenList[childIter];
				if (child == nullptr || child->m_cellType != CELL_INTERIOR)
					continue;

				if (interiorSubtrees.find(child) == interiorSubtrees.end())
				{
					interiorSubtrees.insert(child);
				}

				const float currentCapAvailable = memCell->getCurrentBufferedCap();
				const float capUsed = child->donateFlow(currentCapAvailable);
				assert(capUsed <= currentCapAvailable);
				totalDonatedFlow += capUsed;

				memCell->subtractData(capUsed);
				if (floatEqual(memCell->getCurrentBufferedCap(), 0.0f))
					break;
			}

			// Try to send to next cell in the membrane if internal buffered flow wasn't consumed yet
			{				
				if (floatEqual(memCell->getCurrentBufferedCap(), 0.0f))
					break;
			}

			Cell* nextCellInMembrane = nullptr;
			{
				Cell* followersList[DIR_COUNT];
				nextCell->fillFollowersList(followersList);

				for (int followersIter = 0; followersIter < DIR_COUNT; followersIter++)
				{
					Cell* follower = followersList[followersIter];
					if (follower == nullptr || follower->m_cellType != CELL_MEMBRANE)
						continue;

					assert(nextCellInMembrane == nullptr && "There are more next cells in membrane !!");
					nextCellInMembrane = follower;
				}
			}
			assert(nextCellInMembrane && "Couldn't find a next item in membrane !!");
			nextCell = nextCellInMembrane;
		} while (nextCell != memCell);
	}

	// Reset the "bandwidth" (buffered flow) from the interior subtrees
	for (Cell* cell : interiorSubtrees)
	{
		cell->resetCapacityUsedInSubtree();
	}

	// Add the donated flow to the root cell which is the statistics collector
	if (considerForStatistics)
	{
		Cell* root = getRootCell();
		root->addNewFlowRecord(totalDonatedFlow);
	}
#endif
}

void BoardObject::runGarbageCollector(const float threshold, std::ostream& outStream)
{
	// Check the exterior roots (child of membrane) only, since removing subtrees from them would destabilize even more their flow
	// If a subroot doesn't meet the threshold defined it will be garbed collected
	std::vector<Cell*> exteriorRoots;
	getMembraneCellsChildren(exteriorRoots, CELL_EXTERIOR);

	std::vector<Cell*> interiorRoots;
	getMembraneCellsChildren(interiorRoots, CELL_INTERIOR);
	const bool noInteriorRoots = interiorRoots.empty();

	// Fill simulation context to decide how much each external leaf will get
	SimulationContext simContext;
	fillSimulationContext(simContext);

	for (Cell* root : exteriorRoots)
	{
		// Capture first to see what we can get in this moment
		root->simulateTick_serial(simContext);
		const float bufferedCap = root->getCurrentBufferedCap();

		root->resetCapacityUsedInSubtree(); // Reset the used capacity

		if (noInteriorRoots || (bufferedCap - root->getCachedEnergyConsumedStat() < threshold))
		{
			outStream << "The subtree starting at (" << root->m_row << "," << root->m_column << ") is being garbage collected. Flow: " << bufferedCap << " consumed energy: " << root->getCachedEnergyConsumedStat() << " threshold: " << threshold << std::endl;
			garbageCollectSubtree(root);
		}
	}

	// Update the internal cells info just to be safe for links creation :)
	updateInternalCellsInfo();
}

void BoardObject::garbageCollectSubtree(Cell* root)
{
	m_garbageCollectedResources[root->m_symbol]++;

	// Iterate over all subtrees and delete the cells and links
	Cell* childrenList[DIR_COUNT];
	root->fillChildrenList(childrenList);

	// Garbage collect children
	for (int childIter = 0; childIter < DIR_COUNT; childIter++)
	{
		Cell* child = childrenList[childIter];
		if (child == nullptr)
			continue;

		assert(child->m_cellType == CELL_EXTERIOR && "I'm expecting only exterior cells here !!!");

		garbageCollectSubtree(child);
	}

	// Reset this node too
	root->reset();
}

void BoardObject::addPotentialNewCellsAround(std::vector<ResourceAllocatedEval>& validSet, const CellType cellType, const int row, const int col, Cell::UniversalHash2D& hash2D, const std::vector<char>& availableSymbolsToFill) const
{
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const int potentialRow = Cell::DIR_OFFSET[dirIter].row + row;
		const int potentialCol = Cell::DIR_OFFSET[dirIter].col + col;

		// Must be on table valid pos, free and not in hash already added
		// And a single valid neighbor (that is the one it is attached to)
		if (!isCoordinateValid(potentialRow, potentialCol) ||
			!m_board[potentialRow][potentialCol].isFree() ||
			hash2D.isCellSet(potentialRow, potentialCol) ||
			this->getNumNeighboors(potentialRow, potentialCol) > 1)
		{
			continue;
		}

		// If the parent cell is a membrane cell then the new position must be OUTSIDE membrane
		if (m_board[row][col].m_cellType == CELL_MEMBRANE && getCellTypeRelativeToMembrane(potentialRow, potentialCol) != cellType)
			continue;

		// Valid, add it to the set and move further
		hash2D.setCell(potentialRow, potentialCol);

		// Add for all available symbols, check later if valid
		for (const char availableSymbol : availableSymbolsToFill)
		{
			ResourceAllocatedEval rsc;
			rsc.pos = TablePos(potentialRow, potentialCol);
			rsc.symbol = availableSymbol;
			validSet.emplace_back(rsc);
		}
	}
}

CellType BoardObject::getCellTypeRelativeToMembrane(const int row, const int col) const
{
	const bool isInteriorMembraneRow = m_membraneBoundsPerRow[row].first <= col && col <= m_membraneBoundsPerRow[row].second;
	const bool isInteriorMembraneCol = m_membraneBoundsPerCol[col].first <= row && row <= m_membraneBoundsPerCol[col].second;

	if (isInteriorMembraneCol && isInteriorMembraneRow)
		return CELL_INTERIOR;
	if (m_board[row][col].m_cellType == CELL_MEMBRANE)
		return CELL_MEMBRANE;
	else
		return CELL_EXTERIOR;
}

template <typename T>
void augmentValue(T& min, T &max, const T value)
{
	if (min > value)
		min = value;

	if (max < value)
		max = value;
}

// We can't cut columns or row which are edges !
bool BoardObject::canCutColumn(const int column) const
{
	if (m_rootCol == column)
		return false;

	int numOccupiedItems = 0;
	for (int i = 0; i < MAX_ROWS; i++)
	{
		if (m_membraneBoundsPerRow[i].first == column ||
			m_membraneBoundsPerRow[i].second == column)
			return false;

		const Cell& cell = m_board[i][column];
		if (cell.isFree() == false && cell.m_cellType != CELL_MEMBRANE)
			return false;

		numOccupiedItems += m_board[i][column].isFree() ? 0 : 1;
	}

	return numOccupiedItems != 0;
}

bool BoardObject::canCutRow(const int row) const
{
	if (m_rootRow == row)
		return false;

	int numOccupiedItems = 0;
	for (int i = 0; i < MAX_COLS; i++)
	{
		if (m_membraneBoundsPerCol[i].first == row ||
			m_membraneBoundsPerCol[i].second == row)
			return false;

		const Cell& cell = m_board[row][i];
		if (cell.isFree() == false && cell.m_cellType != CELL_MEMBRANE)
			return false;

		numOccupiedItems += m_board[row][i].isFree() ? 0 : 1;
	}

	return numOccupiedItems != 0;
}

bool BoardObject::evaluateMembraneCut(membraneCutFunctorType func, const DIRECTION dirs[2], const int index, const float baselineFlowAvg, DIRECTION& outDir, float& outFlowDiff) const
{
	outFlowDiff = INVALID_FLOW;
	outDir = DIR_COUNT;

	for (int dirIter = 0; dirIter < 2; dirIter++)
	{
		DIRECTION cutDir = dirs[dirIter];

		if (g_verboseBestGatheredSolutions)
		{
			(*g_debugLogOutput) << " ---- Index: " << index << " dir: " << Cell::getDirString(cutDir) << std::endl;
		}
		
		// Copy the board and cut the membrane by shifting to the desired direction
		BoardObject newBoard = *this;
		(newBoard.*func)(index, cutDir, false);

		// Check if this is complaint with language stuff
		if (newBoard.isCompliantWithRowColPatterns() == false)
			continue;

		newBoard.updateBoardAfterSymbolsInit();

		newBoard.expandInternalTrees();

		// Try to add the garbaged items to improve the flow
		newBoard.expandExternalTrees();

		// Run the simulation on it and get result
		const float localFlowDiff = newBoard.doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1) - baselineFlowAvg;
		if (localFlowDiff > outFlowDiff)
		{
			outFlowDiff = localFlowDiff;
			outDir = cutDir;
		}
	}

	return (outDir != DIR_COUNT);	
}

// Returns true if any valid.
// Returns row != INVALID_POS for the row to be cut
// Returns col != INVALID_POS for the col to be cut
// Returns the flowBenefit of the best between row/col
bool BoardObject::evaluateMembraneOptimization(int& row, int &col, DIRECTION& dirToCut, float& flowBenefit)
{
	row = INVALID_POS;
	col = INVALID_POS;
	flowBenefit = INVALID_FLOW;

	float flowBenRow = INVALID_FLOW;
	float flowBenCol = INVALID_FLOW;
	DIRECTION dirRow = DIR_COUNT;
	DIRECTION dirCol = DIR_COUNT;

	// Init the membrane bounds
	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);
	updateMembraneBounds(membraneCells);

	// Obtain the baseline flow of this board
	const float baselineFlow = doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1);

	// Check each row, get the best one.
	{
		if (g_verboseBestGatheredSolutions)
		{
			(*g_debugLogOutput) << " -- Evaluating cutting on ROWS: " << std::endl;
		}


		for (int rowIter = m_membraneBoundsRows.first + 1; rowIter <= m_membraneBoundsRows.second - 1; rowIter++)
		{
			if (!canCutRow(rowIter))
				continue;

			DIRECTION outDir = DIR_COUNT;
			float outFlowDiff = INVALID_FLOW;
			const DIRECTION dirsToTry[2] = { DIR_UP, DIR_DOWN };
			if (evaluateMembraneCut(&BoardObject::cutMembraneByRow, dirsToTry, rowIter, baselineFlow, outDir, outFlowDiff))
			{
				if (g_verboseBestGatheredSolutions)
				{
					(*g_debugLogOutput) << "----- Eval row " << rowIter << " dir " << Cell::getDirString(outDir) << " - " << outFlowDiff << std::endl;
				}

				if (outFlowDiff > flowBenRow)
				{
					flowBenRow = outFlowDiff;
					row = rowIter;
					dirRow = outDir;
				}
			}
		}
	}

	// Check each col, get the best one
	{
		if (g_verboseBestGatheredSolutions)
		{
			(*g_debugLogOutput) << " -- Evaluating cutting on COLUMNS: " << std::endl;
		}


		for (int colIter = m_membraneBoundsCols.first + 1; colIter <= m_membraneBoundsCols.second - 1; colIter++)
		{
			if (!canCutColumn(colIter))
				continue;

			DIRECTION outDir = DIR_COUNT;
			float outFlowDiff = INVALID_FLOW;
			const DIRECTION dirsToTry[2] = { DIR_LEFT, DIR_RIGHT };
			if (evaluateMembraneCut(&BoardObject::cutMembraneByCol, dirsToTry, colIter, baselineFlow, outDir, outFlowDiff))
			{
				if (g_verboseBestGatheredSolutions)
				{
					(*g_debugLogOutput) << "----- Eval col " << colIter << " dir " << Cell::getDirString(outDir) << " - " << outFlowDiff << std::endl;
				}

				if (outFlowDiff > flowBenCol)
				{
					flowBenCol = outFlowDiff;
					col = colIter;
					dirCol = outDir;
				}
			}
		}
	}

	// Compare and output only the best result
	if (row == INVALID_POS && col == INVALID_POS)
	{
		return false;
	}
	else if (row == INVALID_POS && col != INVALID_POS)
	{
		flowBenefit = flowBenCol;
		dirToCut = dirCol;
		return true;
	}
	else if (row != INVALID_POS && col == INVALID_POS)
	{
		flowBenefit = flowBenRow;
		dirToCut = dirRow;
		return true;
	}
	else
	{
		if (flowBenRow > flowBenCol)
		{
			flowBenefit = flowBenRow;
			dirToCut = dirRow;
			col = INVALID_POS;
		}
		else
		{
			flowBenefit = flowBenCol;
			dirToCut = dirCol;
			row = INVALID_POS;
		}

		return true;
	}

	return false;
}


void BoardObject::updateMembraneBounds(std::vector<Cell*>& membraneCells)
{
	auto invalidMinMaxCol = std::make_pair(MAX_COLS + 1, -1);
	// Init min / max first
	for (int i = 0; i < MAX_ROWS; i++)
	{
		m_membraneBoundsPerRow[i] = invalidMinMaxCol;
	}

	auto invalidMinMaxRow = std::make_pair(MAX_ROWS + 1, -1);
	for (int i = 0; i < MAX_COLS; i++)
	{
		m_membraneBoundsPerCol[i] = invalidMinMaxRow;
	}

	for (const Cell* cell : membraneCells)
	{
		const int row = cell->m_row;
		const int col = cell->m_column;

		augmentValue(m_membraneBoundsPerRow[row].first, m_membraneBoundsPerRow[row].second, col);
		augmentValue(m_membraneBoundsPerCol[col].first, m_membraneBoundsPerCol[col].second, row);

		augmentValue(m_membraneBoundsCols.first, m_membraneBoundsCols.second, col);
		augmentValue(m_membraneBoundsRows.first, m_membraneBoundsRows.second, row);
	}
}

// Expands the external trees to improve overall flow
void BoardObject::expandExternalTrees()
{
	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);

	updateMembraneBounds(membraneCells);


	// Step 1: collect all nodes starting from membrane nodes and exterior subtrees
	std::vector<std::pair<Cell*, std::vector<Cell*>>> occupiedCellsList; // occupiedCellList[M] = list => the subtree nodes below membrane node M
	for (Cell* memCell : membraneCells)
	{
		std::vector<Cell*> occupiedCellsBelowMemCell;
		occupiedCellsBelowMemCell.push_back(memCell); // Add the membrane cell too 
		collectAllNodesFromRoot(memCell, occupiedCellsBelowMemCell, CELL_EXTERIOR);
		occupiedCellsList.emplace_back(std::make_pair(memCell, occupiedCellsBelowMemCell));
	}

	// Step 2: identify all available positions near the positions at Step 1 - valid are positions inside table which have a SINGLE valid neighbor on table
	// Let this set be ValidSet
	Cell::UniversalHash2D hash2D;


	// Fill the list of different symbols that can be used (the ones garbage collected)
	std::vector<char> availableSymbolsToFill;
	for (auto& entry : m_garbageCollectedResources)
	{
		if (entry.second > 0)
			availableSymbolsToFill.push_back(entry.first);
	}

	std::vector<ResourceAllocatedEval> validSet;
	for (std::pair<Cell*, std::vector<Cell*>>& cellsBelowParent : occupiedCellsList)
	{
		Cell* parent = cellsBelowParent.first;
		const std::vector<Cell*>& cellsList = cellsBelowParent.second;

		for (const Cell* cell: cellsList)
		{
			const int row = cell->m_row;
			const int column = cell->m_column;

			addPotentialNewCellsAround(validSet, CELL_EXTERIOR, row, column, hash2D, availableSymbolsToFill);
		}
	}

	if (false)
	{
		printBoard(std::cout);
	}

	float baselineCurrentFlow = doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1);

	do {
		int argmax = -1;
		float bestFlowBenefit = INVALID_FLOW;

		// Step 3: for each such position compute the improvements added if we put something there. Continue moving in the most promising direction.
		// Update ValidSet after each new position added.
		for (int targetsIter = 0; targetsIter < validSet.size(); targetsIter++)
		{
			ResourceAllocatedEval& resourceEval = validSet[targetsIter];
			resourceEval.flowBenefit = INVALID_FLOW;

			// If we don't have remaining symbols of this type, don't do anything
			if (m_garbageCollectedResources[resourceEval.symbol] == 0)
			{
				continue;
			}

			// Copy board, apply the symbol, check if valid then compute the avg flow for one tick to decide		
			BoardObject* newBoard = new BoardObject();
			*newBoard = *this;

			const int targetRow = resourceEval.pos.row;
			const int targetCol = resourceEval.pos.col;

			// Apply symbol and set links to neighbors
			newBoard->setNewCell(targetRow, targetCol, resourceEval.symbol, CELL_EXTERIOR, false);

			if (newBoard->isCompliantWithRowColPatterns(targetRow, targetCol) == false)
			{
				delete newBoard;
				continue;
			}

			newBoard->doDataFlowSimulation_serial(1, false);
			const float flowRes = newBoard->getLastSimulationAvgDataFlowPerUnit();
			resourceEval.flowBenefit = flowRes - baselineCurrentFlow;

			if (g_verboseLocalSolutions)
			{
				(*g_debugLogOutput) << " -------- Expand external tree on Pos (" << targetRow << "," << targetCol << ")" << " Sym: " << resourceEval.symbol <<" Benefit: " << resourceEval.flowBenefit << std::endl;
			}

			if (resourceEval.flowBenefit > 0.0f && resourceEval.flowBenefit > bestFlowBenefit)
			{
				bestFlowBenefit = resourceEval.flowBenefit;
				argmax = targetsIter;
			}

			delete newBoard;
		}

		// TODO: verbose argmax !!
		if (bestFlowBenefit > 0.0f && argmax > 0)
		{
			// Set this symbol on board
			const ResourceAllocatedEval& rsc = validSet[argmax];
			Cell& newAddedCell = m_board[rsc.pos.row][rsc.pos.col];
			this->setNewCell(rsc.pos.row, rsc.pos.col, rsc.symbol, CELL_EXTERIOR, true); // Definitive this time !

			if (g_verboseBestGatheredSolutions)
			{
				(*g_debugLogOutput) <<" ------- ExpandExternalTree Best Pos: (" << rsc.pos.row << "," << rsc.pos.col << ")" << " Sym: " << rsc.symbol << " Benefit: " << bestFlowBenefit << std::endl;
			}

			// Consume the symbol committed from the map
			assert(m_garbageCollectedResources[rsc.symbol] > 0);
			m_garbageCollectedResources[rsc.symbol]--;

			// Update the potential list of positions
			addPotentialNewCellsAround(validSet, CELL_EXTERIOR, rsc.pos.row, rsc.pos.col, hash2D, availableSymbolsToFill);

			// Move on to select another one !

			// Update first the base flow value
			baselineCurrentFlow += bestFlowBenefit;
		}
		else
		{
			// No more moves, exit !
			break;
		}

	} while (true);
}

void BoardObject::expandInternalTrees()
{
	// IDEA: collect all potential nodes around membrane nodes or existing internal nodes that are NOT LEAF and extend them with something that would be a leaf
	// to have more leafs and collect more flow

	const float baselineCurrentFlow = doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1);

	// Try this until adding a new leaf does not further increase the flow
	while (true)
	{
		std::vector<Cell*> interiorRoots;
		getMembraneCellsChildren(interiorRoots, CELL_INTERIOR);
		const bool noInteriorRoots = interiorRoots.empty();

		// PART 1: collect potential locations
		//-------------------------------------------------------
		std::vector<Cell*> membraneCells;
		getMembraneCells(membraneCells);

		updateMembraneBounds(membraneCells);

		// Step 1: collect all nodes starting from membrane nodes and exterior subtrees
		std::vector<std::pair<Cell*, std::vector<Cell*>>> occupiedCellsList; // occupiedCellList[M] = list => the subtree nodes below membrane node M
		for (Cell* memCell : membraneCells)
		{
			std::vector<Cell*> occupiedCellsBelowMemCell;
			occupiedCellsBelowMemCell.push_back(memCell); // Add the membrane cell too 
			collectAllNodesFromRoot(memCell, occupiedCellsBelowMemCell, CELL_INTERIOR);
			occupiedCellsList.emplace_back(std::make_pair(memCell, occupiedCellsBelowMemCell));
		}

		// Step 2: identify all available positions near the positions at Step 1 - valid are positions inside table which have a SINGLE valid neighbor on table
		// Let this set be ValidSet
		Cell::UniversalHash2D hash2D;

		// Fill the list of different symbols that can be used (the ones garbage collected)
		std::vector<char> availableSymbolsToFill;
		for (auto& entry : m_garbageCollectedResources)
		{
			if (entry.second > 0)
				availableSymbolsToFill.push_back(entry.first);
		}

		std::vector<ResourceAllocatedEval> validSet;
		for (std::pair<Cell*, std::vector<Cell*>>& cellsBelowParent : occupiedCellsList)
		{
			Cell* parent = cellsBelowParent.first;
			const std::vector<Cell*>& cellsList = cellsBelowParent.second;

			for (const Cell* cell : cellsList)
			{
				const int row = cell->m_row;
				const int column = cell->m_column;

				// Do not add cells around leafs !!
				if (cell->isInteriorLeaf())
					continue;

				addPotentialNewCellsAround(validSet, CELL_INTERIOR, row, column, hash2D, availableSymbolsToFill);
			}
		}

		// Part 2: try each potential cell and get the best one
		int argmax = -1;
		float bestFlowBenefit = INVALID_FLOW;

		// Step 3: for each such position compute the improvements added if we put something there. Continue moving in the most promising direction.
		// Update ValidSet after each new position added.
		for (int targetsIter = 0; targetsIter < validSet.size(); targetsIter++)
		{
			ResourceAllocatedEval& resourceEval = validSet[targetsIter];
			resourceEval.flowBenefit = INVALID_FLOW;

			// If we don't have remaining symbols of this type, don't do anything
			if (m_garbageCollectedResources[resourceEval.symbol] == 0)
			{
				continue;
			}

			// Copy board, apply the symbol, check if valid then compute the avg flow for one tick to decide		
			BoardObject* newBoard = new BoardObject();
			*newBoard = *this;

			const int targetRow = resourceEval.pos.row;
			const int targetCol = resourceEval.pos.col;

			// Apply symbol and set links to neighbors
			newBoard->setNewCell(targetRow, targetCol, resourceEval.symbol, CELL_INTERIOR, false);

			if (newBoard->isCompliantWithRowColPatterns(targetRow, targetCol) == false)
			{
				delete newBoard;
				continue;
			}

			newBoard->doDataFlowSimulation_serial(1, false);
			const float flowRes = newBoard->getLastSimulationAvgDataFlowPerUnit();
			resourceEval.flowBenefit = flowRes - baselineCurrentFlow;

			if (g_verboseLocalSolutions)
			{
				(*g_debugLogOutput) << " -------- Expand external tree on Pos (" << targetRow << "," << targetCol << ")" << " Sym: " << resourceEval.symbol << " Benefit: " << resourceEval.flowBenefit << std::endl;
			}

			const bool isFlowBetter = (resourceEval.flowBenefit > 0.0f && resourceEval.flowBenefit > bestFlowBenefit);
			if (noInteriorRoots || isFlowBetter)
			{
				bestFlowBenefit = resourceEval.flowBenefit;
				argmax = targetsIter;

				if (!isFlowBetter)
				{
					// Don't evaluate others..continue just
					break;
				}
			}

			delete newBoard;
		}

		// TODO: verbose argmax !!
		if ((bestFlowBenefit > 0.0f || noInteriorRoots) && argmax > 0)
		{
			// Set this symbol on board
			const ResourceAllocatedEval& rsc = validSet[argmax];
			Cell& newAddedCell = m_board[rsc.pos.row][rsc.pos.col];
			this->setNewCell(rsc.pos.row, rsc.pos.col, rsc.symbol, CELL_INTERIOR, true); // Definitive this time !

			if (g_verboseBestGatheredSolutions)
			{
				(*g_debugLogOutput) << " ------- ExpandInternalTree Best Pos: (" << rsc.pos.row << "," << rsc.pos.col << ")" << " Sym: " << rsc.symbol << " Benefit: " << bestFlowBenefit << std::endl;
			}

			// Consume the symbol committed from the map
			assert(m_garbageCollectedResources[rsc.symbol] > 0);
			m_garbageCollectedResources[rsc.symbol]--;
		}
		else
		{
			// No more moves, exit !
			break;
		}
	}
}

void BoardObject::setNewCell(const int targetRow, const int targetCol, const char symbol, const CellType cellType, const bool definitive)
{
	// Set the symbol
	Cell& newCell = m_board[targetRow][targetCol];
	newCell.setSymbol(symbol);

	// Connect the links and distance to root 
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const int neighbRow = Cell::DIR_OFFSET[dirIter].row + targetRow;
		const int neighbCol = Cell::DIR_OFFSET[dirIter].col + targetCol;

		if (!isCoordinateValid(neighbRow, neighbCol))
			continue;

		Cell& neighbCell = m_board[neighbRow][neighbCol];
		if (neighbCell.isFree())
			continue;

		// Need to connect with it !
		(*neighbCell.m_previousByDir[getOppositeDirection((DIRECTION)dirIter)]) = &newCell;
		(*newCell.m_followersByDir[dirIter]) = &neighbCell;

		newCell.m_distanceToRoot = neighbCell.m_distanceToRoot + 1;
		newCell.m_cellType = cellType;
		newCell.m_row = targetRow;
		newCell.m_column = targetCol;
	}

	if (definitive)
	{
		// Broadcast the structure and call discovery
		Cell* rootCell = getRootCell();
		rootCell->onMsgDiscoverStructure(rootCell->m_row, rootCell->m_column, 0);
		rootCell->onRootMsgBroadcastStructure(this);
	}
	else
	{
		newCell.m_boardView = nullptr; // NullFor now
	}
}

void BoardObject::updateSourcesPower()
{
	if (!variableSourcesPower)
		return;

	// For each source
	for (auto& it : m_posToSourceMap)
	{
		SourceInfo& srcInfo = it.second;
		// Update current power according to their targets
		{
			float amountToAdd = srcInfo.getTarget() - srcInfo.getPower();
			const float sgn = amountToAdd > 0.0f ? 1.0f : -1.0f;

			const float absAmountToAdd = std::abs(amountToAdd);
			amountToAdd = sgn * std::min(absAmountToAdd, g_maxPowerVelocityPerTick);
			srcInfo.setCurrentPower(srcInfo.getPower() + amountToAdd);

			propagateSourceEvent(Cell::EVENT_SOURCE_MODIFY, it.first, srcInfo, false);
		}
	}

	// Update the sources' power target
	m_numTicksRemainingToUpdateSources--;
	if (m_numTicksRemainingToUpdateSources == 0)
	{
		m_numTicksRemainingToUpdateSources = g_powerChangeFrequency;

		for (auto& it : m_posToSourceMap)
		{
			SourceInfo& srcInfo = it.second;

			// Time expired, update sources' targets
			srcInfo.setPowerTarget((float)randRange(g_minPowerForWirelessSource, g_maxPowerForWirelessSource));
		}
	}
}


float BoardObject::getLastSimulationAvgDataFlowPerUnit() const
{
	return getRootCell()->getAvgFlow();
}

bool BoardObject::isNumberOfCharactersGood() const
{
	assert(false);
	return false;
}

bool BoardObject::isCompliantWithRowColPatterns(int onlyTestRow, int onlyTestCol, TablePos* outWrongPos ) const
{
	// Check the language on rows
	for (int row = 0; row < MAX_ROWS; row++)
	{
		if (onlyTestRow != INVALID_POS && row != onlyTestRow)
			continue;

		int start = INVALID_POS;
		for (int col = 0; col < MAX_COLS; col++)
		{
			const bool isFreeCell = m_board[row][col].isFree();
			if (isFreeCell)
			{
				if (start != INVALID_POS) // end of a contiguous row
				{
					if (checkRow(row, start, col - 1) == false)
					{
						if (outWrongPos)
							*outWrongPos = TablePos(row, start);

						return false;
					}

					start = INVALID_POS;
				}
			}
			else if (start == INVALID_POS) // Start of a new row
				start = col;
		}

		if (start != INVALID_POS)
		{
			if (checkRow(row, start, MAX_COLS - 1) == false)
			{
				if (outWrongPos)
					*outWrongPos = TablePos(row, start);

				return false;
			}
		}
	}

	// Check the language on columns
	for (int col = 0; col < MAX_COLS; col++)
	{
		if (onlyTestCol != INVALID_POS && col != onlyTestCol)
			continue;

		int start = INVALID_POS;
		for (int row = 0; row < MAX_ROWS; row++)
		{
			const bool isFreeCell = m_board[row][col].isFree();
			if (isFreeCell)
			{
				if (start != INVALID_POS) // end of a contig col
				{
					if (checkCol(col, start, row - 1) == false)
					{
						if (outWrongPos)
							*outWrongPos = TablePos(start, col);

						return false;
					}

					start = INVALID_POS;
				}
			}
			else if (start == INVALID_POS) // Start of a new row
				start = row;
		}

		if (start != INVALID_POS)
		{
			if (checkCol(col, start, MAX_ROWS - 1) == false)
			{
				if (outWrongPos)
					*outWrongPos = TablePos(start, col);

				return false;
			}
		}
	}

	return true;
}

bool BoardObject::tryApplySubtree(const int row, const int col, const SubtreeInfo& subtree, const bool checkPositions, const bool checkLanguage)
{
	if (checkPositions)
	{
		const bool res = canPasteSubtreeAtPos_noLangCheck(row, col, subtree);
		if (res == false)
			return false;
	}

	// Set all the symbols in the subtree according to the offsets
	for (const OffsetAndSymbol& offsetAndSymbol : subtree.m_offsets)
	{
		const int rowAp = row + offsetAndSymbol.rowOff;
		const int colAp = col + offsetAndSymbol.colOff;
		assert(isCoordinateValid(rowAp, colAp));

		m_board[rowAp][colAp].setSymbol(offsetAndSymbol.symbol);
		
		if (offsetAndSymbol.isRented)
		{
			m_board[rowAp][colAp].setAsRented();
		}
	}

	// Update the internal board info - rows, columns, links and distance to root currently
	updateInternalCellsInfo();

	if (checkLanguage)
	{
		const bool res = isCompliantWithRowColPatterns();
		if (res == false)
			return false;
	}

	return true;
}

void BoardObject::cutSubtree(const int row, const int col, SubtreeInfo& subtree)
{
	const Cell& thisCell = m_board[row][col];

	// Gather all cells first in the subtree data structure
	internalCutSubtree(thisCell, 0, 0, subtree);

	// Remove the content from this board
	for (const OffsetAndSymbol& offsetAndSymbol : subtree.m_offsets)
	{
		Cell& targetCell = m_board[row + offsetAndSymbol.rowOff][col + offsetAndSymbol.colOff];

		// Disable connection to its parent then reset
#if RUNMODE == DIRECTIONAL_MODE
		assert(false);
#else
		Cell* parentCell = targetCell.m_parent;
		if (parentCell)
		{
			if (parentCell->m_left == &targetCell)
				parentCell->m_left = nullptr;

			if (parentCell->m_down == &targetCell)
				parentCell->m_down = nullptr;
		}
#endif

		m_board[row + offsetAndSymbol.rowOff][col + offsetAndSymbol.colOff].reset();
	}
}

void BoardObject::internalCutSubtree(const Cell& currCell, const int rowOff, const int colOff, SubtreeInfo& outSubtree)
{
#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	assert(currCell.isFree() == false && "cutting empty subtrees are not allowed !");

	OffsetAndSymbol newEntry = OffsetAndSymbol(rowOff, colOff, currCell.m_symbol, currCell.isRented());
	outSubtree.add(newEntry);

	if (currCell.m_left)
		internalCutSubtree(*currCell.m_left, rowOff, colOff - 1, outSubtree);

	if (currCell.m_down)
		internalCutSubtree(*currCell.m_down, rowOff + 1, colOff, outSubtree);
}

// 
bool BoardObject::canPasteSubtreeAtPos_noLangCheck(const int targetRow, const int targetCol, const SubtreeInfo& subTree) const
{
	// Check if we can paste the subtree there only considering their positions
	const bool isTargetCellFree = m_board[targetRow][targetCol].isFree();
	const bool canGlueAbove = targetRow > 0 && m_board[targetRow - 1][targetCol].isFree() == false;
	const bool canGlueRight = (targetCol < MAX_COLS - 1) && m_board[targetRow][targetCol + 1].isFree() == false;
	const bool canGlueToOneSide = canGlueAbove || canGlueRight;
	
	if (isTargetCellFree == false || canGlueToOneSide == false)
	{
		return false;
	}

	for (const OffsetAndSymbol& offsetAndSymbol : subTree.m_offsets)
	{
		const int rowAp = targetRow + offsetAndSymbol.rowOff;
		const int colAp = targetCol + offsetAndSymbol.colOff;
		//assert(isCoordinateValid(rowAp, colAp));

		if (!isCoordinateValid(rowAp, colAp))
			return false;
		
		if (m_board[rowAp][colAp].isFree() == false)
			return false;
	}

	return true;
}


bool BoardObject::checkRow(const int row, const int startCol, const int endCol) const
{
	assert(isCoordinateValid(row, startCol) && isCoordinateValid(row, endCol) && startCol <= endCol);
	
	std::string testLocal;
	testLocal.reserve(endCol - startCol + 2);
	for (int colIter = startCol; colIter <= endCol; colIter++)
	{
		testLocal += m_board[row][colIter].m_symbol;
	}
	
	const bool res = std::regex_match(testLocal.c_str(), g_rowRXExpr);
	return res;
}

bool BoardObject::checkCol(const int col, const int startRow, const int endRow) const
{
	assert(isCoordinateValid(startRow, col) && isCoordinateValid(endRow, col) && startRow <= endRow);
	
	std::string testLocal;
	testLocal.reserve(endRow - startRow + 2);
	for (int rowIter = startRow; rowIter <= endRow; rowIter++)
	{
		testLocal += m_board[rowIter][col].m_symbol;
	}
	
	const bool res = std::regex_match(testLocal.c_str(), g_colRXExpr);
	return res;
}

void BoardObject::evaluatePositionsToMove(const int cellRow, const int cellCol, const SubtreeInfo& subtreeCut, AvailablePositionsToMove& outPos, int& outBestOptionIndex) const
{
	// For each position on the table, check if this subtree cut can be put in there
	const int min_row = std::abs(subtreeCut.minRowOffset);
	const int max_row = MAX_ROWS - subtreeCut.maxRowOffset;
	const int min_col = std::abs(subtreeCut.minColOffset);
	const int max_col = MAX_COLS - subtreeCut.maxColOffset;

	for (int rowIter = min_row; rowIter < max_row; rowIter++)
	{
		for (int colIter = min_col; colIter < max_col; colIter++)
		{
			// Same movement as initial cut ?
			if (rowIter == cellRow && colIter == cellCol)
				continue;

			// Test if we have something near to paste this subtree - We must have an item either in the upper side or right side
			//if (rowIter == 0 || colIter == MAX_COLS - 1)
			//	continue;

			// Check if we can paste the subtree there only considering their positions first
			if (canPasteSubtreeAtPos_noLangCheck(rowIter, colIter, subtreeCut) == false)
				continue;

			// TODO: optimize this !!! We are allocating and copy a lot of memory here !
			BoardObject boardWithNewSubtree = *this;
			if (boardWithNewSubtree.tryApplySubtree(rowIter, colIter, subtreeCut, false, true) == false)
				continue;

			// Simulate and get the average flow then send it to root
			boardWithNewSubtree.doDataFlowSimulation_serial(1);
			const float dataFlow = boardWithNewSubtree.getLastSimulationAvgDataFlowPerUnit();

			AvailablePosInfoAndDeltaScore posInfo;
			posInfo.col = colIter;
			posInfo.row = rowIter;
			posInfo.selectedColumn = cellCol;
			posInfo.selectedRow = cellRow;
			posInfo.score = dataFlow;
			outPos.push_back(posInfo);
		}
	}

	outBestOptionIndex = INVALID_POS;
	float bestScore = 0.0f; // FLT_MIN
	for (int iter = 0; iter < outPos.size(); iter++)
	{
		const AvailablePosInfoAndDeltaScore& posInfo = outPos[iter];
		if (outBestOptionIndex == INVALID_POS || posInfo.score > bestScore)
		{
			outBestOptionIndex = iter;
			bestScore = posInfo.score;
		}
	}
}

void BoardObject::printBoard(std::ostream& outStream)
{
	HANDLE hstdout = GetStdHandle(STD_OUTPUT_HANDLE);

	outStream << "Current board: " << endl;

	outStream << ' ' << ' ';
	for (uint j = 0; j < MAX_COLS; j++)
		outStream << ' ' << j%10 << ' ';

	outStream << endl;
	for (uint i = 0; i < MAX_ROWS; i++)
	{
		outStream << i%10 << ' ';

		for (uint j = 0; j < MAX_COLS; j++)
		{
			if (m_board[i][j].isFree())
			{
				outStream << ' ' << BOARD_SKIP_CHARACTER << ' ';
			}
			else
			{
				const CellType cellType = m_board[i][j].m_cellType;
				const bool membraneCell = cellType == CELL_MEMBRANE;
				const bool isInteriorTree = cellType == CELL_INTERIOR;
				const bool isExteriorTree = cellType == CELL_EXTERIOR;
				const bool isColorChanged = membraneCell || isInteriorTree || isExteriorTree;
				
				// Put red color if membrane
				if (isColorChanged)
				{
					if (membraneCell)
						SetConsoleTextAttribute(hstdout, 0x04);
					else if (isExteriorTree)
						SetConsoleTextAttribute(hstdout, 0x02);
					else if (isInteriorTree)
						SetConsoleTextAttribute(hstdout, 0x03);
				}

				outStream << ' ' << m_board[i][j].m_symbol << ' ';

				// Revert back to black
				if (isColorChanged)
					SetConsoleTextAttribute(hstdout, 0x0F); 
			}
		}

		outStream << std::endl;
	}

	outStream << "Current sources ((row,col - power): ";
	for (auto& it : m_posToSourceMap)
	{
		const TablePos& pos = it.first;
		const SourceInfo& srcInfo = it.second;

		outStream << " (" << pos.row << ", " << pos.col << ") - " << srcInfo.getPower();
	}
	outStream << endl << endl << endl;
}


// Sets the expression on row starting at a position from first character of the expressions
void BoardObject::setExprOnRow(const int row, const int startCol, const std::string& expr)
{
	assert(expr.empty() == false);
	const int startIterPos = startCol - 1;
	int exprStrIter = (int)expr.size() - 2; // NOt cache friendly but doesn't matter with our dimensions

	Cell* prevNodeOnRow = &m_board[row][startCol];
	for (uint i = startIterPos; exprStrIter >= 0; i--, exprStrIter--)
	{
		Cell* thisCell = &m_board[row][i];
		assert(thisCell->isFree() && "THere is a bug ! I'm overriding the same positions here !");
		thisCell->setSymbol(expr[exprStrIter]);

#if RUNMODE == DIRECTIONAL_MODE
		assert(false);
#else
		prevNodeOnRow->m_left = thisCell;
		thisCell->m_parent = prevNodeOnRow;
		prevNodeOnRow = thisCell;
#endif
	}
}

// Sets the expression on columns starting at a position from first character of the expressions
void BoardObject::setExprOnCol(const int col, const int startRow, const std::string& expr)
{
	assert(expr.empty() == false);
	const uint startIterPos = 1;
	Cell* prevNodeOnCol = &m_board[startRow][col];
	for (uint i = 1; i < expr.size(); i++)
	{
		const int targetRow = startRow + i;
		Cell* thisCell = &m_board[targetRow][col];
		assert(thisCell->isFree() && "There is a bug ! I'm overriding the same positions here !");
		thisCell->setSymbol(expr[i]);

#if RUNMODE == DIRECTIONAL_MODE
		assert(false);
#else
		prevNodeOnCol->m_down = thisCell;
		thisCell->m_parent = prevNodeOnCol;
		prevNodeOnCol = thisCell;
#endif
	}
}

// Clear the expression on columns starting at a position from first character of the expressions
void BoardObject::clearExprOnRow(const int row, const int startCol, const uint size)
{
	const int startIterPos = startCol - 1;
	int exprStrIter = (int)size - 2; // NOt cache friendly but doesn't matter with our dimensions

	Cell* prevNodeOnRow = &m_board[row][startCol];
	prevNodeOnRow->m_left = nullptr;
	for (uint i = startIterPos; exprStrIter >= 0; i--, exprStrIter--)
	{
		Cell* thisCell = &m_board[row][i];
		thisCell->reset();
	}
}

// Clear the expression on columns starting at a position from first character of the expressions
void BoardObject::clearExprOnCol(const int col, const int startRow, const uint size)
{
	const uint startIterPos = 1;
	Cell* prevNodeOnCol = &m_board[startRow][col];
	prevNodeOnCol->m_down = nullptr;
	for (uint i = 1; i < size; i++)
	{
		const int targetRow = startRow + i;
		Cell* thisCell = &m_board[targetRow][col];
		thisCell->reset();
	}
}

bool BoardObject::propagateSourceEvent(const Cell::BroadcastEventType srcEventType, const TablePos& pos, const SourceInfo& sourceInfo, const bool allSources)
{
	return internalPropagateSourceEvent(srcEventType, getRootCell(), pos, sourceInfo, allSources, 0);
}

bool BoardObject::internalPropagateSourceEvent(const Cell::BroadcastEventType srcEventType, Cell* cell, const TablePos& tablePos, const SourceInfo& source, const bool allSources, const int depth)
{
	if (cell == nullptr)
		return true;

	bool childResuls = true;

#if RUNMODE == DIRECTIONAL_MODE
	if (cell->isRoot())
	{
		// Did we come back to the root ?
		if (depth > 0)
			return true;

		// Redirect to the cell bellow it
		Cell* cellBelow = &(cell->m_boardView->m_board[cell->m_row + 1][cell->m_column]);
		childResuls &= internalPropagateSourceEvent(srcEventType, cellBelow, tablePos, source, allSources, depth + 1);
	}

	// Follow previous links. Can't get a cycle so no need to protect the flood fill
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const int rowOffset = Cell::DIR_OFFSET[dirIter].row;
		const int colOffset = Cell::DIR_OFFSET[dirIter].col;

		Cell* prev = *(cell->m_previousByDir[dirIter]);
		if (prev)
		{
			childResuls &= internalPropagateSourceEvent(srcEventType, prev, tablePos, source, allSources, depth + 1);
		}
	}
#else
	childResuls &= internalPropagateSourceEvent(srcEventType, cell->m_left, prev, tablePos, source, allSources, depth + 1);
	childResuls &= internalPropagateSourceEvent(srcEventType, cell->m_right, prev, tablePos, source, allSources, depth + 1);
#endif

	bool resThis = false;
	switch (srcEventType)
	{
		case Cell::EVENT_SOURCE_ADD:
			resThis = cell->m_boardView->addSource(tablePos, source);
			break;
		case Cell::EVENT_SOURCE_MODIFY:
			resThis = cell->m_boardView->modifySource(tablePos, source);
			break;
		case Cell::EVENT_SOURCE_REMOVE:
			resThis = cell->m_boardView->removeSource(tablePos, allSources);
			break;
		default:
			assert(false);
	}
	
	return (resThis && childResuls);
}

int BoardObject::getOccupiedItemsOnCol(const int col, const int startRow, const bool down /* = true */, const bool includeMembrane/* = false*/) const
{
	int occupiedItems = 0;
	const int dirOffset = (down ? 1 : -1);

	for (int iRowIter = startRow + dirOffset; ; iRowIter += dirOffset)
	{
		if (!isCoordinateValid(iRowIter, col))
			break;

		// This cell must be free
		if (m_board[iRowIter][col].isFree() || (includeMembrane && m_board[iRowIter][col].m_cellType == CELL_MEMBRANE))
			break;

		occupiedItems++;
	}

	return occupiedItems;
}

//  How many items are on this column below the given start row
int BoardObject::getFreeItemsOnCol(const int col, const int startRow, const bool down) const
{
#if RUNMODE == DIRECTIONAL_MODE
	int freeItems = 0;
	const int dirOffset = (down ? 1 : -1);

	for (int iRowIter = startRow; ; iRowIter += dirOffset)
	{
		if (!isCoordinateValid(iRowIter, col))
			break;

		// This cell must be free
		if (!m_board[iRowIter][col].isFree())
			break;

		// The next cell in direction should be either invalid or free
		if (isCoordinateValid(iRowIter + dirOffset, col) && m_board[iRowIter + dirOffset][col].isFree() == false)
			break;

		// Left/Right cells should be empty
		if (isCoordinateValid(iRowIter, col - 1) && m_board[iRowIter][col - 1].isFree() == false)
			break;

		if (isCoordinateValid(iRowIter, col + 1) && m_board[iRowIter][col + 1].isFree() == false)
			break;

		freeItems++;
	}

	return freeItems;
#else
	//Test if this column is not actually a continuation of another column...
	if (startRow - 1 >= 0 && !m_board[startRow - 1][col].isFree())
		return -1;

	int freeItems = 0;
	for (int iterRow = startRow + 1; iterRow < MAX_ROWS; iterRow++)
	{
		if (m_board[iterRow][col].isFree() == false)
			break;

		freeItems++;
	}

	return freeItems;
#endif
}

int BoardObject::getOccupiedItemsOnRow(const int row, const int startCol, const bool left /* = true */, const bool includeMembrane /* = false */) const
{
	int occupiedItems = 0;
	const int dirOffset = (left ? -1 : 1);

	for (int iColIter = startCol + dirOffset; ; iColIter += dirOffset)
	{
		if (!isCoordinateValid(row, iColIter))
			break;

		// This cell must be free
		if (m_board[row][iColIter].isFree() || (includeMembrane && m_board[row][iColIter].m_cellType == CELL_MEMBRANE))
			break;

		occupiedItems++;
	}

	return occupiedItems;
}

//  How many items are on this row in the right side from the given start column
int BoardObject::getFreeItemsOnRow(const int row, const int startCol, const bool left) const
{
#if RUNMODE == DIRECTIONAL_MODE
	int freeItems = 0;
	const int dirOffset = (left ? -1 : 1);

	for (int iColIter = startCol; ; iColIter += dirOffset)
	{
		if (!isCoordinateValid(row, iColIter))
			break;

		// This cell must be free
		if (!m_board[row][iColIter].isFree())
			break;

		// The next cell in direction should be either invalid or free
		if (isCoordinateValid(row, iColIter + dirOffset) && m_board[row][iColIter + dirOffset].isFree() == false)
			break;

		// Left/Right cells should be empty
		if (isCoordinateValid(row - 1, iColIter) && m_board[row - 1][iColIter].isFree() == false)
			break;

		if (isCoordinateValid(row + 1, iColIter) && m_board[row + 1][iColIter].isFree() == false)
			break;

		freeItems++;
	}

	return freeItems;
#else
	//Test if this row is not actually a continuation of another row...
	if (startCol + 1 < MAX_COLS && m_board[row][startCol + 1].isFree() == false)
		return -1;

	int freeItems = 0;
	for (int iterCol = startCol - 1; iterCol >= 0; iterCol--)
	{
		if (m_board[row][iterCol].isFree() == false)
			break;

		freeItems++;
	}

	return freeItems;
#endif
}

// Try several attempts to generate a column at pivotRow with trying of different columns between startCol and endCol
void BoardObject::generateCol(const int pivotROW, const int startCol, const int endCol, const int depth)
{
	if (depth < 0)
		return;

	const int lenCol = endCol - startCol + 1;
	for (int attempt = 1; attempt <= lenCol; attempt++)
	{
		// Select randomly a column and check the free space on this column below
		const int selectedCol = randRange(startCol, endCol);
		const int limit = getFreeItemsOnCol(selectedCol, pivotROW);

		if (limit >= m_colGenerator->getMinSize())
		{
			ExprMatchResult exprRes;
			int localConsumed = 0;
			Constraint constraint;
			assert(m_board[pivotROW][selectedCol].isFree() == false);
			constraint.setConstraintFirst(m_board[pivotROW][selectedCol].m_symbol);

			const bool succeed = m_colGenerator->GenerateRandom(limit + 1, exprRes, &constraint);
			if (succeed)
			{
				const int rowStart = pivotROW;
				const int rowEnd = pivotROW + (int)exprRes.m_str.size() - 1;
				setExprOnCol(selectedCol, rowStart, exprRes.m_str);

				// If the board is fine, go next
				if (isCompliantWithRowColPatterns())
				{
					generateRow(selectedCol, rowStart, rowEnd - 1, depth - 1);
					break;
				}
				else
				{
					clearExprOnCol(selectedCol, rowStart, (uint)exprRes.m_str.size());
				}
			}
		}
	}
}

void BoardObject::generateRow(const int pivotCol, const int rowStart, const int rowEnd, const int depth)
{
	if (depth < 0)
		return;

	const int lenRow = rowEnd - rowStart + 1;
	for (int attempt = 1; attempt <= lenRow; attempt++)
	{
		const int selectedRow = randRange(rowStart, rowEnd);
		const int limit = getFreeItemsOnRow(selectedRow, pivotCol);

		if (limit >= m_rowGenerator->getMinSize())
		{
			ExprMatchResult exprRes;
			int localConsumed = 0;
			Constraint constraint;
			assert(m_board[selectedRow][pivotCol].isFree() == false);
			constraint.setConstraintLast(m_board[selectedRow][pivotCol].m_symbol);

			const bool succeed = m_rowGenerator->GenerateRandom(limit + 1, exprRes, &constraint);
			if (succeed)
			{
				const int colStart = pivotCol - (int)exprRes.m_str.size() + 1;
				const int colEnd = pivotCol;
				setExprOnRow(selectedRow, colEnd, exprRes.m_str);

				if (isCompliantWithRowColPatterns())
				{
					generateCol(selectedRow, colStart, colEnd, depth - 1);
					break;
				}
				else
				{
					clearExprOnRow(selectedRow, colEnd, (uint)exprRes.m_str.size());
				}
			}
		}
	}
}

#if RUNMODE == DIRECTIONAL_MODE

// Try a few random variants to produce according to iterator at nextPointer,
// Checks inside if produced thing is fine and if the neighb rule is satisfied
void BoardObject::ProduceItem(TablePos& nextPointer, Expression_Node::Iter& exprIter, const int maxAttemptsForTree,
	ProduceItemResult& outRes, int& outNumItemsProduced)
{
	// Knowing the format we trick it a little.... generate N characters from the first element known as STAR, then one 
	// from the middle element which is a single one
	// Impose a strict character by knowing the format !
	// E.g. if -> then produce 7, if  <-  then 2 , if NORTH then 4, if SOUTH then e
	// Always produce at least 5 items of each STAR to satisfy the neighb rule
	// 
	const Expression_Node* nextNode = exprIter.getNext();
	
	ExprMatchResult outResult;
	Expression_Generator::internalGenerateRandom(*nextNode, 1/*limit*/, outResult, nullptr);
}

Expression_Node::Iter BoardObject::getExprIter(const DIRECTION dir,
	const Expression_Generator* m_rowGenerator,
	Expression_Generator* m_colGenerator) const
{
	switch (dir)
	{
	case DIR_DOWN:
		return m_colGenerator->getIter();
	case DIR_UP:
		return m_colGenerator->getIter(true);
	case DIR_RIGHT:
		return m_rowGenerator->getIter();
	case DIR_LEFT:
		return m_rowGenerator->getIter(true);
	default:
		assert(false);
		return Expression_Node::Iter();
	}
}

void BoardObject::generateMembrane(const int minMembraneSize, const int maxMembraneSize)
{
	TablePos nextPointer(m_rootRow, m_rootCol);

#if NOTCHEATING
	//Cell& rootCell = m_board[rootRow][rootColumn];
	//rootCell.setSymbol(resultRow.m_str.back());
	//rootCell.m_column = rootColumn;
	//rootCell.m_row = rootRow;

	DIRECTION currDir = DIR_RIGHT;
	TablePos nextPointer(rootRow, rootColumn);
	//bool ROOT_CELL_PRODUCED = false;

	// This will be a stack of values along each new direction being tried to compute the main membrane
	struct NewDirectionValues
	{
		TablePos nextPointer;	// The position where the generation started
		DIRECTION dir;			// The direction where the generation was done
		int numItemsProduced;   // The number of items generated

		NewDirectionValues(DIRECTION _prevDir, TablePos& _nextPointer, int _numItemsProduced)
		{
			dir = _prevDir;
			nextPointer = _nextPointer;
			numItemsProduced = _numItemsProduced;
		}
	};

	std::stack<NewDirectionValues> movementsStack;
	bool IS_MEMBRANE_GENERATION_SUCCEEDED = false;

	while (true) // Not finished the process yet
	{
		// Get an iterator for the expression needed according to the current direction
		Expression_Node::Iter currExprIter = getExprIter(currDir, m_rowGenerator, m_colGenerator);

		ProduceItemResult res = P_RES_FAILED;
		int numItemsProduced = 0;

		// Try a few random variants to produce according to refToken
		// Checks inside if produced thing is fine and if the neighb rule is satisfied
		ProduceItem(nextPointer, currExprIter, numMaxAtttempts, res, numItemsProduced);

		if (res == ProduceItemResult::P_RES_FINISHED) // Meet root again
		{
			IS_MEMBRANE_GENERATION_SUCCEEDED = true;
		}
		else if (res == ProduceItemResult::P_RES_SUCCEED)
		{
			DIRECTION prevDir = currDir;
			currDir = getDirection(lastPosGenerated)
				movementsStack.push(NewDirectionValues(prevDir, nextPointer, numItemsProduced);
			nextPointer = lastPosGenerate + DirectionOffset[CURRDIR]
		}
		else
		{
			if (movementsStack.empty())
			{
				IS_MEMBRANE_GENERATION_SUCCEEDED = false;
				break; // FAIL TO GENERATE MEMBRANE
			}
			else
			{
				NewDirectionValues topValue = movementsStack.top();
				currDir = topValue.dir;
				nextPointer = topValue.nextPointer;
				movementsStack.pop();
			}
		}
	}
#else
	// Produce n * 4 +  m * 7 + (n+1)*e + (m-1)*2
	int n = randRange(minMembraneSize, maxMembraneSize);
	int m = randRange(minMembraneSize, maxMembraneSize);

	n = std::min(n, MAX_COLS - nextPointer.col - 1);
	m = std::min(m, MAX_ROWS - nextPointer.row - 1);

	assert(n >= minMembraneSize && "Can't fit the minimum membrane size");
	assert(m >= minMembraneSize && "Can't fit the minimum membrane size");

	// Produce the 4's
	Cell* prevCell = nullptr;
	for (int i = 0; i < n; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('4');
		thisCell->m_cellType = CELL_MEMBRANE;
		nextPointer.col++;
	}

	// Produce the 7's
	for (int i = 0; i < m; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('7');
		thisCell->m_cellType = CELL_MEMBRANE;
		nextPointer.row++;
	}

	// Produce the e's
	for (int i = 0; i < n + 1; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('e');
		thisCell->m_cellType = CELL_MEMBRANE;
		nextPointer.col--;
	}

	// Produce the 2's
	nextPointer.col++;
	nextPointer.row--;
	for (int i = 0; i < m - 1; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('2');
		thisCell->m_cellType = CELL_MEMBRANE;
		nextPointer.row--;
	}
#endif
}

void BoardObject::getBorderPoints(const int rootRow, const int rootCol, TablePos& outMin, TablePos& outMax)
{
	outMin = TablePos(rootRow, rootCol);
	int currRow = rootRow;
	int currCol = rootCol;
	while (currCol + 1 < MAX_COLS && m_board[currRow][currCol + 1].isFree() == false) // Go to max right
	{
		currCol++;
	}

	// Go to max down
	while (currRow + 1 < MAX_ROWS && m_board[currRow + 1][currCol].isFree() == false)
	{
		currRow++;
	}

	outMax = TablePos(currRow, currCol);
}

// Generates 7* up and  2* down (middle is considered as 4 OR e).
void BoardObject::generateColExpression(const int middleRow, const int maxAttempts, const int depth, const int colMin, const int colMax)
{
	if (depth <= 0)
		return;

	std::vector<int> indices;
	indices.reserve(colMax - colMin + 1);
	for (int i = colMin; i <= colMax; i++)
		indices.push_back(i);
	std::random_shuffle(indices.begin(), indices.end());

	const int attempts = std::min(maxAttempts, (int)indices.size());
	for (int attemptIter = 0; attemptIter < attempts; attemptIter++)
	{
		const int col = indices[attemptIter];

		Cell* startCell = &m_board[middleRow][col];
		assert(startCell->m_symbol == '4' || startCell->m_symbol == 'e');

		const int freeItemsUp = getFreeItemsOnCol(col, middleRow, false);
		const int freeItemsDown = getFreeItemsOnCol(col, middleRow, true);

		// Randomize the numbers of items to put down and up
		const int numItemsUp = randRange(0, freeItemsUp);
		const int numItemsDown = randRange(0, freeItemsDown);

		// Generate Up (7 symbol)
		{
			Cell* cellIter = startCell;
			for (int upIter = 1; upIter <= numItemsUp; upIter++)
			{
				Cell* newCell = &m_board[middleRow - upIter][col];
				newCell->setSymbol('7');
				newCell->m_down = cellIter;
				cellIter->m_prevUp = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_UP);
			}
		}

		// Generate down (2 symbol)
		{
			Cell* cellIter = startCell;
			for (int downIter = 1; downIter <= numItemsDown; downIter++)
			{
				Cell* newCell = &m_board[middleRow + downIter][col];
				newCell->setSymbol('2');
				newCell->m_up = cellIter;
				cellIter->m_prevDown = newCell;
				cellIter = newCell;

				newCell->m_cellType = decideCellType(startCell, DIR_DOWN);
			}
		}

		generateRowExpression(col, maxAttempts, depth - 1, middleRow - numItemsUp, middleRow - 1);
		generateRowExpression(col, maxAttempts, depth - 1, middleRow + 1, middleRow + numItemsDown);
	}
}


// Generates 4* left and e* right (middle is considered as 2 OR 7).
void BoardObject::generateRowExpression(const int middleCol, const int maxAttempts, int depth, const int rowMin, const int rowMax)
{
	if (depth <= 0)
		return;

	std::vector<int> indices;
	indices.reserve(rowMax - rowMin + 1);
	for (int i = rowMin; i <= rowMax; i++)
		indices.push_back(i);
	std::random_shuffle(indices.begin(), indices.end());

	const int attempts = std::min(maxAttempts, (int)indices.size());
	for (int attemptIter = 0; attemptIter < attempts; attemptIter++)
	{
		const int row = indices[attemptIter];

		Cell* startCell = &m_board[row][middleCol];
		assert(startCell->m_symbol == '2' || startCell->m_symbol == '7');

		const int freeItemsLeft = getFreeItemsOnRow(row, middleCol, true);
		const int freeItemsRight = getFreeItemsOnCol(row, middleCol, false);

		// Randomize the numbers of items to put down and up
		const int numItemsLeft = randRange(0, freeItemsLeft);
		const int numItemsRight = randRange(0, freeItemsRight);

		// Generate Left (4 symbol)
		{
			Cell* cellIter = startCell;		

			for (int leftIter = 1; leftIter <= numItemsLeft; leftIter++)
			{
				Cell* newCell = &m_board[row][middleCol - leftIter];
				newCell->setSymbol('4');
				newCell->m_cellType = decideCellType(startCell, DIR_LEFT);
			}
		}

		// Generate right (e symbol)
		{
			Cell* cellIter = startCell;
			for (int rightIter = 1; rightIter <= numItemsRight; rightIter++)
			{
				Cell* newCell = &m_board[row][middleCol + rightIter];
				newCell->setSymbol('e');
				newCell->m_cellType = decideCellType(startCell, DIR_RIGHT);
			}
		}

		// Recursively generate columns over the newly left and right sequences
		generateColExpression(row, maxAttempts, depth - 1, middleCol - numItemsLeft, middleCol - 1);
		generateColExpression(row, maxAttempts, depth + 1, middleCol + 1, middleCol + numItemsRight);
	}
}

void BoardObject::generateDirectionalBoardModel(
					Expression_Generator* m_rowGenerator,
					Expression_Generator* m_colGenerator, 
					const int numMaxAtttempts, 
					const int depth, const int minMembraneSize, const int maxMembraneSize)
{
	generateMembrane(minMembraneSize, maxMembraneSize);

	// Generate new childrens on NORTH, SOUTH, EAST and WEST sides.
	// Get first the rectangles
	TablePos minPos, maxPos;
	getBorderPoints(m_rootRow, m_rootCol, minPos, maxPos);

	// Generates 7* up and  2* down (middle is considered as 4 OR e).
	generateColExpression(minPos.row, numMaxAtttempts, depth, minPos.col, maxPos.col);
	generateColExpression(maxPos.row, numMaxAtttempts, depth, minPos.col, maxPos.col);

	// Generates 4* left and e* right (middle is considered as 2 OR 7).
	generateRowExpression(minPos.col, numMaxAtttempts, depth, minPos.row, maxPos.row);
	generateRowExpression(maxPos.col, numMaxAtttempts, depth, minPos.row, maxPos.row);

	// Updates the links
	updateInternalCellsInfo();
}
#endif

bool BoardObject::generateRandomBoard(const int numDepthBranches, const int numSources)
{
	assert(numDepthBranches >= 0);
	assert(numSources >= 0);
	assert(m_colGenerator && m_rowGenerator);

	const int maxAttemptsForTree = 100;
	for (int treeAttempt = 0 ; treeAttempt < maxAttemptsForTree; treeAttempt++)
	{
		reset(true);

#if RUNMODE == LEFTRIGHTONLY_MODE
		const int maxAttemptsForRoot = 10; // Just to be sure that we don;t block forever in tehe case of a wrong input
										   // INstantiate two expression generators 

		ExprMatchResult resultRow, resultCol;

		// Step 1: Generate root 
		//-------------------------------------------------
		bool succeded = false;
		for (int i = 0; i < maxAttemptsForRoot; i++)
		{
			const bool succededRow = m_rowGenerator->GenerateRandom(MAX_COLS, resultRow, nullptr);
			if (!succededRow)
				continue;

			Constraint constr;
			constr.setConstraintFirst(resultRow.m_str.back());
			const bool succededCol = m_colGenerator->GenerateRandom(MAX_ROWS, resultCol, &constr);
			if (!succededCol)
				continue;

			succeded = true;
			break;
		}

		if (succeded == false)
		{
			assert(false && "couldn't create ROOT from the given expressions!");
			return false;
		}

		// Set the root first on board
		const int rootColumn = MAX_COLS - 1;
		const int rootRow = 0;

		const int startColumn = MAX_COLS - (int)resultRow.m_str.size(), endColumn = MAX_COLS - 1;
		m_board[0][MAX_COLS - 1].setSymbol(resultRow.m_str.back());
		m_board[0][MAX_COLS - 1].m_row = 0;
		m_board[0][MAX_COLS - 1].m_column = MAX_COLS - 1;
		setExprOnRow(0, endColumn, resultRow.m_str);
		setExprOnCol(MAX_COLS - 1, 0, resultCol.m_str);
		//--------------------------------------

		// Step 2: generate the cols and rows recursively from maxDepth
		generateCol(0, startColumn, endColumn, numDepthBranches - 1);
		generateRow(MAX_COLS - 1, 0, (int)resultCol.m_str.size() - 1, numDepthBranches - 1);

		setRootLocation(rootRow, rootColumn);
#else
		generateDirectionalBoardModel(m_rowGenerator, m_colGenerator, maxAttemptsForTree, numDepthBranches, MIN_MEMBRANE_SIZE, MAX_MEMBRANE_SIZE);
#endif

		const int numNodes = countNodes();
		if (numNodes < g_minNodesOnRandomTree)
		{
			continue;
		}

		// Step 3: simulate initial messages
		// a) discover structure message - root (fixed position) sends a message to help every cell find its position. When the information come back to root we do step b)
		// b) broadcast entire structure to everyone
		// After these two steps everybody knows the distance to root and the number of branches below
		Cell* root = getRootCell();
		root->onMsgDiscoverStructure(root->m_row, root->m_column, 0);
		
		root->onRootMsgBroadcastStructure(this); // Root uses the global board !!!

												 // Step 2.5: generate some random sources
		const int numSourcesToGenerate = numSources; //randRange(1, 4);
		for (int i = 0; i < numSourcesToGenerate; i++)
		{
			const int row = randRange(0, MAX_ROWS - 1);
			const int col = randRange(0, MAX_COLS - 1);
			const float power = (float)randRange(g_minPowerForWirelessSource, g_maxPowerForWirelessSource);

			SourceInfo src;
			src.overridePower(power);
			propagateSourceEvent(Cell::EVENT_SOURCE_ADD, TablePos(row, col), src, false);
		}

		// We found a solution !
		getRootCell()->initFlowStatistics(g_simulationTicksForDataFlowEstimation);
		return true;
	}

	return false;
}

bool BoardObject::optimizeMembrane_byCutRowCols()
{
	int outRowToCut = INVALID_POS, outColToCut = INVALID_POS;
	float flowBenefit = INVALID_FLOW;
	DIRECTION outDirToCut = DIR_COUNT;
	const bool res = evaluateMembraneOptimization(outRowToCut, outColToCut, outDirToCut, flowBenefit);
	
	if (!res || flowBenefit == INVALID_FLOW)
		return false;
	

	assert(((outRowToCut != INVALID_POS) + (outColToCut != INVALID_POS)) == 1);
	
	if (g_verboseBestGatheredSolutions)
	{
		(*g_debugLogOutput) << "### Best sol found for membrane CUT: " << " row: " << outRowToCut << " col: " << outColToCut << " dir: " << Cell::getDirString(outDirToCut) << " " << std::endl << " flow benefit " << flowBenefit << std::endl;
	}
	
	// Perform definitive cut over this board
	if (outRowToCut != INVALID_POS)
	{
		cutMembraneByRow(outRowToCut, outDirToCut, true);
	}
	
	if (outColToCut != INVALID_POS)
	{
		cutMembraneByCol(outColToCut, outDirToCut, true);
	}
	
	// With the new resources garbage collected try to add internal and external trees
	expandInternalTrees();
	expandExternalTrees();
		
	return true;
}

bool BoardObject::optimizeMembrane_byCutCorners()
{
	std::vector<TablePos> inflexionPoints;
	getInflexionPointAndConnectMembrane(inflexionPoints, true);

	CutCornerDescription res;
	float flowBenefit = INVALID_FLOW;
	if (!evaluateBestCornerCutChance(inflexionPoints, res, flowBenefit))
		return false;

	cutMembraneCorner(inflexionPoints, res, true);

	// With the new resources garbage collected try to add internal and external trees
	expandInternalTrees();
	expandExternalTrees();

	return true;
}

void BoardObject::optimizeMembrane()
{
	// TODO HACK REMOVE
	g_debugLogOutput = &std::cout;

	// Cut nodes from membrane and add them to external nodes 
	if (g_verboseBestGatheredSolutions)
	{
		(*g_debugLogOutput) << "==== Starting to evaluate membrane optimizations ====" << std::endl;
	}

	do {
		// Method A: try to remove cells from the membrane and add them to internal/external trees
		const bool methodA_hasResult = optimizeMembrane_byCutRowCols();
		const bool methodB_hasResult = optimizeMembrane_byCutCorners();

		if (methodA_hasResult == false && methodB_hasResult == false)
			break;

	} while (true);
}

void BoardObject::cutMembraneByRow(const int index, const DIRECTION dir, const bool definitive /* = false */)
{
	assert(dir == DIR_UP || dir == DIR_DOWN);

	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);
	updateMembraneBounds(membraneCells);

	const int colMin = m_membraneBoundsCols.first;
	const int colMax = m_membraneBoundsCols.second;
	const int rowMin = m_membraneBoundsRows.first;
	const int rowMax = m_membraneBoundsRows.second;

	// Add the membrane's row to the garbage list
	for (int colIter = colMin; colIter <= colMax; colIter++)
	{
		Cell& cell = m_board[index][colIter];
		if (cell.m_cellType != CELL_MEMBRANE)
			continue;

		m_garbageCollectedResources[cell.m_symbol]++;
	}

	// Reset the cells
	resetCells(false);

	int newRootRow = m_rootRow;
	int newRootCol = m_rootCol;

	// Perform the actual shift
	if (dir == DIR_UP)
	{
		// Items come from up side
		for (int iRow = index; iRow > rowMin; iRow--)
		{
			for (int iCol = 0; iCol < MAX_COLS; iCol++)
			{
				m_board[iRow][iCol].setSymbol(m_board[iRow - 1][iCol].m_symbol);
				m_board[iRow - 1][iCol].setEmpty();
			}
		}

		if (m_rootRow < index)
			newRootRow++;
	}
	else if (dir == DIR_DOWN)
	{
		// Items come from down side
		for (int iRow = index; iRow < rowMax; iRow++)
		{
			for (int iCol = 0; iCol < MAX_COLS; iCol++)
			{
				//std::cout << "last " << iRow << " " << iCol << std::endl;

				m_board[iRow][iCol].setSymbol(m_board[iRow + 1][iCol].m_symbol);
				m_board[iRow + 1][iCol].setEmpty();
			}
		}

		if (m_rootRow > index)
			newRootRow--;
	}

	updateRootLocation(newRootRow, newRootCol);

	if (definitive)
	{
		updateBoardAfterSymbolsInit();
	}
}

void BoardObject::cutMembraneByCol(const int index, const DIRECTION dir, const bool definitive /* = false */)
{
	assert(dir == DIR_LEFT || dir == DIR_RIGHT);

	std::vector<Cell*> membraneCells;
	getMembraneCells(membraneCells);
	updateMembraneBounds(membraneCells);

	const int colMin = m_membraneBoundsCols.first;
	const int colMax = m_membraneBoundsCols.second;
	const int rowMin = m_membraneBoundsRows.first;
	const int rowMax = m_membraneBoundsRows.second;

	// Add the membrane's column to the garbage list
	for (int rowIter = rowMin; rowIter <= rowMax; rowIter++)
	{
		Cell& cell = m_board[rowIter][index];
		if (cell.m_cellType != CELL_MEMBRANE)
			continue;

		m_garbageCollectedResources[cell.m_symbol]++;
	}

	// Reset the cells
	resetCells(false);

	int newRootRow = m_rootRow;
	int newRootCol = m_rootCol;

	// Perform the actual shift
	if (dir == DIR_LEFT)
	{
		// Items come from left side
		for (int iCol = index; iCol > colMin; iCol--)
		{
			for (int iRow = 0; iRow < MAX_ROWS; iRow++)
			{
				Cell& prevCell = m_board[iRow][iCol - 1];
				Cell& newCell = m_board[iRow][iCol];
				newCell.setSymbol(prevCell.m_symbol);
				prevCell.setEmpty();
			}
		}

		if (m_rootCol < index)
			newRootCol++;
	}
	else if (dir == DIR_RIGHT)
	{
		// Items come from right side
		for (int iCol = index; iCol < colMax; iCol++)
		{
			for (int iRow = 0; iRow < MAX_ROWS; iRow++)
			{
				m_board[iRow][iCol].setSymbol(m_board[iRow][iCol + 1].m_symbol);
				m_board[iRow][iCol + 1].setEmpty();
			}
		}

		if (m_rootCol > index)
			newRootCol--;
	}

	updateRootLocation(newRootRow, newRootCol);
	
	if (definitive)
	{
		updateBoardAfterSymbolsInit();
	}
}

const bool IsGoing_NorthWest(const TablePos& startPos, const TablePos& middlePos, const TablePos& endPos)
{
	// Going north ?
	if (startPos.col != middlePos.col
		|| startPos.row <= middlePos.row)
		return false;

	// Going west ?
	if (middlePos.col >= endPos.col 
		|| middlePos.row != endPos.row)
		return false;

	return true;
}

bool BoardObject::cutMembraneCorner(const std::vector<TablePos>& inflexionPoints, CutCornerDescription& membraneCornerDesc, const bool definitive /* = false */)
{
	const TablePos& startP = inflexionPoints[membraneCornerDesc.indexS];
	const TablePos& middleP = inflexionPoints[membraneCornerDesc.indexM];
	const TablePos& endP = inflexionPoints[membraneCornerDesc.indexE];

	const int yOffset = membraneCornerDesc.yOffset;
	const int xOffset = membraneCornerDesc.xOffset;

	// Remove all cells and save newRoot pos if needed
	int newRootRow = m_rootRow;
	int newRootCol = m_rootCol;

	const TablePos upDir = get2DNormDir(startP, middleP);
	const TablePos westDir = get2DNormDir(middleP, endP);
	TablePos dirToUse = upDir;
	for (TablePos iterPos = startP; ; iterPos += dirToUse)
	{
		if (iterPos == endP)
		{
			break;
		}

		// Collect external / internal trees attached to this 
		// ------------------------------------
		{
			Cell& currentCell = m_board[iterPos.row][iterPos.col];
			Cell* childrenList[DIR_COUNT];
			currentCell.fillChildrenList(childrenList);

			for (int childIter = 0; childIter < DIR_COUNT; childIter++)
			{
				Cell* child = childrenList[childIter];
				if (child == nullptr || child->m_cellType == CELL_MEMBRANE)
					continue;

				garbageCollectSubtree(child);
			}
		}

		// Check root deletion and set empty this cell
		{
			Cell& thisCell = m_board[iterPos.row][iterPos.col];
			if (thisCell.isRoot())
			{
				newRootRow = newRootCol = INVALID_POS;
			}

			thisCell.setEmpty();
		}

		// Check end / change dir
		//-------------------------
		if (iterPos == middleP)
		{
			dirToUse = westDir;
		}
	}

	resetCells(false);

	// Set new root coordinates if it was deleted
	if (newRootRow == INVALID_POS || newRootCol == INVALID_POS)
	{
		newRootRow = middleP.row;
		newRootCol = xOffset;
	}

	for (int y = startP.row; y >= yOffset + 1; y--)
		m_board[y][startP.col].setSymbol('2');

	for (int x = startP.col; x <= xOffset - 1; x++)
		m_board[yOffset][x].setSymbol('4');

	for (int y = yOffset; y >= middleP.row + 1; y--)
		m_board[y][xOffset].setSymbol('2');

	for (int x = xOffset; x <= endP.col - 1; x++)
		m_board[middleP.row][x].setSymbol('4');

	if (isCompliantWithRowColPatterns() == false)
	{
		if (false)
		{
			printBoard(std::cout);
		}

		return false;
	}

	// Update new root's location and links on board
	updateRootLocation(newRootRow, newRootCol);
	updateBoardAfterSymbolsInit();

	return true;
}

bool BoardObject::evaluateBestCornerCutChance(const std::vector<TablePos>& inflexionPoints, CutCornerDescription& outBestCornerCutRes, float& outBestFlowBenefit)
{
	outBestCornerCutRes.reset();
	outBestFlowBenefit = INVALID_FLOW;

	const float baseFlowValue = this->doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1);

	// Take 3 consecutive points and try to cut the corner
	for (int infPointIter = 0; infPointIter < inflexionPoints.size() - 1; infPointIter++)
	{
		const int startIdx = infPointIter;
		const int middleIdx = infPointIter + 1;
		int endIdx = infPointIter + 2;
		if (endIdx == inflexionPoints.size())
			endIdx = 1;

		const TablePos& startP = inflexionPoints[startIdx];
		const TablePos& middleP = inflexionPoints[middleIdx];
		const TablePos& endP = inflexionPoints[endIdx];

		// Check side
		if (IsGoing_NorthWest(startP, middleP, endP))
		{			
			// Find the best corner desc here. Try all possible deviations
			// Then do the cut and compare 
			CutCornerDescription localBestCornerDesc;
			const int cutStartRow = startP.row - 2;
			const int cutEndRow = middleP.row + 1;
			const int colStartDeviation = startP.col + 1;
			const int colEndDeviation = endP.col - 1;

			for (int rowIter = cutStartRow; rowIter >= cutEndRow; rowIter--)
			{
				for (int colIter = colStartDeviation; colIter <= colEndDeviation; colIter++)
				{
					BoardObject* newBoard = new BoardObject();
					*newBoard = *this;

					//--- trees are collected now
					// ####################################################
					localBestCornerDesc.reset();
					localBestCornerDesc.indexS = startIdx;
					localBestCornerDesc.indexM = middleIdx;
					localBestCornerDesc.indexE = endIdx;
					localBestCornerDesc.yOffset = rowIter;
					localBestCornerDesc.xOffset = colIter;
					localBestCornerDesc.dirType = CutCornerDescription::NORTH_WEST;

					const bool isBoardOK = newBoard->cutMembraneCorner(inflexionPoints, localBestCornerDesc, false);
					if (isBoardOK)
					{
						// Expand internal and external trees
						newBoard->expandInternalTrees();
						newBoard->expandExternalTrees();

						const float flowRes = newBoard->doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(1);
						const float localFlowBenefit = flowRes - baseFlowValue;
						if (localFlowBenefit > outBestFlowBenefit)
						{
							outBestFlowBenefit = localFlowBenefit;
							outBestCornerCutRes = localBestCornerDesc;
						}
					}

					delete newBoard;
				}
			}			
		}

		// TODO: Add the code for others
	}

	// If we have any flow benefit, check the result in outBestCornerCutRes
	return (outBestFlowBenefit > 0.0f);
}

void BoardObject::reorganize(ostream& outStream)
{
	g_debugLogOutput = &outStream;

#if RUNMODE == DIRECTIONAL_MODE
	runGarbageCollector(g_energyLossThreshold, outStream);

	expandInternalTrees();

	expandExternalTrees();

	optimizeMembrane();
#else
	Cell* rootCell = getRootCell();
	assert(rootCell);
	rootCell->onRootMsgReorganize();
#endif
}


int BoardObject::countNodes() const
{
	int count = 0;
	for (int i = 0; i < MAX_ROWS; i++)
		for (int j = 0; j < MAX_COLS; j++)
		{
			if (!m_board[i][j].isFree())
				count++;
		}

	return count;
}

void BoardObject::updateRootLocation(const int newRootRow, const int newRootCol)
{
	if (newRootRow == m_rootRow && newRootCol == m_rootCol)
		return;

	transferFlowStatistics(m_rootRow, m_rootCol, newRootRow, newRootCol);

	m_rootRow = newRootRow;
	m_rootCol = newRootCol;
}

void BoardObject::transferFlowStatistics(const int prevRootRow, const int prevRootCol, const int newRootRow, const int newRootCol)
{
	Cell& prevRoot = m_board[prevRootRow][prevRootCol];
	Cell& newRoot = m_board[newRootRow][newRootCol];

	assert(prevRoot.m_flowStatistics != nullptr && newRoot.m_flowStatistics == nullptr);

	newRoot.m_flowStatistics = prevRoot.m_flowStatistics;
	prevRoot.m_flowStatistics = nullptr;
}

void BoardObject::reorganizeMaxFlow(int* outNumReorganizationsMade)
{
	int numIterations = 0;
	float prevFlow = 0.0f;
	while (true)
	{
		reorganize(*g_debugLogOutput);
		doDataFlowSimulation_serial(1);
		const float newFlow = getLastSimulationAvgDataFlowPerUnit();
		if (newFlow <= prevFlow)
			break;

		prevFlow = newFlow;
		numIterations++;
	}

	if (outNumReorganizationsMade)
		*outNumReorganizationsMade = numIterations;
}

void BoardObject::gatherLeafNodes(const Cell* currentCell, std::vector<TablePos>& outLeafNodes) const
{
	if (currentCell == nullptr)
		return;

	if (currentCell->isLeaf())
	{
		outLeafNodes.push_back(TablePos(currentCell->m_row, currentCell->m_column));
	}
	else
	{
		Cell* childrenList[DIR_COUNT];
		currentCell->fillChildrenList(childrenList);
		
		for (int childIter = 0; childIter < DIR_COUNT; childIter++)
		{
			Cell* cell = childrenList[childIter];
			if (cell)
			{
				gatherLeafNodes(cell, outLeafNodes);
			}
		}
	}
}

void BoardObject::fillSimulationContext(SimulationContext& simContext) const
{
	// Step1 : Gather all the leaf nodes and fill an augmented data structure
	std::vector<TablePos> leafNodes;

	const Cell* root = getRootCell();
#if RUNMODE == DIRECTIONAL_MODE
	// In directional mode start below root and follow the links
	const Cell* cellBelowRoot = &root->m_boardView->m_board[root->m_row + 1][root->m_column];
	gatherLeafNodes(cellBelowRoot, leafNodes);
#else
	gatherLeafNodes(root, leafNodes);
#endif

	struct CellTempCaptureInfo
	{
		const Cell* cell = nullptr;
		TablePos pos;
		float remainingCap = 0.0f;

		float currentIterCapSum = 0.0f;

		void onAddCapture(const float value)
		{
			remainingCap -= value;
			assert(remainingCap >= 0.0f);

			currentIterCapSum += value;
		}
	};

	using LeafNodsCaptureInfoArray = std::vector<CellTempCaptureInfo>;
	LeafNodsCaptureInfoArray leafNodesCapture;
	leafNodesCapture.reserve(leafNodes.size());
	for (int i = 0; i < leafNodes.size(); i++)
	{
		const TablePos& pos = leafNodes[i];

		CellTempCaptureInfo tempInfo;
		tempInfo.cell = &m_board[pos.row][pos.col];
		tempInfo.pos = pos;
		tempInfo.remainingCap = tempInfo.cell->getRemainingCap();
		tempInfo.currentIterCapSum = 0.0f;

		leafNodesCapture.push_back(tempInfo);
	}

	//------------------------

	// Step 2: Shuffle the sources and leaf nodes list to have variation from time to time
	std::vector<std::pair<TablePos, SourceInfo>> shuffledSources(m_posToSourceMap.begin(), m_posToSourceMap.end());
	std::random_shuffle(shuffledSources.begin(), shuffledSources.end());

	std::vector<int> leafNodesCaptureIndirection(leafNodesCapture.size()); // Indices: when iterating over element i becomes = >leafNodesCaptureIndirection[i]
	for (int i = 0; i < leafNodesCapture.size(); i++) leafNodesCaptureIndirection[i] = i;

	// Define the lambda function used to sort and shuffle. It will be used at each iteration through the sources
	auto SortAndShuffleLeafNodesFunc = [&](const TablePos& srcPos){
		// Sort by distance to source using a functor
		struct LeafToSrcFunctor
		{
			LeafToSrcFunctor(const TablePos& srcPos, LeafNodsCaptureInfoArray& refInfoArray) 
				:	m_srcPos(srcPos), 
					m_refInfoArray(refInfoArray) 
				{}

			bool operator()(int a, int b) const
			{ 
				return manhattanDist(m_srcPos, m_refInfoArray[a].pos) < manhattanDist(m_srcPos, m_refInfoArray[b].pos); 
			}

		private:
			TablePos m_srcPos;
			LeafNodsCaptureInfoArray& m_refInfoArray;
		};

		std::sort(leafNodesCaptureIndirection.begin(), leafNodesCaptureIndirection.end(), LeafToSrcFunctor(srcPos, leafNodesCapture));

		// Then shuffle		
		//std::random_shuffle(leafNodesCapture.begin(), leafNodesCapture.end(), [] (int index){return index - 1; });
		// BUG IN STL AT LEAST IN VS version !! Doing the line above manually
		const uint numLeafNodes = (uint)leafNodesCapture.size();
		for (uint i = 1; i < numLeafNodes; i++)
		{
			const float randNum = (float)rand() / (RAND_MAX + 1.0f);
			const float probabilityToChangeThis = (((float)(numLeafNodes - i)) / numLeafNodes) * 0.5f;
			if (randNum < probabilityToChangeThis)
			{
				const uint swapIndex = rand() % i;
				std::swap(leafNodesCaptureIndirection[i], leafNodesCaptureIndirection[swapIndex]);
			}
		}
	};
	//------------------------

	// Step 3: For each source check and clamp how much dataflow can each leaf cell gather
	for (const auto& src : shuffledSources)
	{
		const TablePos& srcPos = src.first;
		const SourceInfo& srcInfo = src.second;
		float srcRemainingCap = srcInfo.getPower();

		SortAndShuffleLeafNodesFunc(srcPos);

		for (int i = 0; i < leafNodesCaptureIndirection.size(); i++)
		{
			auto& leafNode = leafNodesCapture[i];
			if (leafNode.remainingCap <= 0.0f)
				continue;

			const TablePos& leafPos = leafNode.pos;
			const float maxCapToGetFromSrc = computeScoreForLeafAndSource(leafPos, srcPos, srcInfo);

			// Get the minimum value between: source remaining capacity, how much this leaf node can subtract from source and the leaf node's remaining capacity
			const float actualCapture = std::min(maxCapToGetFromSrc, leafNode.remainingCap);
			const float capFromSrc = std::min(actualCapture, srcRemainingCap);
			
			// Extract the data flow from source and add it to the leaf node
			leafNode.onAddCapture(capFromSrc);			
			srcRemainingCap -= capFromSrc;
			assert(srcRemainingCap >= 0.0f);

			// Don't continue if this source is finished
			if (srcRemainingCap <= 0.0f)
				break;
		}
	}

	// Fill the context with the leaf nodes and how much each can capture
	simContext.mLeafNodeToCaptureValue.clear();
	for (const auto& leafNode : leafNodesCapture)
	{
		simContext.mLeafNodeToCaptureValue.insert(std::make_pair(leafNode.pos, leafNode.currentIterCapSum));
	}

		/*
	if (isLeaf())
	{
		assert(isCoordinateValid(m_row, m_column));
		const float maxFlowFromEnvironment = m_boardView->computeScoreForLeaf(TablePos(m_row, m_column), m_distanceToRoot, true);
		const float capToAdd = std::min(maxFlowFromEnvironment, capRemaining);
		m_bufferedData.add(capToAdd);
	}*/
}

void BoardObject::addRentedResource(const char symbol, const TablePos& tablePos)
{
	RentedResourceInfo info;
	info.symbol = symbol;
	info.pos = tablePos;
	m_rentedResources.insert(info);
	m_board[tablePos.row][tablePos.col].setAsRented();

	assert(m_rentedResources.size() <= g_maxResourcesToRent);
}

bool BoardObject::removeRentedSource(const TablePos& tablePos)
{
	assert(isCoordinateValid(tablePos));
	assert(m_board[tablePos.row][tablePos.col].isRented());

	RentedResourceInfo searchInfo;
	searchInfo.pos = tablePos;
	auto iter = m_rentedResources.find(searchInfo); //std::find_if(m_rentedResources.begin(), m_rentedResources.end(), 
				//			 [&tablePos](const RentedResourceInfo& other) { return (tablePos == other.pos); });

	if (iter == m_rentedResources.end())
	{
		assert(false && " trying to remove a rented resource but not found there is a bug!!");
		return false;
	}

	m_rentedResources.erase(iter);
	return true;
}

void BoardObject::setAvailableSymbols(const std::vector<char>& allSymbols)
{
	for (const char r : allSymbols)
		m_garbageCollectedResources.insert(std::make_pair(r, 0));
}

int BoardObject::getNumAvailableResourcesToRent() const
{
	return g_maxResourcesToRent - (int)m_rentedResources.size();
}

void BoardObject::copyJustCells(const BoardObject& other)
{
	assert(sizeof(m_board) == sizeof(other.m_board));
	//memcpy(m_board, other.m_board, sizeof(other.m_board));

	for (int row = 0; row < MAX_ROWS; row++)
	{
		for (int col = 0; col < MAX_COLS; col++)
		{
			// Horrible hack: TODO make more generic - store / reload rented state
			const bool isRented = other.m_board[row][col].isRented();//m_board[row][col].isRented();
			m_board[row][col].setEmpty();

			if (other.m_board[row][col].isFree() == false)
			{
				m_board[row][col].setSymbol(other.m_board[row][col].m_symbol);

				// Horrible hack - see above comment
				if (isRented)
				{
					m_board[row][col].setAsRented();
				}
			}
		}
	}
}
