#include "BoardObject.h"
#include "Utils.h"
#include <unordered_map>
#include <assert.h>
#include "ExprGenerator.h"
#include <algorithm>
#include <stack>
#include <algorithm>

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

void BoardObject::updateInternalCellsInfo()
{
	// Start from root node
	Cell* rootCell = getRootCell();
	rootCell->m_distanceToRoot = 0;

	rootCell->m_row = m_rootRow;
	rootCell->m_column = m_rootCol;

	assert(rootCell->isRented() == false); // Jesus, i hope not :)
	m_rentedResources.clear();

#if RUNMODE == DIRECTIONAL_MODE
	for (int dirIter = 0; dirIter < DIR_COUNT; dirIter++)
	{
		const int offsetRow = Cell::DIR_OFFSET[dirIter].row;
		const int offsetCol = Cell::DIR_OFFSET[dirIter].col;

		internalCreateLinks(rootCell, (DIRECTION)dirIter, m_rootRow + offsetRow, m_rootCol + offsetCol);
	}
#else
	internalCreateLinks(rootCell, DIR_LEFT, 0, MAX_COLS - 2);
	internalCreateLinks(rootCell, DIR_DOWN, 1, MAX_COLS - 1);
#endif

}

CellType BoardObject::decideCellType(const Cell* startCell, const DIRECTION dir)
{
	if (startCell == nullptr)
	{
		assert(false); // Because i'm not using this use case. if using, remove assert
		return CELL_MEMBRANE;
	}

	if (startCell->m_cellType == CELL_MEMBRANE)
	{
		if ((dir == DIR_LEFT && startCell->symbol == '2')
			|| (dir == DIR_UP && startCell->symbol == '4')
			|| (dir == DIR_RIGHT && startCell->symbol == '7')
			|| (dir == DIR_DOWN && startCell->symbol == 'e'))
		{
			return CELL_EXTERIOR;
		}
		else if ((dir == DIR_RIGHT && startCell->symbol == '2')
			|| (dir == DIR_DOWN && startCell->symbol == '4')
			|| (dir == DIR_LEFT && startCell->symbol == '7')
			|| (dir == DIR_UP && startCell->symbol == 'e'))
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
}

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
		addRentedResource(currCell->symbol, TablePos(row, col));
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

void BoardObject::reset(const bool withoutStatistics)
{
	for (int i = 0; i < MAX_ROWS; i++)
		for (int j = 0; j < MAX_COLS; j++)
		{
			m_board[i][j] = Cell();
		}

	m_posToSourceMap.clear();
	m_numTicksRemainingToUpdateSources = g_powerChangeFrequency;

	if (!withoutStatistics)
	{
		getRootCell()->initFlowStatistics(g_simulationTicksForDataFlowEstimation);
	}
}

void BoardObject::doDataFlowSimulation_serial(const int ticksToSimulate, const bool isRealTick)
{
	Cell* root = getRootCell();
	root->beginSimulation();

	for (int i = 0; i < ticksToSimulate; i++)
	{
		SimulationContext simContext;
		fillSimulationContext(simContext);
		root->simulateTick_serial(simContext);

		if (isRealTick)
		{
			updateSourcesPower();
		}
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

			propagateModifySource(it.first, srcInfo);
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

bool BoardObject::isCompliantWithRowColPatterns(int onlyTestRow, int onlyTestCol) const
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
				if (start != INVALID_POS) // end of a contig row
				{
					if (checkRow(row, start, col - 1) == false)
						return false;

					start = INVALID_POS;
				}
			}
			else if (start == INVALID_POS) // Start of a new row
				start = col;
		}

		if (start != INVALID_POS)
		{
			if (checkRow(row, start, MAX_COLS - 1) == false)
				return false;
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
						return false;

					start = INVALID_POS;
				}
			}
			else if (start == INVALID_POS) // Start of a new row
				start = row;
		}

		if (start != INVALID_POS)
		{
			if (checkCol(col, start, MAX_ROWS - 1) == false)
				return false;
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

	OffsetAndSymbol newEntry = OffsetAndSymbol(rowOff, colOff, currCell.symbol, currCell.isRented());
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
		testLocal += m_board[row][colIter].symbol;
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
		testLocal += m_board[rowIter][col].symbol;
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
	outStream << "Current board: " << endl;

	outStream << ' ' << ' ';
	for (uint j = 0; j < MAX_COLS; j++)
		outStream << ' ' << j << ' ';

	outStream << endl;
	for (uint i = 0; i < MAX_ROWS; i++)
	{
		outStream << i << ' ';

		for (uint j = 0; j < MAX_COLS; j++)
		{
			if (m_board[i][j].isFree())
			{
				outStream << ' ' << BOARD_SKIP_CHARACTER << ' ';
			}
			else
			{
				outStream << ' ' << m_board[i][j].symbol << ' ';
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

bool BoardObject::propagateAddSource(const TablePos& pos, const SourceInfo& sourceInfo)
{
	return internalPropagateAddSource(getRootCell(), pos, sourceInfo);
}

bool BoardObject::propagateModifySource(const TablePos& pos, const SourceInfo& sourceInfo)
{
	return internalPropagateModifySource(getRootCell(), pos, sourceInfo);
}

bool BoardObject::propagateRemoveSource(const TablePos& pos, const bool allSources)
{
	return internalPropagateRemoveSource(getRootCell(), pos, allSources);
}

bool BoardObject::internalPropagateAddSource(Cell* cell, const TablePos& tablePos, const SourceInfo& source)
{
#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	if (cell == nullptr)
		return true;

	const bool resLeft = internalPropagateAddSource(cell->m_left, tablePos, source);
	const bool resRight = internalPropagateAddSource(cell->m_down, tablePos, source);

	const bool resThis = cell->m_boardView->addSource(tablePos, source);
	return (resThis && resLeft && resRight);
}


bool BoardObject::internalPropagateModifySource(Cell* cell, const TablePos& tablePos, const SourceInfo& source)
{
#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	if (cell == nullptr)
		return true;

	const bool resLeft = internalPropagateModifySource(cell->m_left, tablePos, source);
	const bool resRight = internalPropagateModifySource(cell->m_down, tablePos, source);

	const bool resThis = cell->m_boardView->modifySource(tablePos, source);
	return (resThis && resLeft && resRight);
}

bool BoardObject::internalPropagateRemoveSource(Cell* cell, const TablePos& tablePos, const bool allSources)
{
#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	if (cell == nullptr)
		return true;

	const bool resLeft = internalPropagateRemoveSource(cell->m_left, tablePos, allSources);
	const bool resRight = internalPropagateRemoveSource(cell->m_down, tablePos, allSources);

	const bool resThis = cell->m_boardView->removeSource(tablePos, allSources);
	return (resThis && resLeft && resRight);
}

//  How many items are on this column below the given start row
int BoardObject::getFreeItemsOnCol(const int col, const int startRow, const bool down) const
{
#if RUNMODE == DIRECTIONAL_MODE
	int freeItems = 0;
	const int dirOffset = (down ? 1 : -1);

	for (int iRowIter = startRow; ; iRowIter += dirOffset)
	{
		if (isCoordinateValid(iRowIter, col))
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

//  How many items are on this row in the right side from the given start column
int BoardObject::getFreeItemsOnRow(const int row, const int startCol, const bool left) const
{
#if RUNMODE == DIRECTIONAL_MODE
	int freeItems = 0;
	const int dirOffset = (left ? -1 : 1);

	for (int iColIter = startCol; ; iColIter += dirOffset)
	{
		if (isCoordinateValid(row, iColIter))
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
			constraint.setConstraintFirst(m_board[pivotROW][selectedCol].symbol);

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
			constraint.setConstraintLast(m_board[selectedRow][pivotCol].symbol);

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
		
		thisCell->m_prevLeft = prevCell;
		if (prevCell)
			prevCell->m_right = thisCell;

		prevCell = thisCell;
		nextPointer.col++;
	}

	// Produce the 7's
	for (int i = 0; i < m; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('7');
		thisCell->m_cellType = CELL_MEMBRANE;

		if (i == 0)
		{
			thisCell->m_prevLeft = prevCell;
			prevCell->m_right = thisCell;
		}
		else
		{
			thisCell->m_prevUp = prevCell;
			prevCell->m_down = thisCell;
		}

		prevCell = thisCell;
		nextPointer.row++;
	}

	// Produce the e's
	for (int i = 0; i < n + 1; i++)
	{
		Cell* thisCell = &m_board[nextPointer.row][nextPointer.col];
		thisCell->setSymbol('e');
		thisCell->m_cellType = CELL_MEMBRANE;

		if (i == 0)
		{
			thisCell->m_prevUp = prevCell;
			prevCell->m_down = thisCell;
		}
		else
		{
			thisCell->m_prevRight = prevCell;
			prevCell->m_left = thisCell;
		}

		prevCell = thisCell;

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

		if (i == m - 2) // End of the chain, link with the source
		{
			Cell* rootCell = &m_board[nextPointer.row - 1][nextPointer.col];
			assert(nextPointer.row - 1 == m_rootRow && nextPointer.col == nextPointer.col);
			thisCell->m_up = rootCell;
		}
		
		prevCell->m_up = thisCell;
		thisCell->m_prevDown = prevCell;
		prevCell = thisCell;

		nextPointer.row--;
	}
#endif
}

void BoardObject::getBorderPoints(const int rootRow, const int rootCol, TablePos& outMin, TablePos& outMax)
{
	outMin = TablePos(rootRow, rootCol);

	const Cell* currCell = &m_board[rootRow][rootCol];
	int currRow = rootRow;
	int currCol = rootCol;
	while (currCell->m_right) // Go to max right
	{
		currCell = currCell->m_right;
		currCol++;
	}

	// Go to max down
	while (currCell->m_down)
	{
		currCell = currCell->m_down;
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
		assert(startCell->symbol == '4' || startCell->symbol == 'e');

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
		assert(startCell->symbol == '2' || startCell->symbol == '7');

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
				newCell->m_right = cellIter;
				cellIter = newCell;

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
				newCell->m_left = cellIter;
				cellIter = newCell;

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
			propagateAddSource(TablePos(row, col), src);
		}

		// We found a solution !
		getRootCell()->initFlowStatistics(g_simulationTicksForDataFlowEstimation);
		return true;
	}

	return false;
}

void BoardObject::reorganize()
{
	Cell* rootCell = getRootCell();
	assert(rootCell);
	rootCell->onRootMsgReorganize();
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

void BoardObject::reorganizeMaxFlow(int* outNumReorganizationsMade)
{
	int numIterations = 0;
	float prevFlow = 0.0f;
	while (true)
	{
		reorganize();
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
#if RUNMODE == DIRECTIONAL_MODE
	assert(false); // TODO
#endif

	if (currentCell == nullptr)
		return;

	if (currentCell->isLeaf())
	{
		outLeafNodes.push_back(TablePos(currentCell->m_row, currentCell->m_column));
	}
	else
	{
		gatherLeafNodes(currentCell->m_left, outLeafNodes);
		gatherLeafNodes(currentCell->m_down, outLeafNodes);
	}
}

void BoardObject::fillSimulationContext(SimulationContext& simContext) const
{
	// Step1 : Gather all the leaf nodes and fill an augmented data structure
	std::vector<TablePos> leafNodes;
	gatherLeafNodes(getRootCell(), leafNodes);

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
				m_board[row][col].setSymbol(other.m_board[row][col].symbol);

				// Horrible hack - see above comment
				if (isRented)
				{
					m_board[row][col].setAsRented();
				}
			}
		}
	}
}
