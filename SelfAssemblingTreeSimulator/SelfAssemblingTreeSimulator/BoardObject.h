#ifndef BOARD_OBJECT_H
#define BOARD_OBJECT_H

#include "Cell.h"
#include <unordered_set>
#include <regex>
#include "ExprGenerator.h"
#include <set>
#include <ostream>

#define INVALID_FLOW  -1000.0f

// Stores offsets of the subtree elements from the subroot position
// When we apply this on a board, we can apply it wherever it respect the layout

class Expression_Generator;

struct OffsetAndSymbol
{
	int rowOff;
	int colOff;
	char symbol;
	bool isRented;

	OffsetAndSymbol(const int _rowOff, const int _colOff, const char _symbol, const bool _isRented) 
		: rowOff(_rowOff)
		, colOff(_colOff)
		, symbol(_symbol)	
		, isRented(_isRented)
	{}
};


struct RentedResourceInfo
{
	// The position and symbol used for the rented resource
	TablePos pos;
	char symbol;

	bool operator==(const RentedResourceInfo& other) const
	{
		return pos == other.pos;
	}
};

namespace std
{
	template <> struct hash<RentedResourceInfo>
	{
		size_t operator()(const RentedResourceInfo& resource) const
		{
			return std::hash<TablePos>()(resource.pos);
		}
	};
}


struct SubtreeInfo
{
	SubtreeInfo() { reset(); }
	void reset()
	{
		minColOffset = minRowOffset = INVALID_MIN_OFFSET;
		maxColOffset = maxRowOffset = INVALID_MAX_OFFSET;
		m_offsets.clear();
	}

	void add(OffsetAndSymbol& offsetAndSymbol)
	{
		m_offsets.push_back(offsetAndSymbol);

		// Update bounds
		minColOffset = std::min(minColOffset, offsetAndSymbol.colOff);
		maxColOffset = std::max(maxColOffset, offsetAndSymbol.colOff);

		minRowOffset = std::min(minRowOffset, offsetAndSymbol.rowOff);
		maxRowOffset = std::max(maxRowOffset, offsetAndSymbol.rowOff);
	}

	// Min max offsets on row/column (how much does this subtree extends)
	int minRowOffset, maxRowOffset;
	int minColOffset, maxColOffset;

	std::vector<OffsetAndSymbol> m_offsets;
};


struct BoardObject
{
	Cell m_board[MAX_ROWS][MAX_COLS];
	inline Cell& operator()(int row, int col)
	{
		return m_board[row][col];
	}

	inline const Cell& operator()(int row, int col) const
	{
		return m_board[row][col];
	}

	BoardObject();
	BoardObject(const BoardObject& other);
	void operator=(const BoardObject& other);
	virtual ~BoardObject();

#if RUNMODE == DIRECTIONAL_MODE
	enum ProduceItemResult { P_RES_SUCCEED, P_RES_FAILED, P_RES_FINISHED };

	// Try a few random variants to produce according to iterator at nextPointer,
	// Checks inside if produced thing is fine and if the neighb rule is satisfied
	void ProduceItem(TablePos& nextPointer, Expression_Node::Iter& iterator, const int maxAttemptsForTree,
		ProduceItemResult& outRes, int& outNumItemsProduced);

	// Gets the border of membrane by knowing its start pos
	void getBorderPoints(const int rootRow, const int rootCol, TablePos& outMin, TablePos& outMax);

#endif

	struct ResourceAllocatedEval
	{
		ResourceAllocatedEval() : symbol(EMPTY_SYMBOL), flowBenefit(INVALID_FLOW) {}
		TablePos pos; // The position where this was attempted
		char symbol; // The symbol used
		float flowBenefit; // The flow benefit - difference between after to previous
	};


	void copyJustCells(const BoardObject& other);

	bool addSource(const TablePos& pos, const SourceInfo& sourceInfo);
	bool modifySource(const TablePos& pos, const SourceInfo& sourceInfo);
	bool removeSource(const TablePos& pos, const bool allSources = false);

	// Gets the number of valid neighboors on the board around a cell
	int getNumNeighboors(const TablePos& pos) const;
	int getNumNeighboors(const int row, const int column) const { return getNumNeighboors(TablePos(row, column)); }

	// Get all cells on membrane
	void getMembraneCells(std::vector<Cell*>& membraneCells);

	// Tries to expand the current model in order to improve the costs
	void expandExternalTrees();
	void expandInternalTrees();
	
	void optimizeMembrane();
	bool optimizeMembrane_byCutRowCols();
	bool optimizeMembrane_byCutCorners();

	// Update root location
	void updateRootLocation(const int newRootRow, const int newRootCol);

	// Transfer the flow statistics object between old and the new root
	void transferFlowStatistics(const int prevRootRow, const int prevRootCol, const int newRootRow, const int newRootCol);

	// Adds the potential new cells that can be attached to (row, col), and having an existing has2D and a list of symbols that are ready to fill in
	void addPotentialNewCellsAround(std::vector<ResourceAllocatedEval>& eval, const CellType cellType, const int row, const int col, Cell::UniversalHash2D& hash2D, const std::vector<char>& availableSymbolsToFill) const;

	// Get all cells that are child of membrane, and if desired, only those specified by the target parameter (i.e. external, internal or both)
	void getMembraneCellsChildren(std::vector<Cell*>& outList, const CellType targetCellType = CELL_NOTSET);

	// Collects all nodes from a given root and a cell target type to collect
	void collectAllNodesFromRoot(const Cell* root, std::vector<Cell*>& outList, const CellType targetCellType = CELL_NOTSET);

	// Collecting top level subtrees below membrane that doesn't meet the required threshold
	void runGarbageCollector(const float threshold, std::ostream& outStream);

	// Garbage collect the subtree starting at cell
	void garbageCollectSubtree(Cell* root);

	CellType getCellTypeRelativeToMembrane(const int row, const int col) const;

	TablePos selectRandomSource() const;

	void generateDirectionalBoardModel(
		Expression_Generator* m_rowGenerator,
		Expression_Generator* m_colGenerator,
		const int numMaxAtttempts,
		const int depth, const int minMembraneSize, const int maxMembraneSize);

	// Generates 4* left and e* right (middle is considered as 2 OR 7).
	void generateRowExpression(const int middleCol, const int maxAttempts, int depth, const int rowMin, const int rowMax);

	// Generates 7* up and  2* down (middle is considered as 4 OR e).
	void generateColExpression(const int middleRow, const int maxAttempts, const int depth, const int colMin, const int colMax);

	void generateMembrane(const int minMembraneSize, const int maxMembraneSize);

	// Decides if a cell is interior/exterior or membrane depending on starting cell and direction
	CellType decideCellType(const Cell* startCell, const DIRECTION dir);

	Expression_Node::Iter getExprIter(const DIRECTION dir,
		const Expression_Generator* m_rowGenerator,
		Expression_Generator* m_colGenerator) const;

	// Computes the score gained by a leaf node at a specified table position and distance to root
	float computeScoreForLeafAndSource(const TablePos& leafPos, const TablePos& srcPos, const SourceInfo& srcInfo) const;

	// Updates the links starting from the root
	void updateBoardAfterSymbolsInit();
	void updateInternalCellsInfo();

	Cell* getRootCell() { assert(isCoordinateValid(m_rootRow, m_rootCol)); return &m_board[m_rootRow][m_rootCol]; }
	const Cell* getRootCell() const { assert(isCoordinateValid(m_rootRow, m_rootCol)); return &m_board[m_rootRow][m_rootCol]; }
	void setRootLocation(const int row, const int col) { m_rootRow = row; m_rootCol = col; }

	void setAvailableSymbols(const std::vector<char>& allSymbols);

	// A hash of sources with keys from TablePositions (no key collide guaranteed)
	std::unordered_map<TablePos, SourceInfo> m_posToSourceMap;

	void resetCells(const bool resetSymbolsToo = true);
	void reset(const bool withoutStatistics = false, const bool resetSymbolsToo = true);

	void updateSourcesPower();
	void simulateTick_serial(const bool considerForStatistics = true );
	void internal_gatherLeafNodes(const Cell* currentCell, std::vector<TablePos>& outLeafNodes);

	// Set new cell on this board
	void setNewCell(const int row, const int col, const char symbol, const CellType cellType, const bool definitive);

	// Serial (deterministic) simulation of data flow in this board object
	// Second parameter allows you to specify if this is a real simulation on the board or just a flow simulation
	// For instance, when it's not a real tick it will not try to modify the sources' power dynamically or other parts of simulation that are not needed
	void doDataFlowSimulation_serial(const int ticksToSimulate, const bool isRealTick = false, const bool considerForStatistics = true);
	float doDataFlowSimulation_serial_WITHOUT_SIDE_EFFECTS(const int ticksToSimulate);


	// Todo: parallel version

	float getLastSimulationAvgDataFlowPerUnit() const;
	void retractLastSimTickFlowRecord() { getRootCell()->retractLastFlowRecord(); }

	// Checks if the current board filling is compliant with the given patterns on row and column
	// TODO: make it take input parametric not globally
	// Returns the wrong position if you want
	bool isCompliantWithRowColPatterns(int onlyTestRow = INVALID_POS, int onlyTestCol = INVALID_POS,  TablePos* outWrongPos = nullptr) const;
	
	// Checks if we have the same numbers of items after transformations - for debugging
	bool isNumberOfCharactersGood() const;

	// Cuts a subtree from this board at given position and stores it in outSubtree
	void cutSubtree(const int row, const int col, SubtreeInfo& outSubtree);

	// Apply a subtree on this board at given position if possible. 
	// Returns false if there is a position / language issue and the check parameters are true
	bool tryApplySubtree(const int row, const int col, const SubtreeInfo& subtree, const bool checkPositions, const bool checkLanguage);

	// Check if we can paste the tree at the target positions only by considering their positions
	bool canPasteSubtreeAtPos_noLangCheck(const int targetRow, const int targetCol, const SubtreeInfo& subTree) const;

	// Gets all the available position to move the tree rooted in this Cell
	void evaluatePositionsToMove(const int cellRow, const int cellCol, const SubtreeInfo& subtreeCut, AvailablePositionsToMove& outPos, int& outBestOptionIndex) const;

	void printBoard(std::ostream& outStream);

	// Sets the expression on row starting at a position from first character of the expressions
	void setExprOnRow(const int row, const int startCol, const std::string& expr);

	// Sets the expression on columns starting at a position from first character of the expressions
	void setExprOnCol(const int col, const int startRow, const std::string& expr);

	// Clear the expression on row starting at a position of a certain size
	void clearExprOnRow(const int row, const int startCol, const uint size);

	// Clear the expression on column starting at a position of a certain size
	void clearExprOnCol(const int col, const int startRow, const uint size);

	// Propagates source add/modify/remove through all cells' board views within this board
	bool propagateSourceEvent(const Cell::BroadcastEventType srcEventType, const TablePos& pos, const SourceInfo& sourceInfo, const bool allSources);

	// Try several attempts to generate a column at pivotRow with trying of different columns between startCol and endCol
	void generateCol(const int pivotROW, const int startCol, const int endCol, const int depth);
	void generateRow(const int pivotCol, const int rowStart, const int rowEnd, const int depth);

	//  How many items are on this column below the given start row
	int getFreeItemsOnCol(const int col, const int startRow, const bool down = true) const;

	//  How many items are on this row in the right side from the given start column
	int getFreeItemsOnRow(const int row, const int startCol, const bool left = true) const;

	// How many items are continuously occupied on this column given start row and dir
	int getOccupiedItemsOnRow(const int row, const int startCol, const bool left = true, const bool includeMembrane = false) const;
	int getOccupiedItemsOnCol(const int col, const int startRow, const bool down = true, const bool includeMembrane = false) const;

	void setRowAndColGenerators(Expression_Generator* rowGenerator, Expression_Generator* colGenerator)
	{
		m_rowGenerator = rowGenerator;
		m_colGenerator = colGenerator;
	}

	// Generates a random board using the row and col generators specified previously 
	// Consider a numDepthBranches maximum number of branches
	// and numSources
	bool generateRandomBoard(const int numDepthBranches, const int numSources);

	// Gets the total number of nodes on the board
	// Be aware that this doesn't do any caching so its O(N*M).
	int countNodes() const;

	// Reorganize for maximal flow output in the current configuration
	void reorganize(std::ostream& outStream);

	// Call reorganize as much as we can update the flow
	void reorganizeMaxFlow(int* outNumReorganizationsMade);


	void fillSimulationContext(SimulationContext& simContext) const;

	bool isPosFree(const TablePos& pos) const { return m_board[pos.row][pos.col].isFree(); }
	bool isPosFree(const int row, const int col) const { return m_board[row][col].isFree(); }

	// The list of all currently rented resources and utility functions
	int getNumAvailableResourcesToRent() const;
	bool removeRentedSource(const TablePos& tablePos);
	void addRentedResource(const char symbol, const TablePos& tablePos);

	std::unordered_set<RentedResourceInfo> m_rentedResources;

private:

	// Cut membrane functionality
	//------
	bool canCutColumn(const int column) const;
	bool canCutRow(const int row) const;

	typedef void (BoardObject::*membraneCutFunctorType)(const int index, const DIRECTION dir, const bool definitive) ;
	void cutMembraneByRow(const int index, const DIRECTION dir, const bool definitive = false);
	void cutMembraneByCol(const int index, const DIRECTION dir, const bool definitive = false);

	struct CutCornerDescription
	{
		enum DIRECTION_TYPE
		{
			NORTH_WEST,
			DIR_TYPE_COUNT,
		};

		// Start, middle, end indexes that define the inflexion points where the corner is cut
		int indexS, indexM, indexE;

		// How long does the cut goes on y and x row axes
		int yOffset, xOffset; 

		DIRECTION_TYPE dirType = DIR_TYPE_COUNT;

		void reset()
		{
			dirType = DIR_TYPE_COUNT;
			indexS = indexM = indexE = INVALID_POS;
			yOffset = xOffset = INVALID_POS;
		}

		CutCornerDescription() { reset(); }
	};

	bool evaluateBestCornerCutChance(const std::vector<TablePos>& inflexionPoints, CutCornerDescription& outBestCornerCutDesc, float& flowBenefit);
	bool cutMembraneCorner(const std::vector<TablePos>& inflexionPoints, CutCornerDescription& membraneCornerDesc, const bool definitive = false);


	// Returns true if the evaluation was successfully.
	// Give the column to cut; 
	// outputs the direction (LEFT / RIGHT to shift) and the flow difference than the given baseline (original board)
	bool evaluateMembraneCut(membraneCutFunctorType func, const DIRECTION dirs[2], const int colIter, const float baselineFlowAvg, DIRECTION& outDir, float& outFlowDiff) const;


	// Returns true if any valid.
	// Returns row != INVALID_POS for the row to be cut
	// Returns col != INVALID_POS for the col to be cut
	// Returns the flowBenefit of the best between row/col
	bool evaluateMembraneOptimization(int& row, int &col, DIRECTION& outDirToCut, float& flowBenefit);

	// =====

	// From symbol to number of collected resources
	// m_garbageCollectedResources['e'] how many resources are available for type 'e'
	std::unordered_map<char, int> m_garbageCollectedResources;

	// Min Max of membrane bounds per each column/row
	std::pair<int, int> m_membraneBoundsPerRow[MAX_ROWS + 1];
	std::pair<int, int> m_membraneBoundsPerCol[MAX_COLS + 1];
	std::pair<int, int> m_membraneBoundsRows; // Minimum and maximum for rows and columns delimiting the membrane's bounding box
	std::pair<int, int> m_membraneBoundsCols;

	// Updated for each row and column the min, max values
	void updateMembraneBounds(std::vector<Cell*>& membraneCells);

	// allSources is used for removing event only
	bool internalPropagateSourceEvent(const Cell::BroadcastEventType srcEventType, Cell* cell, const TablePos& tablePos, const SourceInfo& source, const bool allSources, const int depth);
	
	// Recursively cuts the subtree starting at currCell - used by cutSubtree public func
	void internalCutSubtree(const Cell& currCell, const int rowOff, const int colOff, SubtreeInfo& outSubtree);

	// Helper to perform a deep copy of data from another board object
	void copyDataFrom(const BoardObject& other);

#if RUNMODE == DIRECTIONAL_MODE
	// Used inside update links

	// Updates the membrane by giving the min and max points of the rectangle defining membrane
	// TODO: split in two - get membrane points and connect inflexion points
	void getInflexionPointAndConnectMembrane(std::vector<TablePos>& outInflexionPositions, const bool justGetInflexionPoint = false);

	// Updates the rows attached to the left / right of the given interval 
	void internalCreateRowsLinks(const int middleColumn, const int rowStart, const int rowEnd);

	// Updates the columns attached to the up / down rows of the given interval
	void internalCreateColsLinks(const int middleRow, const int colMin, const int colMax);
	
#else
	// Used inside update links
	void internalCreateLinks(Cell* prevCell, const DIRECTION dir, const int row, const int col);
#endif

	bool checkRow(const int row, const int startCol, const int endCol) const;
	
	bool checkCol(const int col, const int startRow, const int endRow) const;

	void gatherLeafNodes(const Cell* currentCell, std::vector<TablePos>& outLeafNodes) const;

	Expression_Generator* m_rowGenerator;
	Expression_Generator* m_colGenerator;
	int m_numTicksRemainingToUpdateSources; // THe number of ticks remaining when all sources' targets should be updated

	int m_rootRow = INVALID_POS; 
	int m_rootCol = INVALID_POS;
};


#endif
