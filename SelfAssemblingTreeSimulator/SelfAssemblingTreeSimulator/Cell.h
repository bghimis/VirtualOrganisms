#ifndef CELL_H
#define CELL_H

#include "Utils.h"
#include <assert.h>
#include <iostream>
#include <vector>
#include <unordered_map>

#define EMPTY_SYMBOL ' '

struct BoardObject;
struct SubtreeInfo;

///////////////////////////////////////////////////////////////////////////////
enum DIRECTION
{
	// Explicit directions for tree collector mode
	DIR_LEFT,
	DIR_DOWN,
	DIR_RIGHT,
	DIR_UP,
	DIR_COUNT,
};

DIRECTION getOppositeDirection(const DIRECTION dir);
///////////////////////////////////////////////////////////////////////////////


// The context data instanced and used in the simulation process
struct SimulationContext
{
	// For performance reasons, we currently cache before a simulation how much data flow each leaf node will capture
	std::unordered_map<TablePos, float> mLeafNodeToCaptureValue;
	bool getLeafNodeCapture(const TablePos& leafPos, float& outValue) const;
};

class DataFlowStatistics
{
public:
	DataFlowStatistics(const int maxStats) // Maximum number of ticks for records
	{
		m_maxStats = maxStats;
		m_flowPerTick = new float[maxStats];
		m_head = 0;
	}
	
	virtual ~DataFlowStatistics() 
	{
		delete [] m_flowPerTick;
	}

	
	void clearStats() { m_head = 0; }
	void addStat(const float dataFlow)
	{
		if (m_head >= m_maxStats)
		{
			assert(false && "not enough records stored for statistics or you forgot to reset");
			
			return;
		}

		m_flowPerTick[m_head++] = dataFlow;
	}

	void removeLastValue()
	{
		m_head--;
		assert(m_head >= 0);
	}
	
	float getAvgDataFlow() const
	{
		float sum = 0;
		for (int i = 0; i < m_head; i++)
			sum += m_flowPerTick[i];
		return (sum/m_head);
	}

	DataFlowStatistics* duplicate() const
	{
		DataFlowStatistics* copy = new DataFlowStatistics(m_maxStats);
		return copy;
	}

private:
	float *m_flowPerTick;
	int m_head; // head of current record
	int m_maxStats; // Maximum number of ticks for records
};

struct AvailablePosInfoAndDeltaScore
{
	// The row, column selected for cutting
	int selectedRow;
	int selectedColumn;
	float score;

	int row; // This is the target row to paste
	int col; // target column to paste
	//void fillPositions(const DIRECTION sourceDir, const DIRECTION targetDir, const int startRow, const int startCol, const int subTreeSize);
	//std::vector<std::pair<TablePos, TablePos>> cellsList; // Source and Target positions after cut. // TODOOOOO : this can be reconstructed with some brain effort which i don't have now
	AvailablePosInfoAndDeltaScore() : row(INVALID_POS), col(INVALID_POS), score(0.0f), selectedColumn(INVALID_POS), selectedRow(INVALID_POS) {}

	// Gets the parent coordinates of this subtree's root 
	//void getRootParentPos(int &outRow, int& outCol) const;

	friend std::ostream& operator<<(std::ostream& out, const AvailablePosInfoAndDeltaScore& info);
};


typedef std::vector<AvailablePosInfoAndDeltaScore> AvailablePositionsToMove;

struct ElasticResourceEval {
	float benefit = 0.0f;
	bool isValid() const { return benefit > 0.0f; }
	char symbolAdded = '#';
	TablePos pos;
	BoardObject* bestBoard_onlySymbols;	// Best board found but only with symbols on cells !!! You have to call the updateInternal stuff to create links and other data structures
	void augment(const BoardObject& copyBoard, const char _symbolAdded, const float _benefit, const TablePos& _pos);

	friend std::ostream& operator<<(std::ostream& out, const ElasticResourceEval& data);
};

enum CellType
{
	CELL_NOTSET = 0x01,
	CELL_MEMBRANE = 0x02, // default for tree-collector model
	CELL_INTERIOR = 0x04,
	CELL_EXTERIOR = 0x06,
};

// Definition of a cell / process
struct Cell
{
	char m_symbol = EMPTY_SYMBOL; //Symbol of this cell
	uint m_distanceToRoot;
	Cell *m_left, *m_down; // Left and right childs - actually left-row and down-column childs in our problem
	Cell* m_up;
	Cell* m_right;

	// Previous 
	Cell* m_prevLeft, *m_prevRight, *m_prevUp, *m_prevDown;

	Cell** m_previousByDir[DIR_COUNT]=
	{
		&m_prevLeft,
		&m_prevDown,
		&m_prevRight,
		&m_prevUp
	};

	Cell** m_followersByDir[DIR_COUNT] =
	{
		&m_left,
		&m_down,
		&m_right,
		&m_up
	};

	enum BroadcastEventType
	{
		EVENT_SOURCE_REMOVE,
		EVENT_SOURCE_ADD,
		EVENT_SOURCE_MODIFY,
	};

#if RUNMODE != DIRECTIONAL_MODE
	Cell *m_parent;
#endif

	static const TablePos DIR_OFFSET[DIR_COUNT];

	static const std::unordered_map<char, DIRECTION> g_symbolToDirection;
	static DIRECTION getDirectionFromSymbol(const char symbol) 
	{
		return g_symbolToDirection.find(symbol)->second;
	}

#if RUNMODE == DIRECTIONAL_MODE
	CellType m_cellType = CELL_NOTSET;
#endif

	BoardObject* m_boardView; // This is broadcasted by root. I;m the owner of this, even if it's a pointer

	//bool m_isSource;	// True if this cell is actually a source
	bool m_isEmpty;
	//bool m_hasBranchesBelow;

	// Discovered row and column for this cell
	int m_row, m_column;

	bool m_isRented;


	Cell();
	virtual ~Cell();


	static const char* getDirString(const DIRECTION dir);

	void operator=(const Cell& other);

	void reset(const bool resetSymbolToo = true);
	void resetLinks();

	bool isFree() const 
	{ 
		assert(m_isEmpty == (m_symbol == EMPTY_SYMBOL));
		return m_isEmpty; 
	}

	// Captures from children at maximum the capRemaining capacity
	// You can specify targetChildCellType which considers only the child cells that has that particular type 
	void captureFromChildren(const float capRemaining, const CellType targetChildCellType = CELL_NOTSET);

	// Give a maximum amount of flow to donate to this subtree
	// Return how much it used from that maximum
	float donateFlow(const float maxFlowToDonate);

	void fillChildrenList(Cell* children[], const bool shuffleList = false) const;
	void fillFollowersList(Cell* followers[], const bool shuffleList = false) const;

	// Mark it as empty
	void setEmpty()
	{
		// This cell is removed from the board.
		// If it has a parent then remove the link between parent and it
#if RUNMODE == DIRECTIONAL_MODE
		if (m_prevUp)
			m_prevUp->m_down = nullptr;

		if (m_prevDown)
			m_prevDown->m_up = nullptr;
		
		if (m_prevLeft)
			m_prevLeft->m_right = nullptr;

		if (m_prevRight)
			m_prevRight->m_left = nullptr;
#else
		if (m_parent != nullptr)
		{
			if (m_parent->m_left == this)
				m_parent->m_left = nullptr;

			if (m_parent->m_down == this)
				m_parent->m_down = nullptr;
		}
#endif

		reset();
		m_isEmpty = true;
		m_symbol = EMPTY_SYMBOL;
		m_isRented = false;
	}

	// Mark it as occupied
	void setSymbol(const char _symbol)
	{
		reset();
		m_symbol = _symbol;
		m_isRented = false;

		m_isEmpty = _symbol == EMPTY_SYMBOL;
	}

	void setAsRented()
	{
		m_isRented = true;
	}

	bool isRented() const { return m_isRented; }

	void setAsSource() { assert(false && "not implemented"); }

	// A union of all possible params that can flow through the network
	/*struct BroadcastMsgParams
	{
		BoardObject* structure = nullptr;
	}

	void broadcastMessage(const BroadcastEventType eventType, const BroadcastMsgParams& broadcastParams, const int row, int const col, const int depth);
	*/

	/// Messages simulation ------------------------
	void onMsgBroadcastStructure(BoardObject* structure);
	void onRootMsgBroadcastStructure(BoardObject* structure);
	void onMsgDiscoverStructure(int currRow, int currCol, int depth);
	void onMsgReorganizeStart(std::vector<AvailablePosInfoAndDeltaScore>& output); // Called to reorganize the tree for better performance | On other nodes than root
	void onRootMsgReorganize(); // Called to reorganize the tree for better performance | Root only !
								//----------------------------------------------

	void onMsgReorganizeEnd(int selectedRow, int selectedCol, const AvailablePosInfoAndDeltaScore& targetPosAndDir); // Called only on ROOT !

	// Returns true if elastic model added/removed something
	bool analyzeElasticModel(std::ostream& outDebugStream);
	
	void initFlowStatistics(const int maxNumRecords) { m_flowStatistics = new DataFlowStatistics(maxNumRecords); }
	void addNewFlowRecord(const float value) { m_flowStatistics->addStat(value); }
	void retractLastFlowRecord() { m_flowStatistics->removeLastValue(); }

	// Simulates a serial dataflow capture of data
	// In the serial simulation the update order is deterministic (this is usefully to have proper evaluation of results under low number of real
	// physical processes).
	void simulateTick_serial(const SimulationContext& simContext);

	float getAvgFlow() const { return m_flowStatistics->getAvgDataFlow(); }

	float getRemainingCap() const;

	void beginSimulation() { m_flowStatistics->clearStats(); }

	friend struct Simulator;

#if RUNMODE == DIRECTIONAL_MODE
	bool hasNoPrevNode() const
	{
		return m_prevDown == nullptr && m_prevRight == nullptr && m_prevUp == nullptr && m_prevLeft == nullptr;
	}
#endif

	bool isLeaf() const 
	{
#if RUNMODE == DIRECTIONAL_MODE
		return m_cellType == CELL_EXTERIOR && hasNoPrevNode();
#else
		return m_left == nullptr && m_down == nullptr;
#endif
	}

#if RUNMODE == DIRECTIONAL_MODE
	bool isInteriorLeaf() const
	{
		return m_cellType == CELL_INTERIOR && hasNoPrevNode();
	}
#endif

	bool isRoot() const;

	// Elastic management stuff
	bool root_checkAddResources(std::ostream& outDebugStream);
	bool root_checkRemoveResources(std::ostream& outDebugStream);

	// Data buffering and consuming management
	void addData(const float _value, const bool _ignoreContraints = false) { m_bufferedData.add(_value, _ignoreContraints); }
	void subtractData(const float _value) { m_bufferedData.subtract(_value); }
	float getCurrentBufferedCap() const { return m_bufferedData.getCurrentCap(); }
	void resetCapacityUsedInSubtree();

	float getCachedEnergyConsumedStat() const {
		return m_lastEnergyConsumedStat;
	}

	struct UniversalHash2D
	{
		UniversalHash2D() { reset(); }

		bool cellsHash[MAX_ROWS][MAX_COLS];

		bool isCellSet(const TablePos& pos) const { assert(isCoordinateValid(pos)); return isCellSet(pos.row, pos.col); }
		bool isCellSet(const int row, const int col) const { assert(isCoordinateValid(row, col)); return cellsHash[row][col]; }
		void setCell(const TablePos& pos) { setCell(pos.row, pos.col); }
		void setCell(const int row, const int col) { assert(isCoordinateValid(row, col)); cellsHash[row][col] = true; }
		void reset();
	};


	// This records the flow statistics when requested, on the root only
	DataFlowStatistics* m_flowStatistics;
	
private:
		
	void gatherNewResourcesPos(Cell* cell, std::vector<TablePos>& outPositions, UniversalHash2D& hash);


	// Captures as much as it can from environment (if leaf) or from children if internal node
	void captureDataFlow(const SimulationContext& simContext);

	void elasticBoardCompare(BoardObject& copyBoard, const bool isResourceAdded, const char symbolOfResource, const TablePos& resourcePos, ElasticResourceEval& outResult, 
							 const float oldBenefitValue, std::ostream& outDebugStream);

	
	// How many ticks is data capture disabled for this not because this is a root of a subtree changing its position
	int m_remainingTicksToDelayDataFlowCapture;
	
	// Current buffered data in this node
	BufferedTrafficData m_bufferedData;

	// The energy consumed by this subtree cached at last simTick call
	float m_lastEnergyConsumedStat;
};

#endif
