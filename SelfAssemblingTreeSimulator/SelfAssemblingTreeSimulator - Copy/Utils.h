#ifndef UTILS_H
#define UTILS_H

#define DIRECTIONAL_MODE 1
#define LEFTRIGHTONLY_MODE 0

#define RUNMODE DIRECTIONAL_MODE
#define MIN_MEMBRANE_SIZE 7
#define MAX_MEMBRANE_SIZE 7

#include <unordered_map>
#include <string>
#include <assert.h>
#include <atomic>
#include <limits.h>

using uint = unsigned int;
#define MAX_ROWS 15
#define MAX_COLS 15

#define INVALID_POS -1
#define MIN_SCORE 0.0f
#define BOARD_SKIP_CHARACTER '*'
#define SIMULATION_BOARD_HISTORY_SIZE 10

#define INVALID_MAX_OFFSET INT_MIN
#define INVALID_MIN_OFFSET INT_MAX

// TODO: optimize the complexity of this op
#define COMPUTE_SCORE_BY_SIMULATION


#define INVALID_COST_PER_RESOURCE -1.0f

struct TablePos
{
	TablePos() : row(INVALID_POS), col(INVALID_POS) {}
	TablePos(int _row, int _col) :row(_row), col(_col) {}
	int row, col;

	bool operator==(const TablePos& other) const {
		return row == other.row && col == other.col;
	}
};
 
int randRange(int min, int max);
bool isCoordinateValid(int row, int col);
bool isCoordinateValid(const TablePos& pos);
float randUniform();

int manhattanDist(const TablePos& p1, const TablePos& p2);
TablePos getRandomTablePos();

struct SourceInfo
{
	SourceInfo() : currentPower(0.0f), powerTarget(0.0f){}

	float getPower() const { return currentPower; }
	float getTarget() const { return powerTarget; }

	void setPowerTarget(float value) { powerTarget = value; }
	void setCurrentPower(float value) { currentPower = value; }

	// This function overrides the current power target
	void overridePower(float value)
	{
		currentPower = value;
		powerTarget = value;
	}

private:
	float currentPower;	// This is the actual power of the source
	float powerTarget;	// This is the power target that this source is trying to achieve
};

SourceInfo getRandomSourceInfo();

namespace std
{
	template <> struct hash<TablePos>
	{
		size_t operator()(const TablePos& tablePos) const
		{
			return tablePos.row * MAX_COLS + tablePos.col;
		}
	};
}

void trimCommentsAndWhiteSpaces(std::string& str);

struct BufferedTrafficData
{
	BufferedTrafficData(float maxFlowSize) :m_maxFlowSize(maxFlowSize), m_value(0.0f){}
	
	float m_value; // TODO: atomic / mutex something

	void add(const float _value) 
	{ 
		assert(m_value <= m_maxFlowSize);
		m_value += _value;
		assert(m_value <= m_maxFlowSize);
	}

	void subtract(const float _value)
	{
		assert(m_value >= 0.0f);
		m_value -= _value;
		assert(m_value >= 0.0f);
	}

	float getCurrentCap() const { return m_value; }

private:
	const float m_maxFlowSize; // MAXIMUM FLOW SIZE SUPPORTED for this buffer
};

template <typename T>
T mysgn(T value)
{
	return (T(0.0f) < value) - (value < T(0.0f));
}

#endif
