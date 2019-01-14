#include "Utils.h"
#include <algorithm>
#include "Cell.h"

extern int g_minPowerForWirelessSource;
extern	int g_maxPowerForWirelessSource;

int randRange(int min, int max)
{
	return min + (rand() % static_cast<int>(max - min + 1));
}

bool floatEqual(const float val1, const float val2)
{
	return std::fabsf(val1 - val2) < EPSILON;
}

float randUniform()
{
	return rand() / (float)RAND_MAX;
}

TablePos getRandomTablePos()
{
	TablePos pos;
	pos.row = randRange(0, MAX_ROWS - 1);
	pos.col = randRange(0, MAX_COLS - 1);
	return pos;
}

SourceInfo getRandomSourceInfo()
{
	SourceInfo s;
	s.overridePower((float)randRange(g_minPowerForWirelessSource, g_maxPowerForWirelessSource));
	return s;
}

bool isCoordinateValid(int row, int col)
{
	return (row >= 0 && col >= 0 && row < MAX_ROWS && col < MAX_COLS);
}

bool isCoordinateValid(const TablePos& pos)
{
	return isCoordinateValid(pos.row, pos.col);
}

TablePos get2DNormDir(const TablePos& from, const TablePos& to)
{
	int rowDir = to.row - from.row;
	rowDir = (rowDir < 0 ? -1 : 1) * std::min(std::abs(rowDir), 1);

	int colDir = to.col - from.col;
	colDir = (colDir < 0 ? -1 : 1) * std::min(std::abs(colDir), 1);

	return TablePos(rowDir, colDir);
}

int manhattanDist(const TablePos& p1, const TablePos& p2)
{
	// Remapped to avoid 0 distances !!!
	return 1 + std::abs(p1.row - p2.row) + std::abs(p1.col - p2.col);
}

void trimCommentsAndWhiteSpaces(std::string& str)
{
	static char* whitespaces = " \t";
	static char* comments = "//";

	// Eliminate comments
	const int commentPos = (int)str.find(comments);
	if (commentPos != std::string::npos)
	{
		str.erase(str.begin() + commentPos, str.end());
	}
	else
	{
		int a = 3;
		a++;
	}

	// Left side
	const auto firstNonWhiteSpace = str.find_first_not_of(whitespaces);
	if (firstNonWhiteSpace == std::string::npos)
	{
		str.clear();
		return;
	}
	str.erase(0, firstNonWhiteSpace);

	// Right side
	const auto lastNonWhiteSpace = str.find_last_not_of(whitespaces);
	if (lastNonWhiteSpace == std::string::npos)
	{
		str.clear();
		return;
	}

	str.erase(str.begin() + lastNonWhiteSpace + 1, str.end());
}