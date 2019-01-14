#ifndef EXPR_GENERATOR_H
#define EXPR_GENERATOR_H

#include <string>
#include <vector>
#include <assert.h>
#include "Utils.h"
#include <algorithm>

// Result def of a matching operation
struct ExprMatchResult
{
	ExprMatchResult(const int maxRes)
	{
		m_str.reserve(maxRes + 1);
		reset();
	}

	ExprMatchResult()
	{
		reset();
	}

	void reset()
	{
		m_consumed = 0;
		m_valid = false;
		m_str.clear();
	}

	void append(const std::string& c)
	{
		m_str.append(c);
		m_consumed++;
	}

	void setValid(const bool isValid) { m_valid = isValid; }

	int m_consumed; // length of the result
	std::string m_str; // The actual string
	bool m_valid; // True if the result is valid
};

class Expression_Node
{
public:
	Expression_Node(std::string const& value_in = "", bool const& leaf_in = false, bool const& more_in = false, bool const& star_in = false, bool const& plus_in = false, int minItems_in = -1)
		: value(value_in)
		, isORNode(more_in)
		, plus(plus_in)
		, minItems(minItems_in)
		, star(star_in)
		, leaf(leaf_in)
	{

	}

	int minItems;
	std::string value;
	bool isORNode;
	bool star;
	bool plus;
	bool leaf;
	std::vector<Expression_Node> children;

	// Do not touch - internal computation stuff
	int m_cachedLastPositionToMeetConstraint;
	int m_cachedFirstPositionToMeetConstraint;
	//----

	struct Iter
	{
		int currChildrenIndex = 0;
		const Expression_Node* objIterated = nullptr;
		int numChildrensInObjIterated = 0; // objIterated->children.size();
		bool inverse = false;

		const Expression_Node* getNext()
		{
			if (numChildrensInObjIterated == 0) // self node
			{
				if (currChildrenIndex == 0) // First child requested ?
					return objIterated;
				else
					return nullptr;
			}
			else
			{
				const auto nextIndexToReturn = currChildrenIndex;
				currChildrenIndex = inverse ? currChildrenIndex + 1 : currChildrenIndex - 1;

				if (nextIndexToReturn < numChildrensInObjIterated && nextIndexToReturn >= 0)
					return &objIterated->children[nextIndexToReturn];
				else
					return nullptr;
			}
		}
	};

	Iter getIterator(const bool inverse = false) const
	{
		Iter iter;
		iter.objIterated = this;
		iter.numChildrensInObjIterated = (int)iter.objIterated->children.size();
		iter.inverse = inverse;
		iter.currChildrenIndex = inverse ? iter.numChildrensInObjIterated - 1 : 0;

		return iter;
	}

	void addNode(Expression_Node& node)
	{
		children.push_back(node);
	}

	// Min number of characters that this Node would expand
	uint totalMinCharactersToExtend;
	std::vector<int> minCharactersToExpand; // minCharactersToExpand[i] = min number of characters needed to expand starting the children in the right of node i (not including i)
};

#define ADD_SYMBOL_IF_NOT_EMPTY(temp) { if (!temp.empty()) {tokens.push_back(temp);  assert(temp.size() == 1); outSymbols.push_back(temp.c_str()[0]);} } // Currently we support symbols of size 1 in this impl

class Expression_Scanner
{
public: Expression_Scanner() = delete;
public: static std::vector<std::string> Scan(std::string const& expression, std::vector<char>& outSymbols)
{
	std::vector<std::string> tokens;
	std::string temp;

	for (auto const& it : expression)
	{
		if (it == '(')
		{
			ADD_SYMBOL_IF_NOT_EMPTY(temp);
			tokens.push_back("(");
			temp.clear();
		}

		else if (it == '|')
		{
			ADD_SYMBOL_IF_NOT_EMPTY(temp);
			tokens.push_back("|");
			temp.clear();
		}

		else if (it == ')')
		{
			ADD_SYMBOL_IF_NOT_EMPTY(temp);
			tokens.push_back(")");
			temp.clear();
		}
		else if (it == '*')
		{
			ADD_SYMBOL_IF_NOT_EMPTY(temp);
			tokens.push_back("*");
			temp.clear();
		}
		else if (it == '+')
		{
			ADD_SYMBOL_IF_NOT_EMPTY(temp);
			tokens.push_back("+");
			temp.clear();
		}
		else if (isalpha(it) || it == ' ' || isdigit(it))
		{
			temp += it;
		}

	}

	ADD_SYMBOL_IF_NOT_EMPTY(temp);
	return tokens;
}
};

class Expression_Parser
{
	Expression_Parser() = delete;

public:

	//get parse tree
	static Expression_Node Parse(std::vector<std::string> const& tokens)
	{
		auto it = tokens.cbegin();
		auto end = tokens.cend() - 1;
		auto parse_tree = Build_Parse_Tree(it, end);

		ComputeLimits(parse_tree);
		return parse_tree;
	}

private:

	using ParseTreeNodesIterator = std::vector<std::string>::const_iterator;

	static uint ComputeLimits(Expression_Node& node)
	{
		const uint numChildren = (uint)node.children.size();
		node.minCharactersToExpand.resize(numChildren);

		// Compute how many characters we need
		node.totalMinCharactersToExtend = 0;
		const bool isORNode = node.isORNode;
		for (uint i = 0; i < numChildren; i++)
		{
			const uint localRes = ComputeLimits(node.children[i]);

			if (isORNode)
			{
				node.totalMinCharactersToExtend = std::max(node.totalMinCharactersToExtend, localRes);
			}
			else
			{
				node.totalMinCharactersToExtend += localRes;
			}

			node.minCharactersToExpand[i] = localRes;
		}

		if (!node.children.empty())
		{
			uint prev = node.minCharactersToExpand[numChildren - 1];
			node.minCharactersToExpand[numChildren - 1] = 0;

			for (int i = (int)numChildren - 2; i >= 0; i--)
			{
				uint newPrev = node.minCharactersToExpand[i];
				node.minCharactersToExpand[i] = prev + node.minCharactersToExpand[i + 1];
				prev = newPrev;
			}
		}

		if (isORNode == false)
		{
			// If leaf
			if (node.children.empty())
			{
				node.totalMinCharactersToExtend += 1;
			}
		}

		return node.totalMinCharactersToExtend;
	}

	static Expression_Node Build_Parse_Tree(std::vector<std::string>::const_iterator & it, std::vector<std::string>::const_iterator const& end)
	{
		Expression_Node root("", false);

		bool isORNode = false;
		auto begin = it;
		// Check if at this level there are one or more '|' symbols and mark the intervals if there are
		std::vector<std::pair<ParseTreeNodesIterator, ParseTreeNodesIterator>> orSplitIntervals;
		{
			int numPharanthesisOpen = 0;
			auto currentIntervalBegin = begin;
			while (begin <= end)
			{
				if (*begin == "(")
					numPharanthesisOpen++;
				else if (*begin == ")")
					numPharanthesisOpen--;

				else if (*begin == "|")
				{
					if (numPharanthesisOpen == 0)
					{
						isORNode = true;
						orSplitIntervals.push_back(std::make_pair(currentIntervalBegin, begin - 1));
						currentIntervalBegin = begin + 1;
					}
				}

				begin++;
			}

			// Add the last interval			
			orSplitIntervals.push_back(std::make_pair(currentIntervalBegin, end));
		}

		if (isORNode)
		{
			root.isORNode = true;
			for (auto& it : orSplitIntervals)
			{
				Expression_Node nodeInner = Build_Parse_Tree(it.first, it.second);
				root.addNode(nodeInner);
			}
		}
		else
		{
			// TODO: check if needed to continue
			while (it <= end)
			{
				// Add words always
				if (Is_Word(*it))
				{
					//Check for star or + operator
					bool isStar = false;
					bool isPlus = false;
					auto itNext = it;
					itNext++;
					const std::string& word = *it;
					if (itNext <= end)
					{
						if (*itNext == "*")
						{
							isStar = true;
							it = itNext;
						}
						else if (*itNext == "+")
						{
							isPlus = true;
							it = itNext;
						}
					}

					Expression_Node newNode(word, true, false, isStar, isPlus);
					root.addNode(newNode);
				}

				//when we see a "(", build the subtree,
				else if (*it == "(")
				{
					++it;

					// Find the position of the coresponding ")"
					auto itSearch = it;
					int numOpened = 1;
					bool found = false;
					while (itSearch <= end)
					{
						if (*itSearch == "(")
							numOpened++;
						else if (*itSearch == ")")
						{
							numOpened--;
							if (numOpened == 0)
							{
								found = true;
								break;
							}
						}

						itSearch++;
					}

					if (found == false)
					{
						assert(false && "couldn't found end parathesis. stopping parsing");
						return root;
					}

					Expression_Node innerNode = Build_Parse_Tree(it, itSearch - 1);
					// Check for (..)* or (..)+
					if (itSearch + 1 <= end)
					{
						auto nextItSearch = itSearch + 1;
						const std::string nextChar = *nextItSearch;
						const bool isNextCharStar = nextChar == "*";
						const bool isNextCharPlus = nextChar == "+";
						if (isNextCharPlus || isNextCharStar)
						{
							innerNode.star = isNextCharStar;
							innerNode.plus = isNextCharPlus;
							itSearch++;
						}
					}
					root.addNode(innerNode);
					it = itSearch;
				}
				else if (*it == ")")
				{
					assert("Incorrect mathing !!. Shouldn't start with an )");
				}
				else
				{
					assert(false && "invalid");
				}

				++it;
			}
		}

		// Prune the root for debugging and efficiency.
		// If only one child node => set this node as root but copy its details too to the parent
		{
			const int numChildren = (int)root.children.size();
			if (numChildren == 1)
			{
				root = root.children[0];
			}
		}

		return root;
	}


	static bool Is_Word(std::string const& it)
	{
		return (it != "(" && it != "|" && it != ")" && it != "*");
	}
};

// For now, we support only first/last character constraint
struct Constraint
{
	bool constrainFirst = false;
	bool constrainLast = false;
	std::string firstChar = "z";
	std::string lastChar = "z";

	void setConstraintLast(const char _lastChar) 
	{
		constrainLast = true;
		lastChar.clear();
		lastChar += _lastChar;
	}

	void setConstraintFirst(const char _firstChar)
	{
		constrainFirst = true;
		firstChar.clear();
		firstChar += _firstChar;
	}
};

class Expression_Generator
{

	//constructors
public:
	Expression_Generator()
	{

	}

	void init(const std::string& expression, std::vector<char>& outSymbols)
	{
		auto tokens = Expression_Scanner::Scan(expression, outSymbols);
		m_parse_tree = Expression_Parser::Parse(tokens);
	}

	inline int getMinSize() const
	{
		return m_parse_tree.totalMinCharactersToExtend;
	}

	static const int INVALID_MEET_CONSTRAINT = -1;

	bool CheckConstraintPossible(Expression_Node& root, const Constraint* constraint)
	{
		if (constraint == nullptr)
			return true;

		root.m_cachedFirstPositionToMeetConstraint = INVALID_MEET_CONSTRAINT;
		root.m_cachedLastPositionToMeetConstraint = INVALID_MEET_CONSTRAINT;

		const uint numChildren = (int)root.children.size();
		// IF leaf, check the constraint
		if (numChildren == 0)
		{
			if (constraint->constrainFirst && constraint->firstChar != root.value)
				return false;

			if (constraint->constrainLast && constraint->lastChar != root.value)
				return false;

			root.m_cachedFirstPositionToMeetConstraint = 0;
			root.m_cachedLastPositionToMeetConstraint = 0;
			return true;
		}
		else
		{
			if (root.isORNode)
			{
				bool isOneOfThemGood = false;
				// One of them needs to satisfy the constraint
				for (uint i = 0; i < root.children.size(); i++)
				{
					if (CheckConstraintPossible(root.children[i], constraint))
					{
						isOneOfThemGood = true;
						break;
					}
				}

				return isOneOfThemGood;
			}
			else
			{
				if (constraint->constrainFirst)
				{
					// Check begin part
					Constraint newConstraint;
					newConstraint.constrainFirst = true;
					newConstraint.firstChar = constraint->firstChar;

					bool possible = false;
					for (uint nodeToTest = 0; nodeToTest < numChildren; nodeToTest++)
					{
						if (CheckConstraintPossible(root.children[nodeToTest], &newConstraint))
						{
							root.m_cachedFirstPositionToMeetConstraint = nodeToTest;
							possible = true;
							break;
						}

						// If this is a star node we can continue
						if (root.children[nodeToTest].star == false)
							return false;
					}

					if (possible == false)
						return false;
				}

				if (constraint->constrainLast)
				{
					// Check end part
					Constraint newConstraint;
					newConstraint.constrainLast = true;
					newConstraint.lastChar = constraint->lastChar;

					bool possible = false;
					for (int nodeToTest = numChildren - 1; nodeToTest >= 0; nodeToTest++)
					{
						if (CheckConstraintPossible(root.children[nodeToTest], &newConstraint))
						{
							root.m_cachedLastPositionToMeetConstraint = nodeToTest;
							possible = true;
							break;
						}

						// If this is a star node we can continue
						if (root.children[nodeToTest].star == false)
							return false;
					}

					if (possible == false)
						return false;
				}
				 
				return true;
			}
		}
	}

	// Returns is successfully or not
	// returns the outResult of a string matching expression with a max length of limit
	bool GenerateRandom(const uint limit, ExprMatchResult& outResult, const Constraint* constraint = nullptr)
	{
		outResult.reset();
		if (!CheckConstraintPossible(m_parse_tree, constraint))
		{
			outResult.setValid(false);
			return false;
		}

		const bool res = internalGenerateRandom(m_parse_tree, limit, outResult, constraint);
		assert(res == outResult.m_valid);
		return res;
	}

	/*
	using ExprMatchResultsArray = std::vector<ExprMatchResult>;
	// Returns the results of ALL matching expressions within a max length of limit
	bool GenerateAll(const uint limit, ExprMatchResultsArray& results, const Constraint* constraint = nullptr)
	{
		results.clear();

		if (!CheckConstraintPossible(m_parse_tree, constraint))
			return false;

		ExprMatchResult emptyResult;
		return internalGenerateAll(m_parse_tree, limit, emptyResult, results, constraint);
	}
	*/

	// Reverse = true for reversed iterator
	Expression_Node::Iter getIter(const bool reverse = false) const { return m_parse_tree.getIterator(reverse); }

private:

	// If the constraint is needed on this leaf node it will be added to result and updates the sender's variables.
	static bool checkConstraintOnLeaf(const Expression_Node& root, const Constraint* constraint, uint& limit, ExprMatchResult& outResult)
	{
		// Is this the last constrained character ? Put it and return
		if (constraint && (constraint->constrainLast || constraint->constrainFirst))
		{
			assert(limit > 0);
			outResult.append(constraint->constrainLast ? constraint->lastChar : constraint->firstChar);
			limit--;
			return true;
		}
		
		return false;
	}

public:
	static bool internalGenerateRandom(const Expression_Node& root, uint limit, ExprMatchResult& outResult, const Constraint* constraint)
	{
		if (root.totalMinCharactersToExtend > limit && root.star == false)
		{
			assert(false && "Limit is not enough !");
			outResult.setValid(false);
			return false;
		}

		const int numChildren = (int)root.children.size();
		const bool isMultiplicativeNode = root.star == true || root.plus == true;
		if (numChildren == 0) // Leaf node >
		{
			if (checkConstraintOnLeaf(root, constraint, limit, outResult))
			{
				outResult.setValid(true);
				return true;
			}

			int numInstances = 0;
			numInstances = randRange(root.star ? 0 : 1, isMultiplicativeNode ? limit : 1);
			for (int i = 0; i < numInstances; i++)
			{
				outResult.append(root.value);
			}
		}
		else
		{
			if (root.isORNode)
			{
				if (checkConstraintOnLeaf(root, constraint, limit, outResult))
				{
					outResult.setValid(true);
					return true;
				}

				int numInstances = 0;
				numInstances = randRange(root.star ? 0 : 1, isMultiplicativeNode ? limit / root.totalMinCharactersToExtend : 1);
				for (int i = 0; i < numInstances; i++)
				{
					// Randomize one choice
					const uint choice = randRange(0, numChildren - 1);

					internalGenerateRandom(root.children[choice], root.totalMinCharactersToExtend, outResult, constraint);
				}
			}
			else
			{
				int startChild = 0;
				int endChild = numChildren - 1;

				// If first item is constrained, print the first item then go to the next node
				if (constraint && constraint->constrainFirst)
				{
					assert(root.m_cachedFirstPositionToMeetConstraint != INVALID_MEET_CONSTRAINT);
					
					// If this node is star or plus we let it derive more characters from this node not only the constrained one
					const Expression_Node& requiredNode = root.children[root.m_cachedFirstPositionToMeetConstraint];
					const int isRequiredNodeIndexStarOrPlus = (requiredNode.star || requiredNode.plus);
					startChild = isRequiredNodeIndexStarOrPlus ? root.m_cachedFirstPositionToMeetConstraint : root.m_cachedFirstPositionToMeetConstraint + 1;

					// WRITE IN THE OTHER SIDE !!! ON LAST NODE
						
					outResult.append(constraint->firstChar);
					limit--;
				}
				

				const bool isLastConstrained = constraint && constraint->constrainLast;
				if (isLastConstrained)
				{
					// Edge case when limit is already 0 but user requested both ends constraints 
					if (limit == 0 && root.m_cachedFirstPositionToMeetConstraint == root.m_cachedLastPositionToMeetConstraint)
					{
						outResult.setValid(true);
						return true;
					}

					assert(root.m_cachedLastPositionToMeetConstraint != INVALID_MEET_CONSTRAINT);
					endChild = (root.star || root.plus) ? root.m_cachedLastPositionToMeetConstraint : root.m_cachedLastPositionToMeetConstraint - 1;
					limit--;
				}

				for (int i = startChild; i <= endChild; i++)
				{
					const uint minToFillNextChildren = root.minCharactersToExpand[i];
					const int maxAssignedForThisNode = limit - minToFillNextChildren - root.children[i].totalMinCharactersToExtend;
					if (maxAssignedForThisNode < 0)
					{
						assert(false && " limit not enough");
						outResult.setValid(false);
						return false;
					}

					internalGenerateRandom(root.children[i], maxAssignedForThisNode, outResult, nullptr);
				}

				if (isLastConstrained)
				{
					outResult.append(constraint->lastChar);
					limit--;
				}
			}
		}

		outResult.setValid(true);
		return true;
	}

private:

	/*
	// currentExpr - the current expr being analyzed for extension
	// localResults - the results obtained by extending currentExpr after this call
	// globalResultsCollector - the results collector gathered after obtaining the final results.
	bool internalGenerateAll(const Expression_Node& root, uint limit, ExprMatchResult& currentExpr, ExprMatchResultsArray& localResultsCollector, ExprMatchResultsArray& globalResultsCollector, const Constraint* constraint)
	{
		localResultsCollector.clear();
		if (root.totalMinCharactersToExtend > limit && root.star == false)
		{
			assert(false && "Limit is not enough !");
			currentExpr.setValid(false);
			return false;
		}

		// TODO: check here if we should add it to the collector array
		const int numChildren = (int)root.children.size();
		const bool isMultiplicativeNode = root.star == true || root.plus == true;
		if (numChildren == 0) // Leaf node >
		{
			if (checkConstraintOnLeaf(root, constraint, limit, currentExpr))
			{
				currentExpr.setValid(true);
				return true;
			}

			// Expand the currentExpr with all possible ranges of instance
			const int lowNumInstances = root.star ? 0 : 1;
			const int highNumInstances = isMultiplicativeNode ? limit : 1;

			for (int instances = lowNumInstances; instances <= highNumInstances; instances++)
			{
				ExprMatchResult newRes = currentExpr;
				for (int i = 0; i < instances; i++)
				{
					newRes.append(root.value);
				}

				localResultsCollector.push_back(newRes);
			}
		}
		else
		{
			if (root.isORNode)
			{
				if (checkConstraintOnLeaf(root, constraint, limit, currentExpr))
				{
					currentExpr.setValid(true);
					return true;
				}

				int numInstances = 0;
				const int lowNumInstances = root.star ? 0 : 1;
				const int highNumInstances = isMultiplicativeNode ? limit / root.totalMinCharactersToExtend : 1;

				for (int instances = lowNumInstances; instances <= highNumInstances; instances++)
				{
					ExprMatchResult newRes = currentExpr;
					for (int i = 0; i < instances; i++)
					{
						internalGenerateRandom(root.children[choice], root.totalMinCharactersToExtend, outResult, constraint);
					}

					localResultsCollector.push_back(newRes);
				}
			}
			else
			{
				int startChild = 0;
				int endChild = numChildren - 1;

				// If first item is constrained, print the first item then go to the next node
				if (constraint && constraint->constrainFirst)
				{
					assert(root.m_cachedFirstPositionToMeetConstraint != INVALID_MEET_CONSTRAINT);

					// If this node is star or plus we let it derive more characters from this node not only the constrained one
					const Expression_Node& requiredNode = root.children[root.m_cachedFirstPositionToMeetConstraint];
					const int isRequiredNodeIndexStarOrPlus = (requiredNode.star || requiredNode.plus);
					startChild = isRequiredNodeIndexStarOrPlus ? root.m_cachedFirstPositionToMeetConstraint : root.m_cachedFirstPositionToMeetConstraint + 1;

					// WRITE IN THE OTHER SIDE !!! ON LAST NODE

					currentExpr.append(constraint->firstChar);
					limit--;
				}

				const bool isLastConstrained = constraint && constraint->constrainLast;
				if (isLastConstrained)
				{
					// Edge case when limit is already 0 but user requested both ends constraints 
					if (limit == 0 && root.m_cachedFirstPositionToMeetConstraint == root.m_cachedLastPositionToMeetConstraint)
					{
						globalResultsCollector.push_back(currentExpr);
						currentExpr.setValid(true);
						return true;
					}

					assert(root.m_cachedLastPositiobnToMeetConstraint != INVALID_MEET_CONSTRAINT);
					endChild = (root.star || root.plus) ? root.m_cachedLastPositionToMeetConstraint : root.m_cachedLastPositionToMeetConstraint - 1;
					limit--;
				}

				ExprMatchResultsArray currResults = { currentExpr };
				ExprMatchResultsArray newLocalResults;
				ExprMatchResultsArray* frontArray = &currResults;
				ExprMatchResultsArray* backArray = &newLocalResults;
				
				int numIterations = 0;
				for (int i = startChild; i <= endChild; i++)
				{
					numIterations++;

					// Expand each current result
					for (ExprMatchResult currentLocalRes : *frontArray)
					{
						backArray->clear();

						const uint minToFillNextChildren = root.minCharactersToExpand[i];
						const int maxAssignedForThisNode = currentLocalRes.limit - minToFillNextChildren - root.children[i].totalMinCharactersToExtend;
						if (maxAssignedForThisNode < 0)
						{
							assert(false && " limit not enough");
							//expr.setValid(false);
							//return false;
							continue;
						}

						internalGenerateAll(root.children[i], maxAssignedForThisNode, currentLocalRes, *backArray, globalResultsCollector, constraint);
					}

					std::swap(frontArray, backArray);
				}

				// Copy the output to the local results collector
				const ExprMatchResultsArray* targetOutput = (numIterations % 2 == 0 ? frontArray : backArray);
				localResultsCollector.clear();
				localResultsCollector.insert(localResultsCollector.end(), targetOutput->begin(), targetOutput->end());

				if (isLastConstrained)
				{
					// For each result in the local collector, append the last constrained char
					for (ExprMatchResult res : localResultsCollector)
					{
						res.append(constraint->lastChar);
					}

					limit--;
				}
			}
		}

		return true;
	}
	*/

	Expression_Node m_parse_tree;
	unsigned int amount;
};

#endif
