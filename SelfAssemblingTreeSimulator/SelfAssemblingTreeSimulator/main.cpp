#include <iostream>
#include "SimulatorBoard.h"
#include <string.h>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;

// Parameter variables
int FIXED_SEED = 0;
bool isFixedSeed = true;
std::string exprForRows("4*(2|6)");
std::string exprForCols("(4|6)2*");
int g_depthForAutoInitialization = 0;
float g_energyLossThreshold;
int g_247eModelRootRow = 4;
int g_247eModelRootCol = 4;
int g_useEModel = 1;
bool isStepByStepSimulatorInteractive = false; //Step by step or auto simulator ?
bool isStepByStepSimulatorFromFile = false; // If simulator is from file input
bool variableSourcesPower = false;
bool initializeFromFile = true;
const char *fileToInitializeFrom = "in.txt";
const char *resultsFileName = "results.txt";
int numStepsOnAutoSimulator = 0; // 1000;
int g_minPowerForWirelessSource = 10;
int g_maxPowerForWirelessSource = 1000;
int g_speedOnConduct = 10;
int g_maxFlowPerCell = 100000;
int g_ticksToDelayDataFlowCaptureOnRestructure = 10;
int g_simulationTicksForDataFlowEstimation = 100;
int g_minNodesOnRandomTree = 20;
int g_numSourcesOnRandomBoard = 2;
int g_numOptimalScenarioSimulations = 1;
int g_simulateOptimalVsRandomReconfigScenarios = 0;
int g_simulateOptimalVsRandomFlowScenarios = 0;
int g_simulateOptimalVsRandomFlowSampleCount = 0;
int g_avgTickBetweenSourceEvents = 0;
int g_maxResourcesToRent = 1;


bool g_verboseBestGatheredSolutions = true; // print the best gathered solutions
bool g_verboseLocalSolutions = true; // #define VERBOSE_LOCALSOLUTIONS	// print the local solutions from each cell
bool g_verboseElasticModel_All = true;
bool g_verboseElasticModel_Results = true;

bool g_elasticModelEnabled = false;
float g_costPerResource[256] = { INVALID_COST_PER_RESOURCE };
float g_benefitPerUnitOfFlow = 1.0f;

int g_powerChangeFrequency = 10;
float g_maxPowerVelocityPerTick = 50.0f;

std::ostream* g_debugLogOutput;

// Globals since we need to extern them
std::regex g_rowRXExpr;
std::regex g_colRXExpr;

bool processCostPerResource(const std::string& str)
{
	for (int i = 0; i < 256; i++)
		g_costPerResource[i] = INVALID_COST_PER_RESOURCE;

	std::stringstream ss(str);
	while (!ss.eof())
	{
		char symbol;
		float cost;
		ss >> symbol;
		if (ss.peek() != '-')
		{
			assert(false && "invalid format for cost strings- string");
			return false;
		}
		ss.ignore();
		ss >> cost;
		assert(cost >= 0 && " cost should be positive !!");

		g_costPerResource[symbol] = cost;
		if (!ss.eof())
		{
			if (ss.peek() != ',')
			{
				assert(false && " invalid format");
				return false;
			}
		}
		ss.ignore();
	}

	return true;
}

void readInput(const char* configFileName)
{
	// Step 1: read the file and set its content to a string stream reader
	ifstream configFile(configFileName);
	if (!configFile.is_open())
	{
		assert("Can't open config file or it's missing !");
	}
	
	configFile.seekg(0, std::ios::end);
	std::streampos length = configFile.tellg();
	configFile.seekg(0, std::ios::beg);

	std::vector<char> buffer((uint)length);
	configFile.read(&buffer[0], length);

	std::istringstream fileContent(buffer.data());
	//fileContent.rdbuf()->pubsetbuf(&buffer[0], length);

	// Step 2: read the options and populate them
	std::string newLine;
	std::map<std::string, std::string> keyValues;
	while(std::getline(fileContent, newLine))
	{
		std::istringstream lineReader(newLine);
		if (newLine.empty() || newLine[0] =='\r' || newLine[0] == '\n' || newLine[0] == '\t' || newLine[0] == '/') // Ignore empty lines
			continue;

		bool wellFormatted = false;
		std::string key, value;
		if (std::getline(lineReader, key, '='))
		{
			trimCommentsAndWhiteSpaces(key);
			if (std::getline(lineReader, value, ' '))
			{
				trimCommentsAndWhiteSpaces(value);
				keyValues.insert(std::make_pair(key, value));
				
				if (value.size() > 0 && key.size() > 0)
				{
					wellFormatted = true;
				}
			}
		}

		if (wellFormatted == false)
		{
			cout << "Incorrect formatted line !! " << newLine << endl;
			return;
		}

		newLine.clear();
	}

	for (auto& it : keyValues)
	{
		const std::string& key = it.first;
		const std::string& value = it.second;

		if (key == "isFixedSeed") { isFixedSeed = std::stoi(value) == 1 ? true : false; }
		else if (key == "FIXED_SEED") { FIXED_SEED = std::stoi(value); }
		else if (key == "exprForRows") { exprForRows = value; }
		else if (key == "exprForCols") { exprForCols = value; }
		else if (key == "depthForAutoInitialization") { g_depthForAutoInitialization = std::stoi(value); }
		else if (key == "isStepByStepSimulatorFromFile") { isStepByStepSimulatorFromFile = std::stoi(value) == 1 ? true : false; }
		else if (key == "isStepByStepSimulatorInteractive") { isStepByStepSimulatorInteractive = std::stoi(value) == 1 ? true : false; }
		else if (key == "variableSourcesPower") { variableSourcesPower = std::stoi(value) == 1 ? true : false; }
		else if (key == "initializeFromFile") { initializeFromFile = std::stoi(value) == 1 ? true : false; }
		else if (key == "fileToInitializeFrom") { fileToInitializeFrom = strdup(value.c_str()); }
		else if (key == "resultsFileName") { resultsFileName = strdup(value.c_str()); }
		else if (key == "numStepsOnAutoSimulator") { numStepsOnAutoSimulator  = std::stoi(value); }
		else if (key == "minPowerForWirelessSource") { g_minPowerForWirelessSource = std::stoi(value); }
		else if (key == "maxPowerForWirelessSource") { g_maxPowerForWirelessSource = std::stoi(value); }
		else if (key == "minNodesOnRandomTree") { g_minNodesOnRandomTree = std::stoi(value); }
		else if (key == "speedOnConduct") { g_speedOnConduct = std::stoi(value); }
		else if (key == "g_verboseLocalSolutions") { g_verboseLocalSolutions = std::stoi(value) == 1 ? true : false; }
		else if (key == "g_verboseBestGatheredSolutions") { g_verboseBestGatheredSolutions = std::stoi(value) == 1 ? true : false; }
		else if (key == "g_verboseElasticModel_All") { g_verboseElasticModel_All = std::stoi(value) == 1 ? true : false; }
		else if (key == "g_verboseElasticModel_Results") { g_verboseElasticModel_Results = std::stoi(value) == 1 ? true : false; }
		else if (key == "g_elasticModelEnabled") { g_elasticModelEnabled = std::stoi(value) == 1 ? true : false; }
		else if (key == "g_costPerResource") { processCostPerResource(value); }
		else if (key == "g_benefitPerUnitOfFlow") { g_benefitPerUnitOfFlow = std::stof(value); }		
		else if (key == "g_maxFlowPerCell") { g_maxFlowPerCell = std::stoi(value); }
		else if (key == "g_ticksToDelayDataFlowCaptureOnRestructure"){ g_ticksToDelayDataFlowCaptureOnRestructure = std::stoi(value); }
		else if (key == "g_simulationTicksForDataFlowEstimation")  { g_simulationTicksForDataFlowEstimation = std::stoi(value); }
		else if (key == "numSourcesOnRandomBoard") { g_numSourcesOnRandomBoard = std::stoi(value); }
		else if (key == "numOptimalScenarioSimulations") { g_numOptimalScenarioSimulations = std::stoi(value); }
		else if (key == "simulateOptimalVsRandomReconfigScenarios") { g_simulateOptimalVsRandomReconfigScenarios = std::stoi(value); }
		else if (key == "simulateOptimalVsRandomFlowScenarios") { g_simulateOptimalVsRandomFlowScenarios = std::stoi(value); }
		else if (key == "g_simulateOptimalVsRandomFlowSampleCount") { g_simulateOptimalVsRandomFlowSampleCount = std::stoi(value); }
		else if (key == "g_avgTickBetweenSourceEvents") { g_avgTickBetweenSourceEvents = std::stoi(value); }
		else if (key == "powerChangeFrequency") { g_powerChangeFrequency = std::stoi(value); }
		else if (key == "maxPowerVelocityPerTick") { g_maxPowerVelocityPerTick = (float)std::stoi(value); }
		else if (key == "g_maxResourcesToRent") { g_maxResourcesToRent = std::stoi(value); }
		else if (key == "useEModel") { g_useEModel = std::stoi(value); }
		else if (key == "g_energyLossThreshold") { g_energyLossThreshold = std::stof(value); }
		else
		{
			ostringstream strErr;
			strErr << "Unrecognized key " << key << " can you please verify your input again ? " << endl;
			assert(false && strErr.str().c_str());
		}
	}
};

void doUnitTests(Simulator& simulator)
{
	//simulator.reorganize();
	//simulator.printBoard(cout);
	
	//simulator.doUnitTests();
	simulator.doDataFlowSimulation_serial(false, cout);
}

int main()
{
	readInput("config.ini");

	if (isFixedSeed == false)
	{
		std::srand((uint)std::time(0));
	}
	else
	{
		std::srand(FIXED_SEED);
	}
	// TODO: move these as input for program
	Simulator simulator(exprForRows, exprForCols, g_speedOnConduct);
	g_colRXExpr = regex(exprForCols);
	g_rowRXExpr = regex(exprForRows);

	g_debugLogOutput = &std::cout;
	if (g_simulateOptimalVsRandomFlowScenarios)
	{
		
		simulator.simulateOptimalVsRandomFlowScenario(fileToInitializeFrom, g_numOptimalScenarioSimulations, g_simulateOptimalVsRandomFlowSampleCount, g_simulationTicksForDataFlowEstimation,
													  g_avgTickBetweenSourceEvents, g_ticksToDelayDataFlowCaptureOnRestructure, *g_debugLogOutput);
	}
	else if (g_simulateOptimalVsRandomReconfigScenarios)
	{
		simulator.simulateOptimalReconfigurationScenarios(g_numOptimalScenarioSimulations, resultsFileName);
	}
	else
	{
		if (initializeFromFile)
		{
			simulator.initialize_fromFile(fileToInitializeFrom);
		}
		else
		{
			simulator.initialize_random(g_depthForAutoInitialization);
		}

		simulator.checkBoardLanguageConstraints();

		//cout << "Initial board: " << endl;

		//doUnitTests(simulator);
		//return true;

		if (isStepByStepSimulatorFromFile)
		{
			std::ifstream inFile("simulator.txt");
			std::ofstream outFile("results.txt");
			g_debugLogOutput = &outFile;
			//outFile << "Initial board: " << std::endl;
			simulator.printBoard(outFile);
			simulator.doStepByStepSimulation(false, inFile, outFile);
		}
		else if (isStepByStepSimulatorInteractive)
		{
			std::ofstream outFile("results.txt");
			g_debugLogOutput = &outFile;
			simulator.printBoard(cout);
			//cout << "Initial board: " << std::endl;
			simulator.doStepByStepSimulation(true, cin, cout);
		}
		else
		{
			//cout << "Initial board: " << std::endl;
			simulator.printBoard(cout);
			simulator.autoSimulate(numStepsOnAutoSimulator, g_minPowerForWirelessSource, g_maxPowerForWirelessSource, resultsFileName);
		}
	}

	return 0;
}
