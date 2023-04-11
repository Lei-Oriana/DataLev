#include <S2M2/cpp/PathPlanner.h>
#include <Detector/BeadDetector.h>
#include <AcousticSimulator/Helper/microTimer.h>
#include <AsierInho_V2.h>
#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverIBP.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverV3.h>
#include <GSPAT_SolverV4.h>
#include <deque>
#include <cmath>
#include <conio.h>

using namespace GSPAT_Naive;
AsierInho_V2::AsierInhoBoard_V2* a;		// this version (V2) of AsierInho has the function to send 32 geometries at the same time
float boardHeight = 0.12f;				// distance between the top and bottom boards
bool running = true;					// indicates the threads are still running
float targetUPS = 10240;				// target updates per second
int numGeometries = 32;					// number of geometries to be sent at the same time
pthread_mutex_t mutex_solution_queue;
pthread_mutex_t solution_available;
std::deque<GSPAT::Solution*> solutions;
pthread_t readerThread, writerThread;

void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false);
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, int& cBufferSize, float*& posBuffer, float*& ampBuffer, unsigned char*& colBuffer);
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);
std::vector<glm::vec3> getFilteredPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset, float zPosOthers);

std::vector<glm::vec3> getFilteredAndSortedPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset);


std::vector<glm::vec3> getHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName);

std::vector<glm::vec3> updateHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName);

std::vector<glm::vec3> getEqualHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float height);

std::vector<glm::vec3> readMultiPos(std::string fileName, int numPos);


void print(const char* str);

void* solutionReaderThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	while (running) {
		//0. Check the Queue:
		pthread_mutex_lock(&mutex_solution_queue);
		while (solutions.size() == 0) {
			//1. There is nothing to be done... wait until some solution is produced.
			pthread_mutex_unlock(&mutex_solution_queue);
			pthread_mutex_lock(&solution_available);
			pthread_mutex_lock(&mutex_solution_queue);
		}
		//1. We got access! 
		{
			//2. Let's get the first solution in the queue (and unlock, so others can keep adding/removing)
			GSPAT::Solution* curSolution = solutions[0];
			solutions.pop_front();
			pthread_mutex_unlock(&mutex_solution_queue);
			// read the final phases and discretise. 
			unsigned char* finalMessages = NULL;
			curSolution->finalMessages(&finalMessages);
			bool transducersOff = curSolution->transducersOff;
			updateGeometries(finalMessages, numGeometries, transducersOff);
			solver->releaseSolution(curSolution);
		}
	}
	return NULL;
}
void* solutionWriterThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	// 1. Initialize the bead detector
	static bool firstTime = true;
	float stageHeight = 0.038f; // the height of the stage (work great to set a little bit above the surface)
	float p1[] = { -0.084f, -0.084f,  stageHeight },
		p2[] = { 0.084f, -0.084f, stageHeight },
		p3[] = { 0.084f,  0.084f, stageHeight },
		p4[] = { -0.084f,  0.084f, stageHeight };
	BeadDetector& detector = BeadDetector::instance(p1, p2, p3, p4);   // corners click order: topleft, bottomleft, bottomright, topright
	detector.startDetection();

	// 2. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06, -0.06, -0.06);
	glm::vec3 boundaryMax(+0.06, +0.06, +0.06);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.007, 0.007, 0.007);	// size of the agents in meter
	float velocity = 0.02f;					// maximum velocity in meter/second
	// define start and targets positions
	int numPoints = 9;   // the number of traps/particles
	int row = 3, column = 3;
	std::vector<glm::vec3> initPositions;  // store the grid points   row-based
	float r1 = 0.03f, r2 = 0.02f;
	float startYpoint = 0.04f, startXpoint = 0.04f, distance = 0.04f, initHeight = 0.062f;
	for (int i = 0; i < row; i++)
	{
		float x = startXpoint - distance * i;   //row
		for (int j = 0; j < column; j++)
		{
			float y = startYpoint - distance * j;   //column
			initPositions.push_back(glm::vec3(x, y, initHeight - boardHeight));
		}
	}
	// 3. Fill buffers: we set all the positions at (0, 0, 0.12), the centre of the working volume
	//int numPoints = paths.size();
	int pBufferSize, aBufferSize, cBufferSize;
	float* posBuffer, * ampBuffer;
	unsigned char* colBuffer;
	fillBuffers(numPoints, pBufferSize, aBufferSize, cBufferSize, posBuffer, ampBuffer, colBuffer);

	// 4. Define matrices to update the positions of the traps
	float initMat[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	float mStarts[16 * 16], mEnds[16 * 16];
	static const size_t X_index = 3, Y_index = 7, Z_index = 11;
	for (int p = 0; p < numPoints; p++) {
		memcpy(&(mStarts[16 * p]), initMat, 16 * sizeof(float));
		memcpy(&(mEnds[16 * p]), initMat, 16 * sizeof(float));

	}

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	float currTime = 0;								// current (virtual) time 
	bool moving = false;							// indicates the particles are moving or not
	bool transducersOff = true;						// transducers can be off before detecting the particle positions
	float initTime = fabs(initHeight - stageHeight) / velocity;

	std::vector<glm::vec3> currPositions = initPositions;
	std::vector<glm::vec3> nextPositions = initPositions;
	std::vector<AgentPath> currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, true);

	while (running) {
		std::vector<glm::vec3>  detectedPositions, halfPositions(numPoints); // halfPositions are the Initial trapping positions (regarding to levitator's coordinate)
		std::vector<AgentPath> plannedPaths;
		// 5.1. React to user's input
		if (_kbhit()) {
			switch (_getch()) {
			case ' ': // finish the writing thread
				printf("SPACE BAR pressed");
				running = false;
				break;
			case '0': // detect the particle positions 
				velocity = 0.02f;
				planner.setBoundary(1.62f * boundaryMin, 1.62f * boundaryMax); // the working volume needs to be expanded from the default (0.06 x 0.06 x 0.06) as the stage height is out of the default volume
				detectedPositions = getDetectedPositions(detector, numPoints); // detect the particle positions

				for (int p = 0; p < numPoints; p++)
					halfPositions[p] = glm::vec3(detectedPositions[p].x, detectedPositions[p].y, initHeight - boardHeight);

				nextPositions = getEqualHeightPositions(numPoints, initPositions, 0.0f);

				plannedPaths = setPaths(planner, halfPositions, nextPositions, size, velocity, true);
				
				if (plannedPaths.size() == 0) {
					printf("Maybe the particles are too close each other...\n"); break;
				}
				moving = true;
				currTime = 0;

				for (int p = 0; p < numPoints; p++) {
					currPaths[p].clearWaypoints();
					currPaths[p].addWayPoint(0, detectedPositions[p].x, detectedPositions[p].y, detectedPositions[p].z);
					currPaths[p].addWayPoint(initTime, halfPositions[p].x, halfPositions[p].y, halfPositions[p].z);
					currPaths[p].addAgentPath(plannedPaths[p]);
				}

				// update the matrices
				for (int p = 0; p < numPoints; p++) {
					glm::vec3 pos = currPaths[p].getPosition(0);
					mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
					mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
					mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
				}
				planner.setBoundary(boundaryMin, boundaryMax); // make the boundary back to the default
				transducersOff = false; // turn the transducers on
				break;
			case '9': // turn the transducers off
				transducersOff = true;
				break;
			case '1': // move to the initial points of dataset
				moving = true;
				currTime = 0;
				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				nextPositions = getHeightPositions(numPoints, initPositions, 0, 0, "AcousticSimulator/PhysicalizationDemos/4CountriesDataLev3by3.csv");
				currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, false);
				break;
			case '2': //filter the dataset
				planner.setBoundary(1.57f * boundaryMin, 1.57f * boundaryMax); // the working volume needs to be expanded from the default (0.06 x 0.06 x 0.06) as the stage height is out of the default volume

				moving = true;
				currTime = 0;
				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				nextPositions = getFilteredPositions(numPoints, currPositions, row, column, 1, 0.025f, 0.02f,-0.02f);
				currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, false);
				break;
			case '3': //sort the dataset
				moving = true;
				currTime = 0;
				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				nextPositions = getFilteredAndSortedPositions(numPoints, nextPositions, row, column, 1, 0, 0);
				currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, false);
				break;
			case '4': //update datasets
				moving = true;
				currTime = 0;
				//paths = (going ? goPaths : returnPaths);
				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				nextPositions = updateHeightPositions(numPoints, currPositions, 0, 0, "AcousticSimulator/PhysicalizationDemos/4CountriesDataLevNew3by3.csv");
				currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, false);
				break;
			case '8': //go to 0 height
				planner.setBoundary(1.62f * boundaryMin, 1.62f * boundaryMax); // the working volume needs to be expanded from the default (0.06 x 0.06 x 0.06) as the stage height is out of the default volume
				moving = true;
				currTime = 0;
				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				nextPositions = getEqualHeightPositions(numPoints, initPositions, 0.0f);
				currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, true);
				break;
			}
		}

		// 5.2. Update matrices
		bool finished = true;
		for (int p = 0; p < numPoints; p++) {
			glm::vec3 pos = currPaths[p].getPosition(currTime); // get the positions of the agents according to the current time

			mStarts[16 * p + X_index] = mEnds[16 * p + X_index];
			mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index];
			mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index];
			mEnds[16 * p + X_index] = pos.x;
			mEnds[16 * p + Y_index] = pos.y;
			mEnds[16 * p + Z_index] = pos.z;
			finished &= (currPaths[p].getFinalTime() < currTime); // check if every agent has finished its path
		}
		if (moving) currTime += dt; // increment the current time
		// 5.3. Compute a hologram
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[0]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		solver->compute(solution);
		solution->transducersOff = transducersOff;
		// 5.4. Pass the solution to the reader thread
		pthread_mutex_lock(&mutex_solution_queue);
		solutions.push_back(solution);
		pthread_mutex_unlock(&solution_available);
		pthread_mutex_unlock(&mutex_solution_queue);
		// 5.5. Update the status
		if (finished) {
			moving = false;
			finished = false;
		}
	}

	delete[] posBuffer, ampBuffer, colBuffer;
	return NULL;
}

void main() {
	//STAGE 0: Initialize
	float transducerPositions[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	RegisterPrintFuncs(print, print, print);
	GSPAT::Solver* solver = createSolver(512);
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	a = AsierInho_V2::createAsierInho();
	a->connect(2, 4);  
	a->readParameters(transducerPositions, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);
	//STAGE 1: Create the reader and writer threads and the concurrency control variables:
	pthread_mutex_init(&solution_available, NULL);
	pthread_mutex_lock(&solution_available);
	pthread_mutex_init(&mutex_solution_queue, NULL);
	pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
	pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));
	//STAGE 2: Simply waiting the other threads finish
	while (running) { ; }
	Sleep(2000);
	a->disconnect();
}

void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff) {
	static float dynamicTargetUPS = targetUPS;
	static float timePerUpdateInMilis = (float)(numGeometries) * 1000.0f / dynamicTargetUPS; //this needs to change every time we change dynamicTargetUPS
	static int numSolutions = dynamicTargetUPS;
	static DWORD prevTime, curTime;
	static DWORD secPrevTime = microTimer::uGetTime(), secCurTime;
	static float timeSinceLastUpdate;

	static bool firstTime = true;
	if (firstTime) {
		prevTime = microTimer::uGetTime();
		firstTime = false;
	}
	//Compute discrete phases for this update:
	if (transducersOff) a->turnTransducersOff();
	else a->updateMessages(messages, numGeometries);

	//let's check the current time and wait untill next update is due
	do {
		curTime = microTimer::uGetTime();
		timeSinceLastUpdate = (curTime - prevTime) / 1000.f;
	} while (timeSinceLastUpdate < timePerUpdateInMilis && timeSinceLastUpdate > 0);
	prevTime = curTime;
	//Send phases and update time

	//Plot performance (should be 1s)
	numSolutions -= numGeometries;
	if (numSolutions <= 0) {
		dynamicTargetUPS += (int)(0.1f * targetUPS);
		if (dynamicTargetUPS > targetUPS)
			dynamicTargetUPS = targetUPS;
		timePerUpdateInMilis = (float)(numGeometries) * 1000.0f / dynamicTargetUPS;
		numSolutions = dynamicTargetUPS;
		secCurTime = microTimer::uGetTime();
		printf("Time Per computation = %f; Last Update: %f, UPS: %f\n", (secCurTime - secPrevTime) / 1000000.f, timeSinceLastUpdate / (float)(numGeometries), dynamicTargetUPS);
		secPrevTime = secCurTime;
	}
}
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment) {
	int numAgents = starts.size();
	std::vector<PathPlanner::AgentInfo> agents;
	for (int i = 0; i < numAgents; i++) {
		PathPlanner::AgentInfo agent(starts[i], targets[i], size, velocity);
		agents.push_back(agent);
	}
	planner.setAgents(agents);

	std::string fileName = "../S2M2/Files/test.csv";
	std::string newFileName = "../S2M2/Files/newTest.csv";
	std::string outFileName = "../S2M2/Files/output.csv";
	planner.saveAgentInfo(fileName);

	if (taskAssignment) {
		planner.taskAssignment(fileName, newFileName);
		planner.runS2M2(newFileName, outFileName);
	}
	else {
		planner.runS2M2(fileName, outFileName);
	}

	return PathPlanner::readAgentPaths(outFileName);
}
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, int& cBufferSize, float*& posBuffer, float*& ampBuffer, unsigned char*& colBuffer) {
	int sampleSize = numGeometries;
	pBufferSize = numPoints * sampleSize * 4;
	aBufferSize = numPoints * sampleSize * 1;
	cBufferSize = sampleSize * 3;
	posBuffer = new float[pBufferSize];
	ampBuffer = new float[aBufferSize];
	colBuffer = new unsigned char[cBufferSize];

	// Define actual content we create
	int posInd = 0, ampInd = 0, colInd = 0;
	for (int s = 0; s < sampleSize; s++) {
		for (int t = 0; t < numPoints; t++) {
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = boardHeight;
			posBuffer[posInd++] = 1;
			ampBuffer[ampInd++] = 16000;
		}
		colBuffer[colInd++] = 0;
		colBuffer[colInd++] = 0;
		colBuffer[colInd++] = 0;
	}
}
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints) {
	static const int minThres = 10, maxThres = 160, thresStep = 10;
	int threshold = minThres;
	bool corrected = false;

	std::vector<glm::vec3> detectedPositions(numPoints);
	while (!corrected) {
		detector.setThreshold(threshold);
		std::vector<cv::Point3d> currentBeads = detector.getCurrentBeads();
		if (currentBeads.size() == numPoints) {
			corrected = true;
			for (int p = 0; p < numPoints; p++)
				detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z - boardHeight); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
		}
		if (threshold == maxThres) threshold = minThres;
		else threshold += thresStep;
	}
	return detectedPositions;
}

std::vector<glm::vec3> getFilteredPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset, float zPosOthers)
{
	std::vector<glm::vec3> positions;
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{

			if (j == filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x - xOffset, prePositions[i * column + j].y, prePositions[i * column + j].z));
				std::cout << "not change index  " << i * column + j << " x: " << prePositions[i * column + j].x - xOffset << " y:" << prePositions[i * column + j].y << " z: " << prePositions[i * column + j].z << std::endl;
			}
			if (j < filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x + xOffset + 0.01f * i, prePositions[i * column + j].y + yOffset, zPosOthers));
				std::cout << "change left index" << i * column + j << " x: " << prePositions[i * column + j].x + xOffset + 0.01f * i << " y:" << prePositions[i * column + j].y << " z: " << zPosOthers << std::endl;
			}
			if (j > filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x + xOffset + 0.01f * i, prePositions[i * column + j].y - yOffset, zPosOthers));
				std::cout << "change right index" << i * column + j << " x: " << prePositions[i * column + j].x + xOffset + 0.01f * i << " y:" << prePositions[i * column + j].y << " z: " << zPosOthers << std::endl;
			}
		}
	}
	return positions;
}

std::vector<glm::vec3> getFilteredAndSortedPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset)
{
	// get the filtered positions and then change their order based on the heights, assign them to new positions
	std::vector<glm::vec3> filteredPositions;  //get the specific column positions
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (j == filteredColume)
			{
				filteredPositions.push_back(glm::vec3(prePositions[i * column + j].x, prePositions[i * column + j].y, prePositions[i * column + j].z));
				std::cout << "not change index" << i * column + j << std::endl;
			}
		}
	}
	std::vector<std::pair<float,int>> filteredHeight;

	//save the order
	std::vector<int> sortedOrder;   

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		filteredHeight.push_back(std::make_pair(filteredPositions[i].z,i));
	}
	std::sort(filteredHeight.begin(), filteredHeight.end());

	std::cout << "Sorted, ascending order \n";
	for (int i = 0; i < filteredPositions.size(); i++)
	{
		std::cout <<"height:"<< filteredHeight[i].first<<" " <<"oringinal index: "<< filteredHeight[i].second<<std::endl;   // outPout the sorted height and oringianl index
	}

	std::vector<std::pair<int, int>> IndexOrderPair;   // with correct index
	int order[] = {0,0,0,0,0};

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		IndexOrderPair.push_back(std::make_pair(filteredHeight[i].second, i));
		std::cout <<  "oringinal index:" << filteredHeight[i].second <<" order : "<< i << std::endl;
	}
	//save the order
	for (int i = 0; i < filteredPositions.size(); i++)
	{
		//[IndexOrderPair[i].first  get oringinal index
		order[filteredHeight[i].second] = i;		// get the order is important
	}

	std::vector<glm::vec3> sortedPositions;  // sort the filtered column

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		std::cout << "order : " << order[i] << std::endl;
		sortedPositions.push_back(glm::vec3(-0.04f, -0.04f + order[i]* 0.04f, filteredPositions[i].z));
	}
	std::vector<glm::vec3> returnedPositions;   //final returned positions 

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (j == filteredColume)
			{
				returnedPositions.push_back(sortedPositions[i]);
				std::cout << "re-new positions: " << sortedPositions[i].x << ", " << sortedPositions[i].y << ", " << sortedPositions[i].z << std::endl;
			}
			else
				returnedPositions.push_back(prePositions[i * column + j]);
		}
	}

	return returnedPositions;
}

std::vector<glm::vec3> updateHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName)
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> readPos = readMultiPos(fileName,numPoints);

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = readPos[i].z;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "updated positions:" << i << " x£º" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}

std::vector<glm::vec3> getHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName)
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> readPos = readMultiPos(fileName, numPoints);

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = readPos[i].z;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "get positions:" << i << " x£º" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}


std::vector<glm::vec3> getEqualHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float height)
{
	std::vector<glm::vec3> positions;

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = height;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "get equal height positions:" << i << " x£º" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}

std::vector<glm::vec3> readMultiPos(std::string fileName, int numPos)
{
	std::vector<glm::vec3> positions;
	int lineNum = 0;
	// 1. Read a file and store it
	std::ifstream ifs(fileName);
	if (ifs.is_open())
	{
		std::string fileLine;
		getline(ifs, fileLine);     // skip the first line
		while (getline(ifs, fileLine)) // parse every line
		{
			std::stringstream toParse(fileLine); //stringstream of the line

			std::vector<std::string> lineTokens;  // store every token in the line
			std::string token;
			while (std::getline(toParse >> std::ws, token, ','))  // input, output, delimeter,  >> std::ws: means extract white space
			{
				lineTokens.push_back(token);
				//std::cout << "token: " << token << std::endl;
			}
			if (!lineTokens.empty())
			{
				float x = 0;  // convert string type to number type
				float y = 0;  // convert string type to number type
				float z = atof(lineTokens[0].c_str());  // convert string type to number type
				//std::cout << "float x: " << x <<"float y: "<< y <<std::endl;
				positions.push_back(glm::vec3(x,y,z));

			}
			lineNum += 1;
		}
		std::cout << numPos << std::endl;
		std::cout << lineNum << std::endl;
		if ((lineNum) == numPos)
			std::cout << "good match for num of claimed and generated  " << std::endl;
		else
			std::cout << "no match for num of claimed and generated  " << std::endl;
	}
	else {
		std::cout << "ERROR: File " << fileName << "cannot be opened!" << std::endl;
	}
	// 2. print what we have read
	for (int i = 0; i < lineNum; i++)
	{
		//printf("line: %i ", i);

		//printf(" pos %i : (%f, %f)", i, positions[i].x, positions[i].y);
		//std::cout << "pos " << i << ": "<<positions[i].x <<","<<positions[i].y<< std::endl;

	}
	return positions;
}


void print(const char* str) {
	printf("%s\n", str);
}
