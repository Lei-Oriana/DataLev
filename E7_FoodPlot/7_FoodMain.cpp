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
#include <conio.h>
#include <random>

using namespace GSPAT_NAIVE;		
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
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3>& starts, std::vector<glm::vec3>& targets, glm::vec3 size, float velocity, bool taskAssignment = false);
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, int& cBufferSize, float*& posBuffer, float*& ampBuffer, unsigned char*& colBuffer);
void fillBuffers2(int numPoints, int& pBufferSize, int& aBufferSize, int& cBufferSize, float*& posBuffer, float*& ampBuffer, unsigned char*& colBuffer);

std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);
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

	float stageHeight = 0.038f; // the height of the stage (work great to set a little bit above the surface)
	float p1[] = { -0.085f, -0.085f,  stageHeight },
		p2[] = { 0.085f, -0.085f, stageHeight },
		p3[] = { 0.085f,  0.085f, stageHeight },
		p4[] = { -0.085f,  0.085f, stageHeight };
	BeadDetector& detector = BeadDetector::instance(p1, p2, p3, p4);
	detector.startDetection();


	// 2. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06, -0.06, -0.06);
	glm::vec3 boundaryMax(+0.06, +0.06, +0.06);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.007, 0.007, 0.007);	// size of the agents in meter
	float velocity = 0.02f;					// maximum velocity in meter/second
	// define test positions
	int numPoints = 5;
	float r = 0.03f;
	std::vector<glm::vec3> initPositions, targetPositions; 
	for (int i = 0; i < numPoints; i++) initPositions.push_back(glm::vec3(r * cos(i * 2.f * M_PI / (float)numPoints), r * sin(i * 2.f * M_PI / (float)numPoints), -0.05f));
	r = 0.03f;
	// representing the food: 1) mushroom 2)coffeebean 3)chip 4)bread 5) lettuce   (anti-clock wise)
	targetPositions.push_back(glm::vec3(-0.017f, -0.002f, -0.044f)); targetPositions.push_back(glm::vec3(-0.04f, 0.005f, -0.006f));  targetPositions.push_back(glm::vec3(-0.052f, 0.012f, 0.004f)); targetPositions.push_back(glm::vec3(-0.04f, -0.021f, -0.04f)); targetPositions.push_back(glm::vec3(-0.02f, -0.018f, -0.052f));
	
	// 3. Fill buffers: we set all the positions at (0, 0, 0.12), the centre of the working volume
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

	// These are for storing the data
	std::vector<float> times;
	std::vector<std::vector<glm::vec3>> trapPositions(numPoints); // where you created traps
	std::vector<std::vector<glm::vec3>> realPositions(numPoints); // where optiTrack detected

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	float currTime = 0;								// current (virtual) time 
	bool transitioning = false;						// indicates the particles are transitioning to the next positions or not
	bool transducersOff = true;					// transducers can be off before detecting the particle positions
	float initHeight = 0.062f;
	float initTime = fabs(initHeight - stageHeight) / velocity;
	std::vector<glm::vec3> currPositions = initPositions;
	std::vector<glm::vec3> nextPositions = initPositions;
	std::vector<AgentPath> currPaths = setPaths(planner, currPositions, nextPositions, size, velocity, false);

	int pIndex = 0, aIndex = 0;
	while (running) {
		// 5.1. React to user's input
		std::vector<glm::vec3> detectedPositions, halfPositions(numPoints); // halfPositions are the Initial trapping positions (regarding to levitator's coordinate)
		std::vector<AgentPath> plannedPaths;

		if (_kbhit()) {
			switch (_getch()) {
			case ' ': // finish the writing thread
				printf("SPACE BAR pressed");
				running = false;
				break;
			case '0': // detect the particle positions 
				planner.setBoundary(1.47f * boundaryMin, 1.47f * boundaryMax); // the working volume needs to be expanded from the default (0.06 x 0.06 x 0.06) as the stage height is out of the default volume

				detectedPositions = getDetectedPositions(detector, numPoints); // detect the particle positions by OptiTrack
				//detectedPositions = initPositions;
				for (int p = 0; p < numPoints; p++)
					halfPositions[p] = glm::vec3(detectedPositions[p].x, detectedPositions[p].y, initHeight - boardHeight);
				// compute the paths
				plannedPaths = setPaths(planner, halfPositions, nextPositions, size, velocity, true);
				if (plannedPaths.size() == 0) {
					printf("Maybe the particles are too close each other...\n"); break;
				}
				transitioning = true;
				currTime = 0;
				for (int p = 0; p < numPoints; p++) {
					currPaths[p].clearWaypoints();
					currPaths[p].addWayPoint(0, detectedPositions[p].x, detectedPositions[p].y, detectedPositions[p].z);
					currPaths[p].addWayPoint(initTime, halfPositions[p].x, halfPositions[p].y, halfPositions[p].z);
					currPaths[p].addAgentPath(plannedPaths[p]);
				}

				// update the matrices																														  // update the matrices
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
			case '1': // move the particles from starts to targets or vice verca
				transitioning = true;
				currTime = 0;

				for (int p = 0; p < numPoints; p++) {
					currPositions[p] = nextPositions[p];	// simply copy
				}
				static bool going = true;
				nextPositions = (going ? targetPositions : initPositions); 
				going = !going;
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

		if (transitioning) currTime += dt; // increment the current time

		// 5.3. Compute a hologram
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[pIndex]), &(ampBuffer[aIndex]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		solver->compute(solution);
		solution->transducersOff = transducersOff;
		// 5.4. Pass the solution to the reader thread
		pthread_mutex_lock(&mutex_solution_queue);
		solutions.push_back(solution);
		pthread_mutex_unlock(&solution_available);
		pthread_mutex_unlock(&mutex_solution_queue);
		// 5.5. Update the status

		if (finished) {
			transitioning = false;
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
	int bottomID = 2, topID = 4;
	a->connect(bottomID, topID);
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
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3>& starts, std::vector<glm::vec3>& targets, glm::vec3 size, float velocity, bool taskAssignment) {
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

	bool success = true;
	if (taskAssignment) {
		success &= planner.taskAssignment(fileName, newFileName);
		if (success) {
			for (int a = 0; a < planner.getNumAgents(); a++)
				targets[a] = planner.getTargetPosition(a);
		}
		success &= planner.runS2M2(newFileName, outFileName);
	}
	else {
		success &= planner.runS2M2(fileName, outFileName);
	}
	if (!success) {
		printf("No plan is found...\n");
		return std::vector<AgentPath>(0);
	}
	else
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
void fillBuffers2(int numPoints, int& pBufferSize, int& aBufferSize, int& cBufferSize, float*& posBuffer, float*& ampBuffer, unsigned char*& colBuffer) {
	int sampleSize = numGeometries * 32;
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
			//posBuffer[posInd++] = 0;
			if (t == 0) posBuffer[posInd++] = +0.02f;
			if (t == 1) posBuffer[posInd++] = -0.02f;
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = boardHeight + 0.01f * sin(s * 2.f * M_PI / sampleSize);
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
			for (int p = 0; p < numPoints; p++) {
				detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z - boardHeight); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
				printf("Detected %d: pos = (%f, %f, %f)\n", p, detectedPositions[p].x, detectedPositions[p].y, detectedPositions[p].z);
			}
		}
		if (threshold == maxThres) threshold = minThres;
		else threshold += thresStep;
	}
	return detectedPositions;
}
void print(const char* str) {
	printf("%s\n", str);
}
