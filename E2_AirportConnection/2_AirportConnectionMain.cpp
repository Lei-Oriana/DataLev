#include <AcousticSimulator/Helper/microTimer.h>
#include <AsierInho_V2.h>
#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverIBP.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverV3.h>
#include <GSPAT_SolverV4.h>

#include <OpenGLFramework/OpenGLFramework.h>
#include <OpenGLFramework/3DUI_Utils/3DUI_Utils.h>
#include <OpenGLFrameworkExtensions/FPS_Controller/FPS_Controller.h>
#include <OpenGLFrameworkExtensions/ProjectionMapping/MappingApplication.h>

#include <S2M2/cpp/PathPlanner.h>
#include <Detector/BeadDetector.h>

#include <deque>
#include <conio.h>

using namespace GSPAT_V2;				// this version (V4) of GS-PAT has induction (using the previous solutions as the initial phases)
AsierInho_V2::AsierInhoBoard_V2* a;		// this version (V2) of AsierInho has the function to send 32 geometries at the same time
float boardHeight = 0.12f;				// distance between the top and bottom boards
bool running = true;					// indicates the threads are still running
float targetUPS = 10240;				// target updates per second
const int numGeometries = 32;					// number of geometries to be sent at the same time
pthread_mutex_t mutex_solution_queue;
pthread_mutex_t solution_available;
std::deque<GSPAT::Solution*> solutions;
pthread_t readerThread, writerThread;
int numPoints = 8;

using namespace OpenGLFramework;
RenderBufferObject* def_fbo;
glm::mat4 projectorV, projectorP;
MappingApplication* projectApplication = new MappingApplication();
pthread_mutex_t new_position_available;
std::vector<glm::vec3> newPositions(numPoints);

void createWorldScene();				//Create the contents and place them where appropriate
void renderWorldScene();				// Render our world
void updateWorldScene(float curTime);	// Update the world according to: user events (inputs), time, etc...

bool turnTransducersOff = false;						// transducers can be off before detecting the particle positions
bool pickingup = false;
bool startMoving = false;
float currentAmplitude = 8000;
bool amplitudeChanged = false;
pthread_mutex_t new_amplitude_available;
std::vector<GLuint> colourTextures;
int numAnimations = 4;
int currentTexture = 0;
class KeyControl : public IKeyboardListener {
public:
	KeyControl() { ; }
	void keyPressed(int key) { ; }
	void keyReleased(int key) {
		if (key == GLFW_KEY_0) { 
			pickingup = true; 
		}
		if (key == GLFW_KEY_9) { 
			turnTransducersOff = true;
		}
		if (key == GLFW_KEY_1) { 
			startMoving = true; 
		}
		if (key == GLFW_KEY_UP) { amplitudeChanged = true; pthread_mutex_lock(&new_amplitude_available); currentAmplitude += 1000.f; pthread_mutex_unlock(&new_amplitude_available); }
		if (key == GLFW_KEY_DOWN) { amplitudeChanged = true; pthread_mutex_lock(&new_amplitude_available); currentAmplitude -= 1000.f; if (currentAmplitude < 0) currentAmplitude = 0; pthread_mutex_unlock(&new_amplitude_available); }

		if (key == GLFW_KEY_LEFT) { currentTexture = (currentTexture - 1 + numAnimations) % numAnimations; }
		if (key == GLFW_KEY_RIGHT) { currentTexture = (currentTexture + 1) % numAnimations; }
	}
};
void loadPandV(std::string fileName);


void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner &planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false);
void fillBuffers(int numPoints, int &pBufferSize, int &aBufferSize, int &cBufferSize, float *&posBuffer, float *&ampBuffer, unsigned char *&colBuffer);
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
			updateGeometries(finalMessages, numGeometries, curSolution->transducersOff);
			solver->releaseSolution(curSolution);
		}
	}
	return NULL;
}
void* solutionWriterThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	// 1. Initialize the bead detector
	float stageHeight = 0.06f + 0.002; // the height of the stage (work great to set a little bit above the surface)
	float p1[] = { -0.084f, -0.084f,  stageHeight },
		p2[] = { 0.084f, -0.084f, stageHeight },
		p3[] = { 0.084f,  0.084f, stageHeight },
		p4[] = { -0.084f,  0.084f, stageHeight };
	BeadDetector& detector = BeadDetector::instance(p1, p2, p3, p4);
	detector.startDetection();

	// 2. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06, -0.06, -0.001);
	glm::vec3 boundaryMax(+0.06, +0.06, +0.06);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.008, 0.008, 0.014);	// size of the agents in meter
	float velocity = 0.02f;					// maximum velocity in meter/second
	// define start and targets positions
	std::vector<glm::vec3> cities;
	cities.push_back(glm::vec3(+0.022, -0.015, 0));
	cities.push_back(glm::vec3(+0.002, -0.028, 0));
	cities.push_back(glm::vec3(+0.014, +0.031, 0));
	cities.push_back(glm::vec3(-0.018, -0.052, 0));
	cities.push_back(glm::vec3(+0.011, -0.048, 0));
	cities.push_back(glm::vec3(-0.017, -0.006, 0));
	cities.push_back(glm::vec3(+0.019, +0.004, 0));
	cities.push_back(glm::vec3(-0.013, +0.028, 0));

	std::vector<glm::vec3> distinations;
	distinations.push_back(cities[7]);
	distinations.push_back(cities[2]);
	distinations.push_back(cities[6]);
	distinations.push_back(cities[1]);
	distinations.push_back(cities[0]);
	distinations.push_back(cities[3]);
	distinations.push_back(cities[4]);
	distinations.push_back(cities[5]);

	std::vector<AgentPath> goPaths = setPaths(planner, cities, distinations, size, velocity, false);
	std::vector<AgentPath> returnPaths = setPaths(planner, distinations, cities, size, velocity, false);

	std::vector<AgentPath> paths = goPaths;

	// 3. Fill buffers: we set all the positions at (0, 0, 0.12), the centre of the working volume
	numPoints = paths.size();
	int pBufferSize, aBufferSize, cBufferSize;
	float *posBuffer, *ampBuffer;
	unsigned char *colBuffer;
	fillBuffers(numPoints, pBufferSize, aBufferSize, cBufferSize, posBuffer, ampBuffer, colBuffer);

	// 4. Define matrices to update the positions of the traps
	float initMat[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	float mStarts[16 * 16], mEnds[16 * 16];
	static const size_t X_index = 3, Y_index = 7, Z_index = 11;
	for (int p = 0; p < numPoints; p++) {
		memcpy(&(mStarts[16 * p]), initMat, 16 * sizeof(float));
		memcpy(&(mEnds[16 * p]), initMat, 16 * sizeof(float));
		glm::vec3 pos = paths[p].getPosition(0);
		mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
		mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
		mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;

		pthread_mutex_lock(&new_position_available);
		newPositions[p] = pos;
		pthread_mutex_unlock(&new_position_available);
	}

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	float currTime = 0;								// current (virtual) time 
	bool moving = false;							// indicates the particles are moving or not
	bool going = true;								// if it's going, the particles go from the start to targets positions (if it's false, they go from targets to starts)
	while (running) {
		// 5.1. React to user's input
		if (pickingup) {
			moving = true;
			currTime = 0;
			planner.setBoundary(glm::vec3(-0.06,-0.06,-0.06), glm::vec3(+0.06, +0.06, +0.06)); // the working volume needs to be expanded from the default (0.06 x 0.06 x 0.06) as the stage height is out of the default volume
			// compute the paths
			if(going) paths = setPaths(planner, getDetectedPositions(detector, numPoints), cities, size, velocity, true);
			else paths = setPaths(planner, getDetectedPositions(detector, numPoints), distinations, size, velocity, true);
			// update the matrices
			for (int p = 0; p < numPoints; p++) {
				glm::vec3 pos = paths[p].getPosition(0);
				mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
				mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
				mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
				pthread_mutex_lock(&new_position_available);
				newPositions[p] = pos;
				pthread_mutex_unlock(&new_position_available);
			}
			planner.setBoundary(boundaryMin, boundaryMax); // make the boundary back to the default
			turnTransducersOff = false; // turn the transducers on
			pickingup = false;
		}
		else if (startMoving) {
			moving = true;
			currTime = 0;
		
			//// compute the paths
			//paths = setPaths(planner, cities, distinations, size, velocity, false);

			if (going) paths = goPaths;
			else paths = returnPaths;
			going = !going;

			// update the matrices
			for (int p = 0; p < numPoints; p++) {
				glm::vec3 pos = paths[p].getPosition(0);
				mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
				mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
				mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
				pthread_mutex_lock(&new_position_available);
				newPositions[p] = pos;
				pthread_mutex_unlock(&new_position_available);
			}
			for (int i = 0; i < numPoints; i++)
				cities[i] = distinations[i];
			startMoving = false;
		}
		if (amplitudeChanged) {
			float _currentAmplitude;
			pthread_mutex_lock(&new_amplitude_available);
			_currentAmplitude = currentAmplitude;
			pthread_mutex_unlock(&new_amplitude_available);
			printf("Current Amplitude = %f\n", _currentAmplitude);
			for (int i = 0; i < numGeometries*numPoints; i++)
				ampBuffer[i] = _currentAmplitude;
		}
		// 5.2. Update matrices
		bool finished = true;
		for (int p = 0; p < numPoints; p++) {
			glm::vec3 pos = paths[p].getPosition(currTime); // get the positions of the agents according to the current time
			mStarts[16 * p + X_index] = mEnds[16 * p + X_index];
			mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index];
			mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index];
			mEnds[16 * p + X_index] = pos.x;
			mEnds[16 * p + Y_index] = pos.y;
			mEnds[16 * p + Z_index] = pos.z;
			finished &= (paths[p].getFinalTime() < currTime); // check if every agent has finished its path
			pthread_mutex_lock(&new_position_available);
			newPositions[p] = pos;
			pthread_mutex_unlock(&new_position_available);
		}
		if (moving) currTime += dt; // increment the current time
		// 5.3. Compute a hologram
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[0]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		solver->compute(solution);
		solution->transducersOff = turnTransducersOff;
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
	pthread_mutex_init(&new_position_available, NULL);
	pthread_mutex_init(&new_amplitude_available, NULL);
	pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
	pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));

	OpenGLFramework::createWindow(608, 684, true, true);
	OpenGLFramework::setupOpenGL();
	ShowCursor(false);
	createWorldScene();

	std::string imgFileName = "Media/Airplane";
	//STAGE 2: Simply waiting the other threads finish
	{
		projectApplication->createScene();
		for (int i = 0; i < numAnimations; i++) {
			std::stringstream ss;
			ss << imgFileName << "Corrected_" << i + 1 << ".png";
			projectApplication->addTexture(ss.str(), false, false);
		}
		projectApplication->changeTexture(0);

		do {
			float curTime = OpenGLFramework::getTimeInSeconds();
			updateWorldScene(curTime);
			renderWorldScene();
		} while (OpenGLFramework::programEnded() && !projectApplication->applicationFinished());
		projectApplication->destroyScene();
	}
	running = false;
	Sleep(2000);
	a->disconnect();
}

void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff) {
	static float dynamicTargetUPS = targetUPS;
	static float timePerUpdateInMilis = (float)(numGeometries)*1000.0f / dynamicTargetUPS; //this needs to change every time we change dynamicTargetUPS
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
		dynamicTargetUPS += (int)(0.1f*targetUPS);
		if (dynamicTargetUPS > targetUPS)
			dynamicTargetUPS = targetUPS;
		timePerUpdateInMilis = (float)(numGeometries)*1000.0f / dynamicTargetUPS;
		numSolutions = dynamicTargetUPS;
		secCurTime = microTimer::uGetTime();
		printf("Time Per computation = %f; Last Update: %f, UPS: %f\n", (secCurTime - secPrevTime) / 1000000.f, timeSinceLastUpdate / (float)(numGeometries), dynamicTargetUPS);
		secPrevTime = secCurTime;
	}
}
std::vector<AgentPath> setPaths(PathPlanner &planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment) {
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
void fillBuffers(int numPoints, int &pBufferSize, int &aBufferSize, int &cBufferSize, float *&posBuffer, float *&ampBuffer, unsigned char *&colBuffer) {
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
			posBuffer[posInd++] = 0.07f;// boardHeight;
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
void print(const char* str) {
	printf("%s\n", str);
}

//Create the contents and place them where appropriate
void createWorldScene() {
	// 1. Initialize
	// Create the buffer where our stuff is going to be rendered.
	def_fbo = RenderBufferObject::createDefaultRenderBufferObject();
	def_fbo->setBackgroundColour(0.0f, 0.0f, 0.0f);
	// Create a scene to be rendered on the window

	ISceneManager& scene = ISceneManager::instance();

	// Add listener
	InputManager::addKeyboardListener(new KeyControl());
}
//Update the world according to: user events (inputs), time, etc...
void updateWorldScene(float curTime) {
	projectApplication->updateScene(glm::mat4(1.f));
}
//Render our world
void renderWorldScene() {
	glm::mat4 P, V;
	P = glm::mat4(1.f);
	V = glm::mat4(1.f);

	//1. Update rendering
	//1.1. Prepare the buffer for rendering:
	def_fbo->preRender();
	//1.2. Render all our contents
	RenderableVisitor r(P, V);
	ISceneManager::instance().getRootNode().visit(r);
	//1.3. Finish rendering
	def_fbo->postRender();
	postRender();

	projectApplication->changeTexture(currentTexture);
}
void loadPandV(std::string fileName) {
	std::ifstream ifs(fileName);
	if (ifs.is_open()) {
		std::string fileLine;
		int lineCount = 0;
		while (getline(ifs, fileLine)) {
			std::stringstream toParse(fileLine);
			std::vector<std::string> lineTokens;
			std::string token;
			while (getline(toParse >> std::ws, token, ',')) {
				lineTokens.push_back(token);
			}
			if (!lineTokens.empty()) {
				for (int i = 0; i < 4; i++) {
					if (lineCount < 4) projectorP[lineCount][i] = atof(lineTokens[i].c_str());
					else projectorV[lineCount - 4][i] = atof(lineTokens[i].c_str());
				}
				lineCount++;
			}
		}
	}
}
