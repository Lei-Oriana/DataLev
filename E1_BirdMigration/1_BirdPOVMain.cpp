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
#include <OpenGLFrameworkExtensions/ProjectionMapping/CalibrateApplication.h>
#include <OpenGLFrameworkExtensions/ProjectionMapping/ProjectScreenApplication.h>
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
bool VERTICAL = true;
bool INVERSE = false;

void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner &planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false);
void fillBuffers(int numPoints, int &pBufferSize, int &aBufferSize, int &cBufferSize, float *&posBuffer, float *&ampBuffer, unsigned char *&colBuffer);
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);
void print(const char* str);

void createWorldScene();				//Create the contents and place them where appropriate
void renderWorldScene();				// Render our world
void updateWorldScene(float curTime);	// Update the world according to: user events (inputs), time, etc...

using namespace OpenGLFramework;
RenderBufferObject* def_fbo;
FPS_Controller* cameraControl;								//Control the camera from mouse and keyboard (FPS style).
pthread_mutex_t new_target_available;
pthread_mutex_t new_matrix_available;

int applicationID = 0;
CalibrateApplication* calibrateApplication = new CalibrateApplication();
//ProjectScreenApplication* projectApplication = new ProjectScreenApplication();
MappingApplication* projectApplication = new MappingApplication();

float currentAngle, targetAngle;
glm::vec3 currentScale, targetScale;
glm::vec3 currentCentre, targetCentre;

bool calibrating = true;
bool gettingX = true;
int mouseX, mouseY;
int currentPoint = 0;
std::vector<glm::vec2> imagePoints;	//Memory for projector coordinates
std::vector<glm::vec3> objectPoints;	//Memory for projector coordinates
glm::mat4 projectorV, projectorP;
class MouseControl : public IMouseEventListener {
public:
	MouseControl() { ; }
	void leftButtonDown() {
		if (calibrating) {
			static float prevX, prevY;
			if (gettingX) {
				prevX = mouseX;
			}
			else {
				prevY = mouseY;
				imagePoints[currentPoint] = glm::vec2(prevX, prevY);
				std::cout << "Image Point " << currentPoint << " is at (" << prevX << ", " << prevY << ")" << std::endl;
				std::cout << "Object Point " << currentPoint << " is at (" << objectPoints[currentPoint].x << ", " << objectPoints[currentPoint].y << ", " << objectPoints[currentPoint].z << ")" << std::endl;

				if (currentPoint == objectPoints.size() - 1) {
					currentPoint = 0;
					calibrating = false;
				}
				else currentPoint++;
				pthread_mutex_lock(&new_target_available);
				targetCentre = objectPoints[currentPoint];
				pthread_mutex_unlock(&new_target_available);
			}
			gettingX = !gettingX;
		}
	}
	void leftButtonUp() { ; }
	void rightButtonDown() { ; }
	void rightButtonUp() { ; }
	void mouseMove(int x, int y) { mouseX = x; mouseY = y; }
};

int numAnimations = 4;
int currentTexture = 0;
float testHeight = 0.065f;
bool doPOV = false;
static float moveRange = 0.025f;
bool startMoving = false;
glm::vec3 distination;
class KeyControl : public IKeyboardListener {
public:
	KeyControl() { ; }
	void keyPressed(int key) { ; }
	void keyReleased(int key) {
		if (key == GLFW_KEY_S) { startMoving = true; }
		if (key == GLFW_KEY_BACKSPACE) {
			if (currentPoint == 0) currentPoint = objectPoints.size() - 1;
			else currentPoint--;
			pthread_mutex_lock(&new_target_available);
			targetCentre = objectPoints[currentPoint];
			pthread_mutex_unlock(&new_target_available);
			gettingX = true;
		}
		if (key == GLFW_KEY_ENTER) {
			if (currentPoint == objectPoints.size() - 1) currentPoint = 0;
			else currentPoint++;
			pthread_mutex_lock(&new_target_available);
			targetCentre = objectPoints[currentPoint];
			pthread_mutex_unlock(&new_target_available);
			gettingX = true;
		}

		if (key == GLFW_KEY_I) { pthread_mutex_lock(&new_target_available); targetAngle += 10.f; std::cout << "target angle is " << targetAngle << "degree" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_K) { pthread_mutex_lock(&new_target_available);  targetAngle -= 10.f; std::cout << "target angle is " << targetAngle << "degree" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_L) { pthread_mutex_lock(&new_target_available); targetAngle += 45.f; std::cout << "target angle is " << targetAngle << "degree" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_J) { pthread_mutex_lock(&new_target_available); targetAngle -= 45.f; std::cout << "target angle is " << targetAngle << "degree" << std::endl; pthread_mutex_unlock(&new_target_available); }

		if (key == GLFW_KEY_UP) { pthread_mutex_lock(&new_target_available); targetCentre.x += 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_DOWN) { pthread_mutex_lock(&new_target_available); targetCentre.x -= 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_LEFT) { pthread_mutex_lock(&new_target_available); targetCentre.y += 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_RIGHT) { pthread_mutex_lock(&new_target_available); targetCentre.y -= 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_PAGE_UP) { pthread_mutex_lock(&new_target_available); targetCentre.z += 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_PAGE_DOWN) { pthread_mutex_lock(&new_target_available); targetCentre.z -= 0.001f; std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }

		if (key == GLFW_KEY_0) { startMoving = true; distination = glm::vec3(0, 0, testHeight); }
		if (key == GLFW_KEY_1) { startMoving = true; distination = glm::vec3(+0.027, -0.030, testHeight); }
		if (key == GLFW_KEY_2) { startMoving = true; distination = glm::vec3(+0.004, +0.007, testHeight); }
		if (key == GLFW_KEY_4) { pthread_mutex_lock(&new_target_available); targetCentre = glm::vec3(+0.027, -0.030, testHeight); std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }

		if (key == GLFW_KEY_7) { pthread_mutex_lock(&new_target_available); targetCentre = glm::vec3(+0.011, -0.048, testHeight); std::cout << "target centre is at (" << targetCentre.x << ", " << targetCentre.y << ", " << targetCentre.z << ")" << std::endl; pthread_mutex_unlock(&new_target_available); }
		if (key == GLFW_KEY_8) { currentTexture = (currentTexture - 1 + numAnimations) % numAnimations; }
		if (key == GLFW_KEY_9) { currentTexture = (currentTexture + 1) % numAnimations; }
	}
};

void glmMat2Floats(std::vector<glm::mat4> glmStarts, std::vector<glm::mat4> glmEnds, float *floatStarts, float *floatEnds);
void computeProjectionViewMatrixes();
void savePandV(std::string fileName);
void loadPandV(std::string fileName);
void updateMatrices(int numPoints, float *mStarts, float *mEnds);

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

	// 3. Fill buffers: we set all the positions at (0, 0, 0.12), the centre of the working volume
	int numPoints = 1;
	int pBufferSize, aBufferSize, cBufferSize;
	float *posBuffer, *ampBuffer;
	unsigned char *colBuffer;
	fillBuffers(numPoints, pBufferSize, aBufferSize, cBufferSize, posBuffer, ampBuffer, colBuffer);
	float posStaticBuffer[32 * 4 * 4];
	for (int i = 0; i < 32 * numPoints; i++) {
		posStaticBuffer[i * 4 + 0] = 0;
		posStaticBuffer[i * 4 + 1] = 0;
		posStaticBuffer[i * 4 + 2] = 0;
		posStaticBuffer[i * 4 + 3] = 1;
	}

	// 4. Define matrices to update the positions of the traps
	float initMat[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	float mStarts[16 * 16], mEnds[16 * 16];
	static const size_t X_index = 3, Y_index = 7, Z_index = 11;
	for (int p = 0; p < numPoints; p++) {
		memcpy(&(mStarts[16 * p]), initMat, 16 * sizeof(float));
		memcpy(&(mEnds[16 * p]), initMat, 16 * sizeof(float));
	}
	mStarts[16 * 1 + X_index] = mEnds[16 * 1 + X_index] = 0.03f;
	mStarts[16 * 1 + Y_index] = mEnds[16 * 1 + Y_index] = 0.03f;
	mStarts[16 * 1 + Z_index] = mEnds[16 * 1 + Z_index] = testHeight;

	// 5. Run the loop for the writing thread
	bool staticBuffer = true;
	int pIndex = 0, aIndex = 0, cIndex = 0;
	while (running) {
		updateMatrices(numPoints, mStarts, mEnds);

		// 5.3. Compute a hologram
		GSPAT::Solution* solution;
		if (doPOV) staticBuffer = false;
		if (staticBuffer) solution = solver->createSolution(numPoints, numGeometries, true, &(posStaticBuffer[0]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		else solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[pIndex]), &(ampBuffer[aIndex]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		solver->compute(solution);
		// 5.4. Pass the solution to the reader thread
		pthread_mutex_lock(&mutex_solution_queue);
		solutions.push_back(solution);
		pthread_mutex_unlock(&solution_available);
		pthread_mutex_unlock(&mutex_solution_queue);
		// 5.5. Update the status
		if (doPOV || pIndex % (1024 * 4) != 0) {
			pIndex += numPoints * numGeometries * 4;
			if (pIndex > pBufferSize - numPoints * numGeometries * 4) pIndex = 0;
			aIndex += numPoints * numGeometries;
			if (aIndex > aBufferSize - numPoints * numGeometries) aIndex = 0;
		}
		else {
			staticBuffer = true;
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
	a->connect(21, 15);
	a->readParameters(transducerPositions, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	// Set up calibration points
	{
		float radius = 0.06f;
		glm::vec3 calibCentre(0, 0, 0.06f);
		objectPoints.push_back(calibCentre + glm::vec3(+radius, +radius, 0));
		objectPoints.push_back(calibCentre + glm::vec3(-radius, +radius, 0));
		objectPoints.push_back(calibCentre + glm::vec3(-radius, -radius, 0));
		objectPoints.push_back(calibCentre + glm::vec3(+radius, -radius, 0));

		imagePoints.resize(objectPoints.size());
		currentAngle = targetAngle = 0;
		currentScale = targetScale = glm::vec3(1.f, 1.f, 1.f);
		currentCentre = targetCentre = glm::vec3(0, 0, 0.12f);
	}
	//STAGE 1: Create the reader and writer threads and the concurrency control variables:
	pthread_mutex_init(&solution_available, NULL);
	pthread_mutex_lock(&solution_available);
	pthread_mutex_init(&mutex_solution_queue, NULL);
	pthread_mutex_init(&new_target_available, NULL);
	pthread_mutex_init(&new_matrix_available, NULL);
	pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
	pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));

	OpenGLFramework::createWindow(608, 684, true, true);
	OpenGLFramework::setupOpenGL();
	ShowCursor(false);
	createWorldScene();

	// 1. Calibrate the projector
	bool calibration = true;
	std::string imgFileName = "Media/Airplane";
	//std::string fileNamePV = "Media/MatricesPandV.csv";
	if (calibration) {
		std::cout << "Levitate a bead and click when the projector illuminates it to get the projection matrix" << std::endl;
		std::cout << "(this will repeat " << objectPoints.size() << " times" << std::endl;
		applicationID = 1;
		pthread_mutex_lock(&new_target_available);
		currentScale = targetScale = glm::vec3(1, 1, 1);
		pthread_mutex_unlock(&new_target_available);
		calibrateApplication->createScene();
		do {
			float curTime = OpenGLFramework::getTimeInSeconds();
			updateWorldScene(curTime);
			renderWorldScene();
		} while (OpenGLFramework::programEnded() && calibrating);
		calibrateApplication->destroyScene();
		//computeProjectionViewMatrixes();
		//if (!calibrating) savePandV(fileNamePV);

		for (int i = 0; i < numAnimations; i++) {
			std::stringstream ss;
			ss << imgFileName << "_"  << i+1 << ".png";
			cv::Mat img = cv::imread(ss.str());
			int width = 1024*2, height = 1024*2;
			if (img.cols != width || img.rows != height)
				cv::resize(img, img, cv::Size(width, height), 0.0, 0.0, cv::INTER_CUBIC);
			int imgWidth = img.cols; int imgHeight = img.rows;
			int wndWidth = OpenGLFramework::getWindowWidth();
			int wndHeight = OpenGLFramework::getWindowHeight();
			std::vector<cv::Point2d> srcCorners;
			srcCorners.push_back(cv::Point2d(0, 0));
			srcCorners.push_back(cv::Point2d(0, imgHeight - 1));
			srcCorners.push_back(cv::Point2d(imgWidth - 1, imgHeight - 1));
			srcCorners.push_back(cv::Point2d(imgWidth - 1, 0));
			float wRatio = (float)imgWidth / (float)wndWidth;
			float hRatio = (float)imgHeight / (float)wndHeight;
			std::vector<cv::Point2d> dstCorners;
			for (int i = 0; i < srcCorners.size(); i++)
				dstCorners.push_back(cv::Point2d(imagePoints[i].x * wRatio, imagePoints[i].y * hRatio));

			cv::Mat homography = cv::findHomography(srcCorners, dstCorners);
			cv::Mat correctedImage(imgWidth, imgHeight, img.type());
			warpPerspective(img, correctedImage, homography, correctedImage.size());
			//cv::resize(correctedImage, correctedImage, cv::Size(width/2, height/2), 0.0, 0.0, cv::INTER_CUBIC);
			ss.str("");
			ss << imgFileName << "Corrected_" << i + 1 << ".png";
			cv::imwrite(ss.str(), correctedImage);
		}
	}
	//else loadPandV(fileNamePV);

	// 2. Project the image onto the screen
	std::cout << "Press the space key when it's ready" << std::endl;
	applicationID = 2;
	pthread_mutex_lock(&new_target_available);
	currentCentre = targetCentre = glm::vec3(0, 0, testHeight);
	pthread_mutex_unlock(&new_target_available);
	cv::Mat testPattern;
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

			static bool moving = false;
			static std::vector<glm::vec3> distinations;
			if (startMoving) {
				glm::vec3 startCentre;
				pthread_mutex_lock(&new_target_available); 
				startCentre = currentCentre;
				pthread_mutex_unlock(&new_target_available);
				glm::vec3 midPoint = 0.5f * (distination + startCentre);
				midPoint.z += 0.03f;
				distinations.push_back(midPoint);
				distinations.push_back(distination);

				glm::vec3 direction = distination - startCentre;
				float distance = glm::length(direction);
				glm::vec3 unitary = (distance > 0 ? direction / distance : glm::vec3(0, -1, 0));
				pthread_mutex_lock(&new_target_available);
				targetAngle = currentAngle = acos(glm::dot(unitary, glm::vec3(0, 1, 0)));
				pthread_mutex_unlock(&new_target_available);

				startMoving = false;
				moving = true;
				doPOV = true;
			}
			if (moving) {
				glm::vec3 _currentCentre, _targetCentre;
				pthread_mutex_lock(&new_target_available);
				_currentCentre = currentCentre;
				_targetCentre = targetCentre;
				pthread_mutex_unlock(&new_target_available);
				if (glm::distance(_currentCentre, _targetCentre) < 0.001f) {
					if (distinations.size() == 0) {
						moving = false;
						doPOV = false;
					}
					else {
						pthread_mutex_lock(&new_target_available);
						targetCentre = distinations[0];
						pthread_mutex_unlock(&new_target_available);
						distinations.erase(distinations.begin());
					}
				}
			}
		} while (OpenGLFramework::programEnded() && !projectApplication->applicationFinished());
		projectApplication->destroyScene();
	}

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
	int revolutionsPerSecond = 10;
	int numWing = 40;

	// allocate memory buffers
	int angleStepsPerRevolution = targetUPS / revolutionsPerSecond;
	pBufferSize = numPoints * angleStepsPerRevolution * numWing * 4;
	aBufferSize = numPoints * angleStepsPerRevolution * numWing * 1;
	cBufferSize = angleStepsPerRevolution * numWing * 3;
	posBuffer = new float[pBufferSize];
	ampBuffer = new float[aBufferSize];
	colBuffer = new unsigned char[cBufferSize];

	// Define actual content we create
	float radius = 0.004;
	int numWingQuater = numWing / 4;
	float wingAngle = 0.6;

	float initinalAngle = glm::radians(0.f);
	int posInd = 0, ampInd = 0, colInd = 0;
	for (int w = 0; w < numWing; w++) {
		for (int s = 0; s < angleStepsPerRevolution; s++) {
			float angle = (4 * M_PI * s) / angleStepsPerRevolution;
			//Compute position of each point:
			for (int p = 0; p < numPoints; p++) {
				glm::vec3 position;
				position.x = radius * (1.0f - cosf(angle));
				position.y = radius * sinf(angle);
				position.z = 0;

				float wp;
				if ((w / numWingQuater) % 4 == 0) wp = (w % numWingQuater) / (float)numWingQuater;
				else if ((w / numWingQuater) % 4 == 1) wp = 1.0 - (w % numWingQuater) / (float)numWingQuater;
				else if ((w / numWingQuater) % 4 == 2) wp = -(w % numWingQuater) / (float)numWingQuater;
				else wp = -1.0 + (w % numWingQuater) / (float)numWingQuater;
				wp *= wingAngle * 0.5 * M_PI;

				if (p % 2 == 1)
					wp *= -1.f;

				if (s >= angleStepsPerRevolution / 2) {
					position.x *= -1.0f;
					wp *= -1.0f;
				}

				glm::mat3 wingRotation(cos(wp), 0, -sin(wp), 0, 1, 0, sin(wp), 0, cos(wp));
				glm::vec3 butterfly = wingRotation * position;

				glm::mat3 R = glm::rotate(glm::mat4(1.0f), initinalAngle, glm::vec3(0, 0, 1.0f));
				butterfly = R * butterfly;
				if (p == 0) {
					posBuffer[posInd++] = butterfly.x;
					posBuffer[posInd++] = butterfly.y;
					posBuffer[posInd++] = butterfly.z;
					posBuffer[posInd++] = 1;
					ampBuffer[ampInd++] = 16000;
				}
				if (p == 1) {
					posBuffer[posInd++] = 0;
					posBuffer[posInd++] = 0;
					posBuffer[posInd++] = 0;
					posBuffer[posInd++] = 1;
					ampBuffer[ampInd++] = 8000;
				}
			}
			colBuffer[colInd++] = 0;
			colBuffer[colInd++] = 0;
			colBuffer[colInd++] = 0;
		}
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
	InputManager::addMouseListener(new MouseControl());
}
//Update the world according to: user events (inputs), time, etc...
void updateWorldScene(float curTime) {
	if (applicationID == 1) {
		calibrateApplication->updateScene(mouseX, mouseY, !gettingX, gettingX);
	}
	else if (applicationID == 2) {
		static bool firstTime = true;
		if (firstTime) {
			projectApplication->updateScene(glm::mat4(1.f));
			firstTime = false;
		}
	}
}
//Render our world
void renderWorldScene() {
	glm::mat4 P, V;
	//Simply render on the monitor
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

	if(applicationID == 2)
		projectApplication->changeTexture(currentTexture);
}

void computeProjectionViewMatrixes() {
	std::cout << "Got all needed points" << std::endl;
	std::cout << "Calibrating ..." << std::endl;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

	std::ifstream ifs("Media/LightCrafterCalibration.csv");
	if (ifs.is_open()) {
		std::string fileLine;
		int lineCount = 0;
		while (getline(ifs, fileLine)) {
			std::stringstream toParse(fileLine);
			std::vector<std::string> lineTokens;
			std::string token;
			while (std::getline(toParse >> std::ws, token, ',')) {
				lineTokens.push_back(token);
			}
			if (!lineTokens.empty()) {
				lineCount++;
				if (lineCount == 3 || lineCount == 4 || lineCount == 5) {
					for (int i = 0; i < 3; i++)
						cameraMatrix.at<double>(lineCount - 3, i) = atof(lineTokens[i].c_str());
				}
				if (lineCount == 7) {
					for (int i = 0; i < lineTokens.size(); i++)
						distCoeffs.at<double>(i) = atof(lineTokens[i].c_str());
				}
			}
		}
	}
	else std::cout << "ERROR: File cannot be opened!" << std::endl;

	cv::Mat rvec, tvec;
	//put the image and object points in the right order for opencv
	std::vector<cv::Point3f> vo; //object points
	std::vector<cv::Point2f> vi; //image points
	for (unsigned int i = 0; i < objectPoints.size(); ++i) {
		cv::Point3f cvObjectPoint(objectPoints[i].x, objectPoints[i].y, objectPoints[i].z);
		cv::Point2f cvImagePoint(imagePoints[i].x, imagePoints[i].y);
		vo.push_back(cvObjectPoint);
		vi.push_back(cvImagePoint);
	}

	//when enough points found, get the extrinsics/intrisics parameter
	float h = OpenGLFramework::getWindowHeight();
	float w = OpenGLFramework::getWindowWidth();
	cv::Size projectorResolution(w, h);

	double rms = solvePnP(vo, vi, cameraMatrix, distCoeffs, rvec, tvec, false);
	std::cout << ".. done , RMS=" << rms << std::endl;

	//build the opengl view matrix
	cv::Mat rot;
	rvec.copyTo(rot);
	cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
	cv::Rodrigues(rot, rotMat);
	cv::Mat V = (cv::Mat1d(4, 4) <<
		rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2), tvec.at<double>(0, 0),
		rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2), tvec.at<double>(1, 0),
		rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2), tvec.at<double>(2, 0),
		0, 0, 0, 1);

	//build the opengl projection matrix
	double fx = cameraMatrix.at<double>(0, 0);
	double cx = cameraMatrix.at<double>(0, 2);
	double fy = cameraMatrix.at<double>(1, 1);
	double cy = cameraMatrix.at<double>(1, 2);
	double width = projectorResolution.width;
	double height = projectorResolution.height;
	double _near = 0.1;
	double _far = 10;
	cv::Mat P = (cv::Mat1d(4, 4) <<
		2.0*fx / width, 0, 1.0 - 2.0*cx / width, 0,
		0, 2.0*fy / height, -1.0 + 2.0*cy / height, 0,
		0, 0, (_far + _near) / (_near - _far), (2.0*_far*_near) / (_near - _far),
		0, 0, -1, 0);

	std::cout << "Camera matrix: " << std::endl << cameraMatrix << std::endl << std::endl;
	std::cout << "Distortion coefficients: " << std::endl << distCoeffs << std::endl << std::endl;
	std::cout << "Translate vector: " << std::endl << tvec << std::endl << std::endl;
	std::cout << "Rotation matrix: " << std::endl << rotMat << std::endl << std::endl;

	//write to file
	std::string outputFileName = "projector";
	std::ostringstream oss;
	outputFileName += oss.str() + ".txt";
	std::cout << "Writing results to " << outputFileName << std::endl;

	cv::FileStorage fs("kinectProjector.yml", cv::FileStorage::WRITE);
	fs << "ProjectionMatrix" << P;
	fs << "ViewMatrix" << V;
	fs << "Intrinsics" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs << "error" << rms;
	fs.release();


	std::ofstream outputFile;
	outputFile.open("kinectProjector.txt");
	outputFile << "Camera Matrix" << std::endl;
	outputFile << cameraMatrix << std::endl;
	outputFile << std::endl;
	outputFile << "Projector translation" << std::endl;
	outputFile << tvec << std::endl;
	outputFile << std::endl;
	outputFile << "Projector rotation" << std::endl;
	outputFile << rvec << std::endl;
	outputFile << std::endl;
	outputFile << "OpenGL Projection Matrix" << std::endl;
	outputFile << P << std::endl;
	outputFile << std::endl;
	outputFile << "OpenGL View Matrix" << std::endl;
	outputFile << V << std::endl;
	outputFile << std::endl;
	outputFile.close();

	// convert V matrix in openCV to openGL
	glm::mat4 _P = glm::mat4(
		P.at<double>(cvPoint(0, 0)), P.at<double>(cvPoint(0, 1)), P.at<double>(cvPoint(0, 2)), P.at<double>(cvPoint(0, 3)),
		P.at<double>(cvPoint(1, 0)), P.at<double>(cvPoint(1, 1)), P.at<double>(cvPoint(1, 2)), P.at<double>(cvPoint(1, 3)),
		P.at<double>(cvPoint(2, 0)), P.at<double>(cvPoint(2, 1)), P.at<double>(cvPoint(2, 2)), P.at<double>(cvPoint(2, 3)),
		P.at<double>(cvPoint(3, 0)), P.at<double>(cvPoint(3, 1)), P.at<double>(cvPoint(3, 2)), P.at<double>(cvPoint(3, 3)));;
	/* //This used to work with left handed*/
	glm::mat4 _V = glm::mat4(
		V.at<double>(cvPoint(0, 0)), V.at<double>(cvPoint(0, 1)), V.at<double>(cvPoint(0, 2)), V.at<double>(cvPoint(0, 3)),
		V.at<double>(cvPoint(1, 0)), V.at<double>(cvPoint(1, 1)), V.at<double>(cvPoint(1, 2)), V.at<double>(cvPoint(1, 3)),
		V.at<double>(cvPoint(2, 0)), V.at<double>(cvPoint(2, 1)), V.at<double>(cvPoint(2, 2)), V.at<double>(cvPoint(2, 3)),
		V.at<double>(cvPoint(3, 0)), V.at<double>(cvPoint(3, 1)), V.at<double>(cvPoint(3, 2)), V.at<double>(cvPoint(3, 3)));;

	// right-handed coordinate
	projectorP = _P;
	glm::mat4 adjustInvertedV = glm::inverse(_V);
	adjustInvertedV = glm::rotate(adjustInvertedV, glm::radians(180.0f), glm::vec3(1, 0, 0));
	_V = glm::inverse(adjustInvertedV);
	projectorV = _V;

	glm::vec4 proPos = glm::inverse(_V) * glm::vec4(0, 0, 0, 1);
	std::cout << "Projector Position: (" << proPos.x << ", " << proPos.y << ", " << proPos.z << ")" << std::endl;
	std::cout << "Distance: " << sqrt(proPos.x*proPos.x + proPos.y*proPos.y + (proPos.z - 0.1194f)*(proPos.z - 0.1194f)) << std::endl;
}
void savePandV(std::string fileName) {
	std::ofstream ofs(fileName);
	ofs << projectorP[0][0] << ", " << projectorP[0][1] << ", " << projectorP[0][2] << ", " << projectorP[0][3] << ", " << std::endl;
	ofs << projectorP[1][0] << ", " << projectorP[1][1] << ", " << projectorP[1][2] << ", " << projectorP[1][3] << ", " << std::endl;
	ofs << projectorP[2][0] << ", " << projectorP[2][1] << ", " << projectorP[2][2] << ", " << projectorP[2][3] << ", " << std::endl;
	ofs << projectorP[3][0] << ", " << projectorP[3][1] << ", " << projectorP[3][2] << ", " << projectorP[3][3] << ", " << std::endl;
	ofs << projectorV[0][0] << ", " << projectorV[0][1] << ", " << projectorV[0][2] << ", " << projectorV[0][3] << ", " << std::endl;
	ofs << projectorV[1][0] << ", " << projectorV[1][1] << ", " << projectorV[1][2] << ", " << projectorV[1][3] << ", " << std::endl;
	ofs << projectorV[2][0] << ", " << projectorV[2][1] << ", " << projectorV[2][2] << ", " << projectorV[2][3] << ", " << std::endl;
	ofs << projectorV[3][0] << ", " << projectorV[3][1] << ", " << projectorV[3][2] << ", " << projectorV[3][3] << ", " << std::endl;
	ofs.close();
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
void glmMat2Floats(std::vector<glm::mat4> glmStarts, std::vector<glm::mat4> glmEnds, float *floatStarts, float *floatEnds) {
	int numMatrices = glmStarts.size();
	for (int i = 0; i < numMatrices; i++) {
		floatStarts[i * 16 + 0] = glmStarts[i][0][0]; floatStarts[i * 16 + 1] = glmStarts[i][1][0]; floatStarts[i * 16 + 2] = glmStarts[i][2][0]; floatStarts[i * 16 + 3] = glmStarts[i][3][0];
		floatStarts[i * 16 + 4] = glmStarts[i][0][1]; floatStarts[i * 16 + 5] = glmStarts[i][1][1]; floatStarts[i * 16 + 6] = glmStarts[i][2][1]; floatStarts[i * 16 + 7] = glmStarts[i][3][1];
		floatStarts[i * 16 + 8] = glmStarts[i][0][2]; floatStarts[i * 16 + 9] = glmStarts[i][1][2]; floatStarts[i * 16 + 10] = glmStarts[i][2][2]; floatStarts[i * 16 + 11] = glmStarts[i][3][2];
		floatStarts[i * 16 + 12] = glmStarts[i][0][3]; floatStarts[i * 16 + 13] = glmStarts[i][1][3]; floatStarts[i * 16 + 14] = glmStarts[i][2][3]; floatStarts[i * 16 + 15] = glmStarts[i][3][3];
		floatEnds[i * 16 + 0] = glmEnds[i][0][0]; floatEnds[i * 16 + 1] = glmEnds[i][1][0]; floatEnds[i * 16 + 2] = glmEnds[i][2][0]; floatEnds[i * 16 + 3] = glmEnds[i][3][0];
		floatEnds[i * 16 + 4] = glmEnds[i][0][1]; floatEnds[i * 16 + 5] = glmEnds[i][1][1]; floatEnds[i * 16 + 6] = glmEnds[i][2][1]; floatEnds[i * 16 + 7] = glmEnds[i][3][1];
		floatEnds[i * 16 + 8] = glmEnds[i][0][2]; floatEnds[i * 16 + 9] = glmEnds[i][1][2]; floatEnds[i * 16 + 10] = glmEnds[i][2][2]; floatEnds[i * 16 + 11] = glmEnds[i][3][2];
		floatEnds[i * 16 + 12] = glmEnds[i][0][3]; floatEnds[i * 16 + 13] = glmEnds[i][1][3]; floatEnds[i * 16 + 14] = glmEnds[i][2][3]; floatEnds[i * 16 + 15] = glmEnds[i][3][3];
	}
}
void updateMatrices(int numPoints, float *mStarts, float *mEnds) {
	static glm::mat4 start, end;
	const float scaleStep = 0.0005f;
	const float rotateStep = 0.5f;
	const float transStep = 0.00002f;// 0.0001f;

	static glm::vec3 _currentScale, _targetScale;
	static float _currentAngle, _targetAngle;
	static glm::vec3 _currentCentre, _targetCentre;
	pthread_mutex_lock(&new_target_available);
	_currentScale = currentScale;
	_currentAngle = currentAngle;
	_currentCentre = currentCentre;
	_targetScale = targetScale;
	_targetAngle = targetAngle;
	_targetCentre = targetCentre;
	pthread_mutex_unlock(&new_target_available);


	static glm::mat4 matS = glm::scale(glm::mat4(1.f), _currentScale);
	static glm::mat4 matR = glm::rotate(glm::mat4(1.f), _currentAngle, glm::vec3(0, 0, 1.f));
	static glm::mat4 matT = glm::translate(glm::mat4(1.f), _currentCentre);
	start = matT * matR * matS;

	// update the current scale
	{
		glm::vec3 difference = _targetScale - _currentScale;
		float distance = glm::length(difference);
		if (distance > 0.51 * scaleStep) {
			glm::vec3 unitary = difference / distance;
			_currentScale = _currentScale + transStep * unitary;
		}
		else currentScale = _targetScale;
	}

	// update the current rotation angle
	{
		if (_targetAngle - _currentAngle > M_PI) _targetAngle -= (2.f * M_PI);
		if (_targetAngle - _currentAngle <= -M_PI) _targetAngle += (2.f * M_PI);
		float distance = abs(_targetAngle - _currentAngle);
		if (distance > 0.51 * rotateStep) {
			float unitary = _targetAngle > _currentAngle ? 1 : -1;
			_currentAngle = _currentAngle + rotateStep * unitary;
		}
		else {
			_currentAngle = _targetAngle;
		}
		if (_currentAngle < 0) _currentAngle += (2.f * M_PI);
		if (_currentAngle >= (2.f * M_PI)) _currentAngle -= (2.f * M_PI);
	}

	// update the current position of the centre
	{
		glm::vec3 difference = _targetCentre - _currentCentre;
		float distance = glm::length(difference);
		if (distance > 0.51 * transStep) {
			glm::vec3 unitary = difference / distance;
			_currentCentre = _currentCentre + transStep * unitary;
		}
		else _currentCentre = _targetCentre;
	}

	matS = glm::scale(glm::mat4(1.f), _currentScale);
	matR = glm::rotate(glm::mat4(1.f), _currentAngle, glm::vec3(0, 0, 1.f));
	matT = glm::translate(glm::mat4(1.f), _currentCentre);
	end = matT * matR * matS;

	pthread_mutex_lock(&new_target_available);
	currentScale = _currentScale;
	currentAngle = _currentAngle;
	currentCentre = _currentCentre;
	pthread_mutex_unlock(&new_target_available);


	//start = end = matT;// glm::mat4(1.f);
	//for (int i = 0; i < numPoints; i++) {
	for (int i = 0; i < 1; i++) {
		mStarts[i * 16 + 0] = start[0][0]; mStarts[i * 16 + 1] = start[1][0]; mStarts[i * 16 + 2] = start[2][0]; mStarts[i * 16 + 3] = start[3][0];
		mStarts[i * 16 + 4] = start[0][1]; mStarts[i * 16 + 5] = start[1][1]; mStarts[i * 16 + 6] = start[2][1]; mStarts[i * 16 + 7] = start[3][1];
		mStarts[i * 16 + 8] = start[0][2]; mStarts[i * 16 + 9] = start[1][2]; mStarts[i * 16 + 10] = start[2][2]; mStarts[i * 16 + 11] = start[3][2];
		mStarts[i * 16 + 12] = start[0][3]; mStarts[i * 16 + 13] = start[1][3]; mStarts[i * 16 + 14] = start[2][3]; mStarts[i * 16 + 15] = start[3][3];
		mEnds[i * 16 + 0] = end[0][0]; mEnds[i * 16 + 1] = end[1][0]; mEnds[i * 16 + 2] = end[2][0]; mEnds[i * 16 + 3] = end[3][0];
		mEnds[i * 16 + 4] = end[0][1]; mEnds[i * 16 + 5] = end[1][1]; mEnds[i * 16 + 6] = end[2][1]; mEnds[i * 16 + 7] = end[3][1];
		mEnds[i * 16 + 8] = end[0][2]; mEnds[i * 16 + 9] = end[1][2]; mEnds[i * 16 + 10] = end[2][2]; mEnds[i * 16 + 11] = end[3][2];
		mEnds[i * 16 + 12] = end[0][3]; mEnds[i * 16 + 13] = end[1][3]; mEnds[i * 16 + 14] = end[2][3]; mEnds[i * 16 + 15] = end[3][3];
	}
}
