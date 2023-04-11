#include <OpenMPD_Prerequisites.h>
#include <OpenMPD_CWrapper.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <vector>
#include <../Helper/TimeFunctions.h>
#include <sstream>
#include <Windows.h>
#define _USE_MATH_DEFINES

void print(const char* msg) { printf("%s\n", msg); }
void* client(void* arg);
float* createSampledArc(float origin[3], float p0[3], float angleInRads, cl_uint numSamples);
float* createSampledCardioid(float origin[3], float angleInRads, float scale, float displacement, cl_uint numSamples);
int curFPS_Divider = 4;

int main() {

	do {
		do {
			OpenMPD_CWrapper_Initialize();
			OpenMPD_CWrapper_RegisterPrintFuncs(print, print, print);
			OpenMPD_CWrapper_SetupEngine(2000000, OpenMPD::GSPAT_SOLVER::V2);
			OpenMPD_Context_Handler  pm = OpenMPD_CWrapper_StartEngine_TopBottom(curFPS_Divider, 32, 4, 2, true);
			OpenMPD_CWrapper_SetupPhaseOnly(false);
			client((void*)pm);
			OpenMPD_CWrapper_StopEngine();
			printf("Press any key to Restart the engine.\nPress SPACE BAR to Release current engine.");
		} while (_getch() != ' ');
		OpenMPD_CWrapper_Release();
		printf("Engine Released (all structures destroyed).\nPress any key to Restart a new instance.\nPress SPACE BAR to finish program.");
	} while (_getch() != ' ');
}
//Client thread data
cl_uint pri1, pri2;
cl_uint pos1, pos2, circle1, circle2, cardioidPath, hapticCirclePath;
cl_uint amp1, amp2;
cl_uint numSamplesCardioid = 2000;
cl_uint numSamplesHapticCircle = 4000;

void declareContent(OpenMPD_Context_Handler pm);
void destroyContent(OpenMPD_Context_Handler pm);

void countFrames() {
	static bool first = true;
	static int numFramesSinceLastSecond;
	static struct timeval last, cur;
	static float lastSecond;
	if (first) {
		first = false;
		numFramesSinceLastSecond = 0;
		gettimeofday(&last, NULL);
	}
	numFramesSinceLastSecond++;
	gettimeofday(&cur, NULL);
	float milisElapsed = computeTimeElapsedInMilis(last, cur);
	if (milisElapsed >= 1000) {
		printf("High-level FPS %f\n", 1000 * numFramesSinceLastSecond / milisElapsed);
		last = cur;
		numFramesSinceLastSecond = 0;
	}
}

void* client(void* arg) {
	// counter
	static int framesCounter = 0;
	static struct timeval prevTime, curTime;
	static double totalTime = 0;
	static bool firstTime = true;
	static int reportPeriod = 1;

	OpenMPD_Context_Handler pm = (OpenMPD_Context_Handler)arg;
	declareContent(pm);
	bool running = true;
	cl_uint primitives[] = { pri1, pri2 };//Declared as global shared variables
	static const size_t X_index = 3, Y_index = 7, Z_index = 11;
	float matrices[] = { 1,0,0,0,
					  0,1,0,0,
					  0,0,1,0,
					  0,0,0,1,
					  1,0,0,0,
					  0,1,0,0,
					  0,0,1,0,
					  0,0,0,1 };
	float* cur = &(matrices[0]), * prev = &(matrices[16]);
	while (running) {
		bool commit = false;
		//LEFT_RIGHT
		//scanf("%c", &c);
#pragma region 1 seconds report
		if (firstTime) {
			gettimeofday(&prevTime, 0x0);
			firstTime = false;
		}
		else {
			gettimeofday(&curTime, 0x0);
			totalTime += computeTimeElapsed(prevTime, curTime);
			//totalTime += computeTimeElapsed(prevTime, curTime);			
			prevTime = curTime;
		}

		if (totalTime <= reportPeriod) {
			framesCounter++;
		}
		else {
			static char logPerformance[512];
			sprintf_s(logPerformance, "Frames counter (last %f seconds): Actual UPS: %f, Target UPS: %f\n", totalTime, framesCounter, 10000);
			framesCounter = 0;
			totalTime = 0;
		}
#pragma endregion

		if (_kbhit())
			switch (_getch()) {
				//UPDATE DIVIDER:
			case 'e':
				curFPS_Divider++;
				OpenMPD_CWrapper_SetupFPS_Divider(curFPS_Divider);
				break;
			case 'q':
				curFPS_Divider--;
				OpenMPD_CWrapper_SetupFPS_Divider(curFPS_Divider);
				break;
				//MOVE
			case 'a':
				commit = true;
				prev[X_index] = cur[X_index];
				cur[X_index] += 0.001f;
				prev[Y_index] = cur[Y_index];
				prev[Z_index] = cur[Z_index];
				break;

			case 'd':
				commit = true;
				prev[X_index] = cur[X_index];
				cur[X_index] -= 0.001f;
				prev[Y_index] = cur[Y_index];
				prev[Z_index] = cur[Z_index];
				break;
			case 'w':
				commit = true;
				prev[Y_index] = cur[Y_index];
				cur[Y_index] += 0.001f;
				prev[X_index] = cur[X_index];
				prev[Z_index] = cur[Z_index];
				break;
			case 's':
				commit = true;
				prev[Y_index] = cur[Y_index];
				cur[Y_index] -= 0.001f;
				prev[X_index] = cur[X_index];
				prev[Z_index] = cur[Z_index];
				break;
			case 'r':
				commit = true;
				prev[Y_index] = cur[Y_index];
				prev[X_index] = cur[X_index];
				prev[Z_index] = cur[Z_index];
				cur[Z_index] += 0.0005f;
				break;
			case 'f':
				commit = true;
				prev[Y_index] = cur[Y_index];
				prev[X_index] = cur[X_index];
				prev[Z_index] = cur[Z_index];
				cur[Z_index] -= 0.0005f;
				break;
			case ' ':
				printf("SPACE BAR pressed");
				running = false;
				break;
				//CHANGE STATE
			case '1':
				OpenMPD_CWrapper_updatePrimitive_Positions(pm, pri1, hapticCirclePath, 0);
				OpenMPD_CWrapper_updatePrimitive_Positions(pm, pri2, cardioidPath, 0);
				OpenMPD_CWrapper_commitUpdates(pm);
				break;
			case '2':
				OpenMPD_CWrapper_updatePrimitive_Positions(pm, pri1, pos1, 0);
				OpenMPD_CWrapper_updatePrimitive_Positions(pm, pri2, pos2, 0);
				OpenMPD_CWrapper_commitUpdates(pm);
				break;
			case '3':
				OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri1, false);
				OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri2, false);
				OpenMPD_CWrapper_commitUpdates(pm);
				break;
			case '4':
				OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri1, true);
				OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri2, true);
				OpenMPD_CWrapper_commitUpdates(pm);
				break;
			}
		//Update engine:
		if (commit)
			printf("(%f, %f, %f)\n", cur[X_index], cur[Y_index], cur[Z_index]);
		float mStarts[32], mEnds[32];
		memcpy(&(mStarts[0]), prev, 16 * sizeof(float));
		memcpy(&(mStarts[16]), prev, 16 * sizeof(float));
		memcpy(&(mEnds[0]), cur, 16 * sizeof(float));
		memcpy(&(mEnds[16]), cur, 16 * sizeof(float));
		OpenMPD_CWrapper_update_HighLevel(pm, primitives, 2, mStarts, mEnds/*, GSPAT::MatrixAlignment::RowMajorAlignment*/);
		countFrames();
	}
	destroyContent(pm);
	return NULL;
}

void declareContent(OpenMPD_Context_Handler pm) {
	
	float p1_data[] = { -0.03f,-0.03f,0.12f,1,};   //haptic  point
	float p2_data[] = { 0.02f,0.02f,0.12f,1, };     //cardioid start point

	float origin1[] = { -0.03f,-0.03f,0.11f };    //  center of haptic circle
	float startPoint1[] = { -0.03f,-0.03f,0.12f };



	float* hapticCircle_data = createSampledArc(origin1, startPoint1, 2 * 3.14159265f, numSamplesHapticCircle);
	float* cardioid_data = createSampledCardioid(p2_data, 2 * 3.14159265f, 0.0006, 0.003, numSamplesCardioid);

	float a1_data[] = { 15000.0f}, a2_data[] = { 13000.0f};

	//Create descriptors
	pos1 = OpenMPD_CWrapper_createPositionsDescriptor(pm, p1_data, 1);
	pos2 = OpenMPD_CWrapper_createPositionsDescriptor(pm, p2_data, 1);
	hapticCirclePath = OpenMPD_CWrapper_createPositionsDescriptor(pm, hapticCircle_data, numSamplesHapticCircle);
	cardioidPath = OpenMPD_CWrapper_createPositionsDescriptor(pm, cardioid_data, numSamplesCardioid);
	amp1 = OpenMPD_CWrapper_createAmplitudesDescriptor(pm, a1_data, 1);
	amp2 = OpenMPD_CWrapper_createAmplitudesDescriptor(pm, a2_data, 1);
	//Create Primitives
	pri1 = OpenMPD_CWrapper_declarePrimitive(pm, pos1, amp1);
	pri2 = OpenMPD_CWrapper_declarePrimitive(pm, pos2, amp2);
	OpenMPD_CWrapper_commitUpdates(pm);
	OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri1, true);
	OpenMPD_CWrapper_setPrimitiveEnabled(pm, pri2, true);
	OpenMPD_CWrapper_commitUpdates(pm);
}

void destroyContent(OpenMPD_Context_Handler pm) {
	//Destroy primitives:
	OpenMPD_CWrapper_releasePrimitive(pm, pri1);
	OpenMPD_CWrapper_releasePrimitive(pm, pri2);
	OpenMPD_CWrapper_releasePositionsDescriptor(pm, pos1);
	OpenMPD_CWrapper_releasePositionsDescriptor(pm, pos2);
	OpenMPD_CWrapper_releasePositionsDescriptor(pm, cardioidPath);
	OpenMPD_CWrapper_releasePositionsDescriptor(pm, hapticCirclePath);
	OpenMPD_CWrapper_releaseAmplitudesDescriptor(pm, amp1);
	OpenMPD_CWrapper_releaseAmplitudesDescriptor(pm, amp2);
}

float* createSampledArc(float origin[3], float p0[3], float angleInRads, cl_uint numSamples) {
	//static float buffer[4 * 8];
	float* buffer = new float[numSamples * 4];
	float radius[] = { p0[0] - origin[0], p0[1] - origin[1], p0[2] - origin[2] };
	float curP[4];
	//Fill in all the samples:
	for (int s = 0; s < numSamples; s++) {
		float curAngle = (s * angleInRads) / numSamples;
		curP[0] = cosf(curAngle) * radius[0] - sinf(curAngle) * radius[1] + origin[0];
		curP[1] = sinf(curAngle) * radius[0] + cosf(curAngle) * radius[1] + origin[1];
		curP[2] = origin[2];
		curP[3] = 1;
		memcpy(&(buffer[4 * s]), curP, 4 * sizeof(float));
	}
	return buffer;
}

float* createSampledCardioid(float origin[3], float angleInRads, float scale, float displacement, cl_uint numSamples) {  //not defining 3D radius
	//static float buffer[4 * 8];
	float* buffer = new float[numSamples * 4];
	float curP[4];
	//Fill in all the samples:
	for (int s = 0; s < numSamples; s++) {
		float curAngle = (s * angleInRads) / numSamples;

		curP[1] = scale * (12 * sinf(curAngle) - 4 * sinf(curAngle * 3)) + origin[1];
		curP[2] = scale * (13 * cosf(curAngle) - 5 * cosf(curAngle * 2) - 2 * cosf(curAngle * 3) - cosf(curAngle * 4)) + origin[2] - displacement;
		curP[0] = origin[0];
		curP[3] = 1;
		memcpy(&(buffer[4 * s]), curP, 4 * sizeof(float));
	}
	return buffer;
}
