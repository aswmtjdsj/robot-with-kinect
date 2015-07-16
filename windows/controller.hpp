#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <kinect.h>
#include <Kinect.Face.h>
// #include <Kinect.VisualGestureBuilder.h>

#include <Windows.h>

#include <opencv2/opencv.hpp>

#include "chat.hpp"

#include <direct.h>

#include <mmsystem.h>

class Controller {

private:
	// Sensor
	IKinectSensor * pSensor; // kinect

	// Source
	IColorFrameSource * pColorSource; // color frame source
	IDepthFrameSource * pDepthSource; // depth frame source
	IBodyFrameSource * pBodySource; // body frame source
	IBodyIndexFrameSource * pBodyIndexSource; // body index frame source
	IFaceFrameSource * pFaceSource[BODY_COUNT];
	IHighDefinitionFaceFrameSource * pHDFaceSource[BODY_COUNT];

	// Reader
	IColorFrameReader * pColorReader; // color frame reader
	IDepthFrameReader * pDepthReader; // depth frame reader
	IBodyFrameReader * pBodyReader; // body frame reader
	IBodyIndexFrameReader * pBodyIndexReader; // body index frame reader
	IFaceFrameReader * pFaceReader[BODY_COUNT];
	IHighDefinitionFaceFrameReader * pHDFaceReader[BODY_COUNT];

	// Description
	IFrameDescription * pDepthDescription, // depth
		* pColorDescription; // color

	// detailed description of frames
	int pDepthWidth, pDepthHeight, // depth
		pColorWidth, pColorHeight; // color

	// frame size, buffer size
	unsigned int pDepthResolution,
		pDepthBufferSize,
		pColorBufferSize;

	unsigned short pDepthMin, pDepthMax;

	// bool: use color, depth, body, body index data or face data
	bool pUseColor, pUseDepth, pUseBody, pUseBodyIndex,
		pUseFace, pUseBodyFaceIntegration, pUseHDFace;

	// matrix for view
	cv::Mat pDepthBufferMat, pDepthMat, // cut mat later
		pColorResizedMat, pColorMat,
		pNormalMat, pVisualMat, pMapperMat;

	// raw color buffer
	RGBQUAD * pColorRGBX;

	// color image format
	ColorImageFormat pImageFormat;
	ColorSpacePoint * colorSpacePts;

	// for depth mapping
	ICoordinateMapper * pMapper;

	// for action recognition
	// store time-window data, including frames of depth, body, (or color?), result of detection
	deque <cv::Mat> pDepthQ;
	deque <IBody **> pBodyQ;
	deque <TIMESPAN> pTimestampQ;
	deque <bool> pOwnMethodResults, pLibraryMethodResults;
	int pMaxQSize;

	// for skeleton recording
	string pSkeletonsFileName[BODY_COUNT];
	fstream pSkeletonsFile[BODY_COUNT];
	uint pSkeletonFrameCount[BODY_COUNT];

	// for frame recording
	int pColorFrameCount, pDepthFrameCount, pBodyFrameCount, pBodyIndexFrameCount;
	std::string pDataDirectory;
	std::fstream pBodyFrameFile;
	std::string pBodyFrameFileName;

	// error information display
	bool error_debug;

	// visually debugging
	bool visual_debug;

	// handle training data
	int handle_training_data;

	// const face frame features
	static const DWORD cFaceFrameFeatures =
		FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

	// base face properties
	std::string tFaceProperties[FaceProperty::FaceProperty_Count];

	// face recording
	bool pFaceContinuous[BODY_COUNT];
	int pFaceCount[BODY_COUNT];
	static const int cFaceCountLimit = 100, cFaceCountBase = 25;
	std::fstream pFaceRecordFile[BODY_COUNT];

	// face body integration recording
	std::string pDataRecordDir[BODY_COUNT];

	// HD Face
	IFaceModelBuilder* pFaceModelBuilder[BODY_COUNT];
	bool pFaceModelProduced[BODY_COUNT];
	IFaceAlignment* pFaceAlignment[BODY_COUNT];
	IFaceModel* pFaceModel[BODY_COUNT];
	std::vector<std::vector<float>> pDeformations;
	// std::shared_ptr < > ;
public:

	Controller() : pSensor(nullptr),
		pColorSource(nullptr), pDepthSource(nullptr),
		pBodySource(nullptr), pBodyIndexSource(nullptr),
		pColorReader(nullptr), pDepthReader(nullptr),
		pBodyReader(nullptr), pBodyIndexReader(nullptr),
		pDepthDescription(nullptr), pColorDescription(nullptr),
		pDepthWidth(0), pDepthHeight(0),
		pColorWidth(0), pColorHeight(0),
		pDepthResolution(0), pDepthBufferSize(0),
		pColorBufferSize(0),
		pDepthMin(0), pDepthMax(0),
		pUseColor(true), pUseDepth(true),
		pUseBody(false), pUseBodyIndex(false), pUseFace(false),
		pUseBodyFaceIntegration(true), pUseHDFace(true),
		pColorRGBX(nullptr),
		pImageFormat(ColorImageFormat_None),
		colorSpacePts(nullptr),
		pMapper(nullptr),
		pMaxQSize(100),
		pColorFrameCount(0), pDepthFrameCount(0),
		pBodyFrameCount(0), pBodyIndexFrameCount(0),
		error_debug(false),
		visual_debug(true),
		handle_training_data(0) {

		pDepthQ.clear();
		pBodyQ.clear();
		pTimestampQ.clear();
		pOwnMethodResults.clear();
		pLibraryMethodResults.clear();


		// std::cout << pBodyFrameFileName << std::endl;
		if (handle_training_data == 0) {
			pDataDirectory = "./data/Integration/";
			//_mkdir(pDataDirectory.c_str());
			// pBodyFrameFileName = pDataDirectory + "/skeleton" + ".raw_data";
			// pBodyFrameFile.open(pBodyFrameFileName, std::fstream::out);
		}

		// for skeleton realtime recording
		for (uint count = 0; count < BODY_COUNT; count++) {
			pSkeletonsFileName[count] = "";
			pSkeletonFrameCount[count] = 0;
			if (pSkeletonsFile[count].is_open()) {
				pSkeletonsFile[count].close();
			}
		}
		// for face frame
		for (uint count = 0; count < BODY_COUNT; count++) {
			pFaceSource[count] = nullptr;
			pFaceReader[count] = nullptr;
		}
		// for face recording
		for (uint count = 0; count < BODY_COUNT; count++) {
			pFaceContinuous[count] = false;
			pFaceCount[count] = 0;
			if (pFaceRecordFile[count].is_open()) {
				pFaceRecordFile[count].close();
			}
		}

		// face properties
		tFaceProperties[0] = "Happy";
		tFaceProperties[1] = "Engaged";
		tFaceProperties[2] = "WearingGlasses";
		tFaceProperties[3] = "LeftEyeClosed";
		tFaceProperties[4] = "RightEyeClosed";
		tFaceProperties[5] = "MouthOpen";
		tFaceProperties[6] = "MouthMoved";
		tFaceProperties[7] = "LookingAway";

		// print info
		std::cout << "[INFO] Error (Debug) Info: " << (error_debug? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Visual (Debug) Info: " << (visual_debug? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Color: " << (pUseColor ? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Depth: " << (pUseDepth ? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Body: " << (pUseBody? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Body Index: " << (pUseBodyIndex? "Yes" : "No") << std::endl;
		std::cout << "[INFO] Face: " << (pUseFace? "Yes" : "No");
		std::cout << " (To Capture Face, Capture Body first)" << std::endl;
		std::cout << "[INFO] Body & Face Integration: " \
			<< (pUseBodyFaceIntegration? "Yes" : "No") << '\n';
		std::cout << "[INFO] HD Face: " << (pUseHDFace? "Yes" : "No");
		std::cout << " (To Capture Face, Capture Body first)" << std::endl;
	}

	~Controller() {
		std::cout << "[INFO] Garbage Collecting ... ";
		// body
		for (uint count = 0; count < BODY_COUNT; count++) {
			if (pSkeletonsFile[count].is_open()) {
				pSkeletonsFile[count].close();
			}
		}

		// collect data
		if (pBodyFrameFile.is_open()) {
			pBodyFrameFile.close();
		}

		// face
		for (uint count = 0; count < BODY_COUNT; count++) {
			SafeRelease(pFaceSource[count]);
			SafeRelease(pFaceReader[count]);
			if (pFaceRecordFile[count].is_open()) {
				pFaceRecordFile[count].close();
			}
		}

		if (pUseHDFace) {
			// hd face
			for (uint count = 0; count < BODY_COUNT; count++) {
				SafeRelease(pHDFaceSource[count]);
				SafeRelease(pHDFaceReader[count]);
				SafeRelease(pFaceModelBuilder[count]);
				SafeRelease(pFaceAlignment[count]);
				SafeRelease(pFaceModel[count]);
			}
		}

		SafeRelease(pColorSource);
		SafeRelease(pDepthSource);
		SafeRelease(pBodySource);
		SafeRelease(pBodyIndexSource);

		SafeRelease(pColorReader);
		SafeRelease(pDepthReader);
		SafeRelease(pBodyReader);
		SafeRelease(pBodyIndexReader);

		SafeRelease(pColorDescription);
		SafeRelease(pDepthDescription);

		SafeRelease(pMapper);

		delete [] pColorRGBX;

		if (pSensor) {
			pSensor->Close();
		}

		cv::destroyAllWindows();
		std::cout << " done.\n";
	}

	static string JointStr(const Joint & jt) {
		return to_string(jt.Position.X) + ", " +
			to_string(jt.Position.Y) + ", "
			+ to_string(jt.Position.Z);
	}

	static string JointOrientationStr(const JointOrientation & jt) {
		return to_string(jt.Orientation.w) + ", " + to_string(jt.Orientation.x) + ", "
			+ to_string(jt.Orientation.y) + ", " + to_string(jt.Orientation.z);
	}

	static inline void PrintJointInfo(const Joint & jt,
		const JointOrientation & jt_or,
		const string & jt_name) {
		std::cerr << "<" << jt_name <<
			"> position: (" << JointStr(jt) <<
			")\t orietation: (" << JointOrientationStr(jt_or) << ")\n";
	}

	static inline string GetTimestampStr() {
		std::time_t tTimestamp = std::time(nullptr);
		std::stringstream tFormattedStr;
		tFormattedStr << std::put_time(std::localtime(&tTimestamp), "%Y_%m_%d_%H_%M_%S");
		return tFormattedStr.str();
	}

	static inline string GetTimestampStr(const std::time_t tTimestamp) {
		std::stringstream tFormattedStr;
		tFormattedStr << std::put_time(std::localtime(&tTimestamp), "%Y_%m_%d_%H_%M_%S");
		return tFormattedStr.str();
	}

	inline int checkResult(const HRESULT & hr, const string & prompt, bool display = true) {
		if (FAILED(hr)) {
			if (error_debug && display) {
				_com_error err(hr);
				LPCTSTR errMsg = err.ErrorMessage();
				std::wcerr << "[ERROR] <" << errMsg << "> " << sConvToW(prompt) << '\n';
			}
			return -1;
		}
		else {
			if (false && display) {
				std::cout << "[SUCCESS] " << prompt << '\n';
			}
			return 0;
		}
	}

	int ColorInit() {

		// get color frame source
		if (checkResult(
			pSensor->get_ColorFrameSource(&pColorSource),
			"IKinectSensor::get_ColorFrameSource()") != 0) {
			return -1;
		}

		// open color frame reader
		if (checkResult(
			pColorSource->OpenReader(&pColorReader),
			"IColorFrameSource::OpenReader()") != 0) {
			return -1;
		}

		// get color frame description
		if (checkResult(
			pColorSource->get_FrameDescription(&pColorDescription),
			"IColorFrameSource::get_FrameDescription()") != 0) {
			return -1;
		}

		if (checkResult(
			pColorDescription->get_Width(&pColorWidth),
			"IFrameDescription::get_Width()") != 0) {
			return -1;
		}

		if (checkResult(
			pColorDescription->get_Height(&pColorHeight),
			"IFrameDescription::get_Height()") != 0) {
			return -1;
		} 

		pColorBufferSize = pColorWidth * pColorHeight * sizeof(RGBQUAD);
		std::cout << "[INFO] Color-> width: " << pColorWidth << " height: " << pColorHeight << std::endl;
		std::cout << "[INFO] Color Buffer Size: " << pColorBufferSize << std::endl;

		// init color buffer
		pColorRGBX = new RGBQUAD[pColorWidth * pColorHeight];
		pColorMat = cv::Mat(pColorHeight, pColorWidth, CV_8UC4);

		return 0;
	}

	int DepthInit() {

		// get depth frame source
		if (checkResult(
			pSensor->get_DepthFrameSource(&pDepthSource),
			"IKinectSensor::get_DepthFrameSource()") != 0) {
			return -1;
		}

		// open depth frame reader
		if (checkResult(
			pDepthSource->OpenReader(&pDepthReader),
			"IDepthFrameSource::OpenReader()") != 0) {
			return -1;
		}

		// get depth frame description
		if (checkResult(
			pDepthSource->get_FrameDescription(&pDepthDescription),
			"IDepthFrameSource::get_FrameDescription()") != 0) {
			return -1;
		}

		if (checkResult(
			pDepthDescription->get_Width(&pDepthWidth),
			"IFrameDescription::get_Width()") != 0) {
			return -1;
		}

		if (checkResult(
			pDepthDescription->get_Height(&pDepthHeight),
			"IFrameDescription::get_Height()") != 0) {
			return -1;
		}

		pDepthResolution = pDepthWidth * pDepthHeight;
		pDepthBufferSize = pDepthResolution * sizeof(unsigned short);

		std::cout << "[INFO] Depth-> width: " << pDepthWidth << " height: " << pDepthHeight << std::endl;
		std::cout << "[INFO] Depth Buffer Size: " << pDepthBufferSize << std::endl;

		if (checkResult(
			pDepthSource->get_DepthMinReliableDistance(&pDepthMin),
			"IDepthFrameSource::get_DepthMinReliableDistance()") != 0) {
			// 500
			return -1;
		}

		if (checkResult(
			pDepthSource->get_DepthMaxReliableDistance(&pDepthMax),
			"IDepthFrameSource::get_DepthMaxReliableDistance()") != 0) {
			// 4500
			return -1;
		}

		std::cout << "[INFO] Depth Range: <" << pDepthMin << "> - <" << pDepthMax << ">\n";

		pDepthBufferMat = cv::Mat(pDepthHeight, pDepthWidth, CV_16UC1);
		pDepthMat = cv::Mat(pDepthHeight, pDepthWidth, CV_8UC3);

		// depth - color mapper
		colorSpacePts = new ColorSpacePoint[pDepthHeight * pDepthWidth];

		return 0;
	}

	int BodyInit() {

		// get body frame source
		if (checkResult(
			pSensor->get_BodyFrameSource(&pBodySource),
			"IBodyFrameSource::get_BodyFrameSource()") != 0) {
			return -1;
		}

		// open body frame reader
		if (checkResult(
			pBodySource->OpenReader(&pBodyReader),
			"IBodyFrameReader::OpenReader()") != 0) {
			return -1;
		}

		return 0;
	}

	int BodyIndexInit() {

		// get body index frame source
		if (checkResult(
			pSensor->get_BodyIndexFrameSource(&pBodyIndexSource),
			"IBodyIndexFrameSource::get_BodyIndexFrameSource()") != 0) {
			return -1;
		}

		// open body index frame reader
		if (checkResult(
			pBodyIndexSource->OpenReader(&pBodyIndexReader),
			"IBodyIndexFrameReader::OpenReader()") != 0) {
			return -1;
		}

		return 0;
	}

	int FaceInit() {
		// get face frame source
		for (uint count = 0; count < BODY_COUNT; count++) {
			if(checkResult(
				CreateFaceFrameSource(pSensor, 0, cFaceFrameFeatures, &pFaceSource[count]),
				"CreateFaceFrameSouce()") != 0) {
				return -1;
			}
			if (checkResult(
				pFaceSource[count]->OpenReader(&pFaceReader[count]),
				"IFaceFrameSource:OpenReader()") != 0) {
				return -1;
			}
		}
		return 0;
	}

	int HDFaceInit() {
		// init deformation array
		pDeformations.resize(BODY_COUNT,
			std::vector<float>(FaceShapeDeformations::FaceShapeDeformations_Count));
		// should we do this using vector? or using smart pointer

		// HD face init
		for (uint count = 0; count < BODY_COUNT; count++) {
			// hd source
			if (checkResult(
				CreateHighDefinitionFaceFrameSource(pSensor, &pHDFaceSource[count]),
				"CreateHighDefinitionFaceFrameSource()") != 0) {
				return -1;
			}
			// hd reader
			if (checkResult(
				pHDFaceSource[count]->OpenReader(&pHDFaceReader[count]),
				"IHighDefinitionFaceFrameSource:OpenReader()") != 0) {
				return -1;
			}
			// hd Face Model Builder
			if (checkResult(
				pHDFaceSource[count]->OpenModelBuilder(
				FaceModelBuilderAttributes::FaceModelBuilderAttributes_None,
				&pFaceModelBuilder[count]),
				"IHighDefinitionFaceFrameSource:OpenModelBuilder()") != 0) {
				return -1;
			}
			// begin collecting face model data
			if (checkResult(
				pFaceModelBuilder[count]->BeginFaceDataCollection(),
				"IFaceModelBuilder:BeginFaceDataCollection()") != 0) {
				return -1;
			}
			// create face alignment
			if (checkResult(
				CreateFaceAlignment(&pFaceAlignment[count]),
				"CreateFaceAlignment()") != 0) {
				return -1;
			}
			// create face model
			CreateFaceModel(1.0f,
				FaceShapeDeformations::FaceShapeDeformations_Count,
				&pDeformations[count][0],
				&pFaceModel[count]);
		}
		return 0;
	}

	void VisualWindows() {
		if (pUseColor) {
			pColorResizedMat = cv::Mat(pColorMat.rows / 2, pColorMat.cols / 2, pColorMat.type());
			cv::resize(pColorMat, pColorResizedMat, cv::Size(pColorResizedMat.cols, pColorResizedMat.rows));
			cv::imshow("Color", pColorResizedMat);
		}
		if (pUseDepth) {
			cv::imshow("Depth", pDepthMat);
		}
	}

	int Init() {
		// get sensor device
		if (checkResult(
			GetDefaultKinectSensor(&pSensor),
			"GetDefaultKinectSensor") != 0) {
			return -1;
		}

		// open sensor
		if (checkResult(
			pSensor->Open(),
			"IKinectSensor::Open()") != 0) {
			return -1;
		}

		// multiple sources init
		if (pUseColor && ColorInit() != 0) {
			return -1;
		}
		if (pUseDepth && DepthInit() != 0) {
			return -1;
		}
		if (pUseBody && BodyInit() != 0) {
			return -1;
		}
		if (pUseBodyIndex && BodyIndexInit() != 0) {
			return -1;
		}
		if (pUseFace && pUseBody && FaceInit() != 0) {
			return -1;
		}
		if (pUseBodyFaceIntegration && !(BodyInit() == 0 && FaceInit() == 0)) {
			return -1;
		}
		if (pUseHDFace && HDFaceInit() != 0) {
			return -1;
		}

		// coordinate mapper
		if (checkResult(
			pSensor->get_CoordinateMapper(&pMapper),
			"ICoordinateMapper::get_CoordinateMapper()") != 0) {
			return -1;
		}

		// visual gesture
		// TODO, later

		return 0;
	}

	void OpenWindows() {
		if (pUseColor) {
			cv::namedWindow("Color");
		}
		if (pUseDepth) {
			cv::namedWindow("Depth");
		}
		/*if (pUseFace && pUseBody) {
			cv::namedWindow("Face");
		}*/
	}

	int CaptureColorFrame() {
		IColorFrame* pColorFrame = nullptr;
		bool tColorOK = true;
		if (checkResult(
			pColorReader->AcquireLatestFrame(&pColorFrame),
			"IColorFrameReader::AcquireLatestFrame()", false) != 0) {
			tColorOK = false;
			goto RELEASE_COLOR_FRAME;
			/*
			* Be sure you release the color frame whenever you're done
			* ( with myColorFrame.Release() )!
			* If the color frame you point to has data,
			* it will return E_PENDING until it's cleared,
			* which confused me for quite a bit.
			*/
		} // color AcquireLatestFrame

		if (checkResult(
			pColorFrame->get_RawColorImageFormat(&pImageFormat),
			"IColorFrame::get_RawColorImageFormat()", false) != 0) {
			tColorOK = false;
			goto RELEASE_COLOR_FRAME;
		}

		if (pImageFormat == ColorImageFormat_Bgra) {
			std::cout << "[INFO] ColorImageFormat: BGRA" << std::endl;
		}
		else {
			if (checkResult(
				pColorFrame->CopyConvertedFrameDataToArray(
				pColorBufferSize,
				reinterpret_cast<BYTE *>(pColorRGBX),
				ColorImageFormat_Bgra),
				"IColorFrame::CopyConvertedFrameDataToArray()", false) != 0) {

				tColorOK = false;
				goto RELEASE_COLOR_FRAME;
			}
			else {
				pColorMat = cv::Mat(pColorHeight, pColorWidth, CV_8UC4, pColorRGBX);
				// has_color = true;
			}
		}

	RELEASE_COLOR_FRAME:
		SafeRelease(pColorFrame);

		if (tColorOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	int CaptureDepthFrame() {
		IDepthFrame* pDepthFrame = nullptr;
		bool tDepthOK = true;
		if (checkResult(
			// get latest depth frame
			pDepthReader->AcquireLatestFrame(&pDepthFrame),
			"IDepthFrameReader::AcquireLatestFrame()", false) != 0) {
				tDepthOK = false;
				goto RELEASE_DEPTH_FRAME;
		}
		else {
			// store depth frame into buffer
			if (checkResult(
				pDepthFrame->AccessUnderlyingBuffer(
				&pDepthBufferSize,
				reinterpret_cast<UINT16**>(&pDepthBufferMat.data)),
				"IDepthFrame::AccessUnderlyingBuffer()", false) != 0) {
				tDepthOK = false;
				goto RELEASE_DEPTH_FRAME;
			}

			/*// store depth data in queue
			cv::Mat depthCopy = bufferDepthMat.clone();
			depthQ.push_back(depthCopy);
			if (depthQ.size() > max_q_size) {
			depthQ.pop_front();
			}*/

			if (visual_debug) {
				cv::Mat tDepthMat = cv::Mat(pDepthHeight, pDepthWidth, CV_8UC1);
				pDepthBufferMat.convertTo(tDepthMat, CV_8U, 255.0f / 4500.0f, .0f); // -255.0f / 4500.0f, 255.0f); // 
				cv::cvtColor(tDepthMat, pDepthMat, CV_GRAY2BGR);
				// cv::imwrite(pDataDirectory + "/" + to_string(pDepthFrameCount) + ".png", pDepthMat);
			}
		}

	RELEASE_DEPTH_FRAME:
		SafeRelease(pDepthFrame);

		if (tDepthOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	int CaptureBodyFrame() {
		IBodyFrame* pBodyFrame = nullptr;
		bool tBodyOK = true;
		// for skeleton
		IBody ** tBodies = new IBody *[BODY_COUNT];
		for (int i = 0; i < BODY_COUNT; i++) {
			tBodies[i] = nullptr;
		}

		if (checkResult(
			pBodyReader->AcquireLatestFrame(&pBodyFrame),
			"IBodyFrameReader::AcquireLatestFrame()") != 0) {
			tBodyOK = false;
			goto RELEASE_BODY_FRAME;
		}
		else {
			int body_count = 0;
			if (checkResult(
				pBodySource->get_BodyCount(&body_count),
				"IBodyFrameSource::get_BodyCount()") != 0) {
				tBodyOK = false;
				goto RELEASE_BODY_FRAME;
			}
			else {
				if (checkResult(
					pBodyFrame->GetAndRefreshBodyData(body_count, tBodies),
					"IBodyFrame::GetAndRefreshBodyData()") != 0) {
					tBodyOK = false;
					goto RELEASE_BODY_FRAME;
				}
				else {
					/*// store body data in queue
					pBodyQ.push_back(pBodies);
					if (pBodyQ.size() > pMaxQSize) {
					IBody ** toRelease = pBodyQ.front();
					for (int i = 0; i < BODY_COUNT; i++) {
					SafeRelease(toRelease[i]);
					}
					delete [] toRelease;
					pBodyQ.pop_front();
					// store timestamp in queue
					pTimestampQ.push_back(clock());
					if (pTimestampQ.size() > pMaxQSize) {
					pTimestampQ.pop_front();
					}
					std::cout << "[INFO] depth: " << pDepthQ.size() << " body: " << pBodyQ.size()
					<< " result1: " << result.size() << " result2: " << result_comp.size()
					<< " timestamp: " << timestampQ.size() << " first stamp was: " << (clock() - timestampQ.front()) << " ms ago" << endl;
					}*/

					/*pBodyQ.push_back(pBodies);

					TIMESPAN tTimeSpan;
					if (checkResult(
						pBodyFrame->get_RelativeTime(&tTimeSpan),
						"IBodyFrame::get_RelativeTime()") != 0) {
						// nothing
					}
					pTimestampQ.push_back(tTimeSpan);*/

					for (uint i = 0; i < BODY_COUNT; i++) {
						BOOLEAN tIsTracked = false;
						if (checkResult(
							tBodies[i]->get_IsTracked(&tIsTracked),
							"IBody::get_IsTracked()") != 0) {
							// nothing
						}

						// std::cout << i << ' ' << tIsTracked << std::endl;
						if (tIsTracked) { // the body is tracked
							// record skeleton data
							// get joint position
							Joint pJoints[JointType::JointType_Count];
							if (checkResult(
								tBodies[i]->GetJoints(JointType::JointType_Count, pJoints),
								"IBody::GetJoints()") == 0) {
								// nothing
							}

							// get joint orientation
							JointOrientation pOrientations[JointType::JointType_Count];
							if (checkResult(
								tBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations),
								"IBody::GetJointOrientations()") == 0) {
								// nothing
							}

							// recording part
							if (!pSkeletonsFile[i].is_open()) {
								pSkeletonsFileName[i] = "./data/Skeleton/" +  \
									GetTimestampStr() + "_skeleton_" + to_string(i) + \
									".raw_data";
								pSkeletonsFile[i].open(pSkeletonsFileName[i], std::fstream::out);
							}
							// pSkeletonsQ[i].push_back(tBodies[i]);
							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& tJoint = pJoints[j];
								const JointOrientation& tOrientation = pOrientations[j];
								pSkeletonsFile[i] << JointStr(tJoint) << ", " <<
									JointOrientationStr(tOrientation) << ", " <<
									(tJoint.TrackingState != TrackingState_NotTracked) << ';';
							}
							pSkeletonsFile[i] << '\n';
							//

							/*// IMPORTANT!!!
							// Set TrackingID to Detect Gesture
							UINT64 trackingId = _UI64_MAX;
							hResult = pBodies[i]->get_TrackingId(&trackingId);
							if (checkResult(hResult, "IBody::get_TrackingId()") == 0) {
							hResult = vgb_source[i]->put_TrackingId(trackingId);
							if (checkResult(hResult, "IVisualGestureBuilderFrameSource::put_TrackingId()") == 0) {
							// nothing
							}
							} */

							if (pUseFace) {
								// Set TrackingID to Detect Face
								UINT64 tTrackingId = _UI64_MAX;
								if (checkResult(
									tBodies[i]->get_TrackingId(&tTrackingId),
									"IBody:get_TrackingId()") != 0) {
									// do nothing
								}
								cout << tTrackingId << endl;
								if (checkResult(
									pFaceSource[i]->put_TrackingId(tTrackingId),
									"IFaceFrameSource:put_TrackingId") != 0) {
									// do nothing
								}
							}

							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& tJoint = pJoints[j];
								const JointOrientation& tOrientation = pOrientations[j];
								// draw all joints
								if (tJoint.TrackingState != TrackingState_NotTracked) {
									// re-mapping the joint position
									CameraSpacePoint tCameraPt = tJoint.Position;
									// std::cout << i << ' ' << j << ' ' << JointStr(tJoint) << std::endl;
									DepthSpacePoint tDepthPt;
									int tNumPts = 1;
									if (checkResult(
										pMapper->MapCameraPointsToDepthSpace(
										tNumPts, &tCameraPt, tNumPts, &tDepthPt),
										"ICoordinateMapper::MapCameraPointsToColorSpace()") == 0) {
										// nothing
									}

									tDepthPt.X = floor(tDepthPt.X);
									tDepthPt.Y = floor(tDepthPt.Y);
									if (tDepthPt.X >= 0 && tDepthPt.X < pDepthWidth
										&& tDepthPt.Y >= 0 && tDepthPt.Y < pDepthHeight) {
										cv::Point tToDraw;
										tToDraw.x = tDepthPt.X;
										tToDraw.y = tDepthPt.Y;
										int tRadius = 5;
										if (tJoint.TrackingState == TrackingState_Inferred) {
											circle(pDepthMat, tToDraw, tRadius,
												cv::Scalar(0, 0, 255));
										}
										else {
											circle(pDepthMat, tToDraw, tRadius,
												cv::Scalar(0, 0, 255), -1);
										}
										cv::Point tTextCorner = tToDraw;
										tTextCorner.x += tRadius;
										tTextCorner.y += tRadius;
										putText(pDepthMat, to_string(j), tTextCorner,
											cv::FONT_HERSHEY_SIMPLEX, 0.5,
											cv::Scalar(0, 255, 255), 1);
									} // depth point within boundary
								} // if tracking
							} // for joints
						} // tIsTracked = true
						else {
							// not tracked
							if (pSkeletonsFile[i].is_open()) {
								pSkeletonsFileName[i] = "";
								pSkeletonsFile[i].close();
							}
						} // tIsTracked = false
					} // for body_count
				}
			}
		}

	RELEASE_BODY_FRAME:
		SafeRelease(pBodyFrame);

		for (int i = 0; i < BODY_COUNT; i++) {
			SafeRelease(tBodies[i]);
		}
		delete[] tBodies;

		if (tBodyOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	int CaptureFaceFrame() {
		// to capture face, capture body first
		bool tFaceOK = true;
		IFaceFrame * tFaceFrame[BODY_COUNT];
		IFaceFrameResult * tFaceResult = nullptr;
		IBodyFrameReference * tBodyCorrespondence = nullptr;
		IBodyFrame * tBodyFrameCorrespondence = nullptr;
		int body_count = BODY_COUNT;
		IBody ** tBodies = nullptr;
		tBodies = new IBody *[BODY_COUNT];
		for (int i = 0; i < BODY_COUNT; i++) {
			tFaceFrame[i] = nullptr;
			tBodies[i] = nullptr;
		}

		for (int i = 0; i < BODY_COUNT; i++) {
			if (checkResult(
				pFaceReader[i]->AcquireLatestFrame(&tFaceFrame[i]),
				"IFaceFrameReader::AcquireLatestFrame()") != 0) {
				tFaceOK = false;
				goto RELEASE_FACE_FRAME;
			}
			else {
				BOOLEAN tFaceTracked = true;
				if (checkResult(
					tFaceFrame[i]->get_IsTrackingIdValid(&tFaceTracked),
					"IFaceFrame:get_IsTrackingIdValid()") != 0) {
					tFaceOK = false;
					goto RELEASE_FACE_FRAME;
				}
				else {
					if (tFaceTracked) {
						// make correspondence between face and body
						/*if (checkResult(
							tFaceFrame[i]->get_BodyFrameReference(&tBodyCorrespondence),
							"IFaceFrame:get_BodyFrameReference()") != 0) {
							tFaceOK = false;
							goto RELEASE_FACE_FRAME;
						}
						else {
							if (checkResult(
								tBodyCorrespondence->AcquireFrame(&tBodyFrameCorrespondence),
								"IBodyFrameReference:AcquireFrame()") != 0) {
								tFaceOK = false;
								goto RELEASE_FACE_FRAME;
							}
							else {
								if (checkResult(
									tBodyFrameCorrespondence->GetAndRefreshBodyData(body_count, tBodies),
									"IBodyFrame:GetAndRefreshBodyData()") != 0) {
									tFaceOK = false;
									goto RELEASE_FACE_FRAME;
								}
								else {
									for (int j = 0; j < body_count; j++) {
										BOOLEAN tBodyTracked = false;
										if (checkResult(
											tBodies[j]->get_IsTracked(&tBodyTracked),
											"IBody::get_IsTracked()") != 0) {
											// nothing
										}
										if (tBodyTracked) {
											tFaceFrame[i]->
											break;
										}
									}
								}
							}
						}*/
						// face detected
						if (checkResult(
							tFaceFrame[i]->get_FaceFrameResult(&tFaceResult),
							"IFaceFrame:get_FaceFrameResult") != 0) {
							tFaceOK = false;
							goto RELEASE_FACE_FRAME;
						}
						else {
							RectI tBoundingBox;
							if (checkResult(
								tFaceResult->get_FaceBoundingBoxInColorSpace(&tBoundingBox),
								"IFaceFrameResult:get_FaceBoundingBoxInColorSpace()") != 0) {
								/*tFaceOK = false;
								goto RELEASE_FACE_FRAME;*/
								// nothing
							}
							else {
								// tBoundingBox.Left;
								auto tSubRegion =
									cv::Rect(tBoundingBox.Left,
									tBoundingBox.Top,
									tBoundingBox.Right - tBoundingBox.Left,
									tBoundingBox.Bottom - tBoundingBox.Top);
								if (tSubRegion.width > 0 && tSubRegion.height > 0) {
									/*std::cout << tBoundingBox.Top << ' ';
									std::cout << tBoundingBox.Bottom << '\t';
									std::cout << tBoundingBox.Left << ' ';
									std::cout << tBoundingBox.Right << std::endl;*/
									if (!pFaceContinuous[i]) {
										cv::Mat temp;
										pColorMat(tSubRegion).copyTo(temp);
										string temp_file = "./data/Face/" + GetTimestampStr() + "_" + to_string(i) + "_face.png";
										cv::imwrite(temp_file, temp);
										/*Vector4 tFaceRotation;
										if (checkResult(
											tFaceResult->get_FaceRotationQuaternion(&tFaceRotation),
											"IFaceFrameResult:get_FaceRotationQuaternion()") != 0) {
											// nothing
										}
										else {
										}*/
									}
									// cv::imshow("Face", temp);
									cv::rectangle(pColorMat, tSubRegion,
										cv::Scalar(0., 0., 255.), 2);
									PointF tFacePoints[FacePointType::FacePointType_Count];
									if (checkResult(
										tFaceResult->GetFacePointsInColorSpace(
										FacePointType::FacePointType_Count,
										tFacePoints),
										"IFaceFrameResult:GetFacePointsInColorSpace()") != 0) {
										// nothing
									}
									else {
										for (int count = 0; count < FacePointType::FacePointType_Count; count++) {
											cv::circle(pColorMat, 
												cv::Point(
												static_cast<int>(tFacePoints[count].X),
												static_cast<int>(tFacePoints[count].Y)),
												2, cv::Scalar(0., 0., 255.), 2);
										} // for face point count
									} // get face points in color space
								} // bounding box valid
							} // get face bounding box
						} // get face frame result
						pFaceContinuous[i] = true;
					} // face tracked
					else {
						// face not tracked
						pFaceContinuous[i] = false;
					}
				} // get is tracking id valid
			} // acquire latest face frame
		} // for body count

	RELEASE_FACE_FRAME:
		SafeRelease(tBodyCorrespondence);
		SafeRelease(tBodyFrameCorrespondence);
		for (int i = 0; i < BODY_COUNT; i++) {
			SafeRelease(tFaceFrame[i]);
			SafeRelease(tBodies[i]);
		}
		if (tBodies != nullptr) {
			delete[] tBodies;
		}
		if (tFaceOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	int CaptureBodyAndFaceFrame() {

		IBodyFrame* tBodyFrame = nullptr;
		bool tBodyOK = true;
		IBody * tBodies[BODY_COUNT] = { nullptr };
		
		bool tFaceOK = true;
		IFaceFrame * tFaceFrame = nullptr;
		IFaceFrameResult * tFaceResult = nullptr;

		// for body frame
		// acquire latest body frame
		if (checkResult(
			pBodyReader->AcquireLatestFrame(&tBodyFrame),
			"IBodyFrameReader::AcquireLatestFrame()") != 0) {
			tBodyOK = false;
			goto RELEASE_BODY_AND_FACE_FRAME;
		}
		else {

			int tBodyCount = 0;
			// get count of body, but it is manually set by kinect
			// maybe updated in the future
			if (checkResult(
				pBodySource->get_BodyCount(&tBodyCount),
				"IBodyFrameSource::get_BodyCount()") != 0) {
				tBodyOK = false;
				goto RELEASE_BODY_AND_FACE_FRAME;
			}
			else {
				// get body data
				if (checkResult(
					tBodyFrame->GetAndRefreshBodyData(BODY_COUNT, tBodies),
					"IBodyFrame::GetAndRefreshBodyData()") != 0) {
					tBodyOK = false;
					goto RELEASE_BODY_AND_FACE_FRAME;
				}
				else {

					for (uint count = 0; count < BODY_COUNT; count++) {
						BOOLEAN tIsTracked = false;
						// check body[count] tracked or not
						if (checkResult(
							tBodies[count]->get_IsTracked(&tIsTracked),
							"IBody::get_IsTracked()") != 0) {
							// nothing
						}
						// body[count] is tracked
						if (tIsTracked) {

							// Set TrackingID to detect face
							UINT64 tTrackingId = _UI64_MAX;
							if (checkResult(
								tBodies[count]->get_TrackingId(&tTrackingId),
								"IBody:get_TrackingId()") != 0) {
								// do nothing
							}
							// this will initialize and allocate some memory
							if (checkResult(
								pFaceSource[count]->put_TrackingId(tTrackingId),
								"IFaceFrameSource:put_TrackingId") != 0) {
								// do nothing
							}
							if (pUseHDFace) {
								if (checkResult(
									pHDFaceSource[count]->put_TrackingId(tTrackingId),
									"IFaceFrameSource:put_TrackingId") != 0) {
									// do nothing
								}
							}
							// record skeleton data
							// get joint position
							Joint tJoints[JointType::JointType_Count];
							if (checkResult(
								tBodies[count]->GetJoints(
								JointType::JointType_Count, tJoints),
								"IBody::GetJoints()") == 0) {
								// nothing
							}
							// get joint orientation
							JointOrientation tOrientations[JointType::JointType_Count];
							if (checkResult(
								tBodies[count]->GetJointOrientations(
								JointType::JointType_Count, tOrientations),
								"IBody::GetJointOrientations()") == 0) {
								// nothing
							}

							// recording part
							if (!pSkeletonsFile[count].is_open()) {
								// create new dir for integration of body and face
								pDataRecordDir[count] = \
									pDataDirectory + GetTimestampStr() + \
									+ "_" + to_string(count) + "/";
								_mkdir(pDataRecordDir[count].c_str());
								pSkeletonsFileName[count] = pDataRecordDir[count] +  \
									"skeleton.raw_data";
								pSkeletonsFile[count].open(pSkeletonsFileName[count], std::fstream::out);
								pSkeletonFrameCount[count] = 0;
							}
							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& tJoint = tJoints[j];
								const JointOrientation& tOrientation = tOrientations[j];
								pSkeletonsFile[count] << JointStr(tJoint) << ", " <<
									JointOrientationStr(tOrientation) << ", " <<
									(tJoint.TrackingState != TrackingState_NotTracked)
									<< ';';
							}
							pSkeletonsFile[count] << '\n';
							pSkeletonFrameCount[count]++;
							//
							// draw joints on depth map
							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& tJoint = tJoints[j];
								const JointOrientation& tOrientation = tOrientations[j];
								// draw all joints
								if (tJoint.TrackingState != TrackingState_NotTracked) {
									// re-mapping the joint position
									CameraSpacePoint tCameraPt = tJoint.Position;
									DepthSpacePoint tDepthPt;
									int tNumPts = 1;
									if (checkResult(
										pMapper->MapCameraPointsToDepthSpace(
										tNumPts, &tCameraPt, tNumPts, &tDepthPt),
										"ICoordinateMapper::MapCameraPointsToColorSpace()") == 0) {
										// nothing
									}

									int tX = floor(tDepthPt.X);
									int tY = floor(tDepthPt.Y);
									if (tX >= 0 && tX < pDepthWidth
										&& tY >= 0 && tY < pDepthHeight) {
										cv::Point tToDraw;
										tToDraw.x = tX;
										tToDraw.y = tY;
										int tRadius = 5;
										if (tJoint.TrackingState == TrackingState_Inferred) {
											circle(pDepthMat, tToDraw, tRadius,
												cv::Scalar(0, 0, 255));
										}
										else {
											circle(pDepthMat, tToDraw, tRadius,
												cv::Scalar(0, 0, 255), -1);
										}
										cv::Point tTextCorner = tToDraw;
										tTextCorner.x += tRadius;
										tTextCorner.y += tRadius;
										putText(pDepthMat, to_string(j), tTextCorner,
											cv::FONT_HERSHEY_SIMPLEX, 0.5,
											cv::Scalar(0, 255, 255), 1);
									} // depth point within boundary
								} // if tracking
							} // for joints
						} // if body tracked
						else {
							// not tracked
							if (pSkeletonsFile[count].is_open()) {
								pSkeletonsFileName[count] = "";
								pSkeletonsFile[count].close();
								pSkeletonFrameCount[count] = 0;
							}
						} // tIsTracked = false
					} // for BODY_COUNT
				} // get body data
			} // get count of body
		} // acquire latest body frame

		// for face frame
		for (uint count = 0; count < BODY_COUNT; count++) {
			// acquire latest face frame
			if (checkResult(
				pFaceReader[count]->AcquireLatestFrame(&tFaceFrame),
				"IFaceFrameReader::AcquireLatestFrame()") != 0) {
				tFaceOK = false;
				goto RELEASE_BODY_AND_FACE_FRAME;
			}
			else {
				BOOLEAN tFaceTracked = true;
				// get tracking id valid and detect face tracked or not
				if (checkResult(
					tFaceFrame->get_IsTrackingIdValid(&tFaceTracked),
					"IFaceFrame:get_IsTrackingIdValid()") != 0) {
					tFaceOK = false;
					goto RELEASE_BODY_AND_FACE_FRAME;
				}
				else {
					if (tFaceTracked) {
						// get face result
						if (checkResult(
							tFaceFrame->get_FaceFrameResult(&tFaceResult),
							"IFaceFrame:get_FaceFrameResult") != 0) {
							tFaceOK = false;
							goto RELEASE_BODY_AND_FACE_FRAME;;
						}
						else {
							// bounding box of face
							RectI tBoundingBox;
							PointF tFacePoints[FacePointType::FacePointType_Count];
							Vector4 tQuaternion;
							DetectionResult tFaceProperty[FaceProperty::FaceProperty_Count];

							bool tBB = false, tFP = false, tQ = false, tProp = false;

							// get face bouding box
							if (checkResult(
								tFaceResult->get_FaceBoundingBoxInColorSpace(
								&tBoundingBox),
								"IFaceFrameResult:get_FaceBoundingBoxInColorSpace()") != 0) {
								// nothing
								// check later
							}

							int face_width = tBoundingBox.Right - tBoundingBox.Left,
								face_height = tBoundingBox.Bottom - tBoundingBox.Top;

							// valid bounding box
							if (face_width > 0 && face_height > 0) {
								tBB = true;
							}
							// extend face area
							int ext_width = face_width * 1.5,
								ext_height = face_height * 1.6;
							int left_offset = (ext_width - face_width) >> 1,
								right_offset = (ext_width - face_width) >> 1,
								up_offset = (ext_height - face_height) * 0.8,
								down_offset = (ext_height - face_height) * 0.2;
							if (tBoundingBox.Left - left_offset < 0) {
								left_offset = tBoundingBox.Left;
							}
							if (tBoundingBox.Right + right_offset >= pColorMat.cols) {
								right_offset = pColorMat.cols - 1 - tBoundingBox.Right;
							}
							if (tBoundingBox.Top - up_offset < 0) {
								up_offset = tBoundingBox.Top;
							}
							if (tBoundingBox.Bottom + down_offset >= pColorMat.rows) {
								down_offset = pColorMat.rows - 1 - tBoundingBox.Bottom;
							}
							auto tSubRegion =
								cv::Rect(tBoundingBox.Left - left_offset,
								tBoundingBox.Top - up_offset,
								face_width + left_offset + right_offset,
								face_height + up_offset + down_offset);

							// get face points in color space
							if (checkResult(
								tFaceResult->GetFacePointsInColorSpace(
								FacePointType::FacePointType_Count,
								tFacePoints),
								"IFaceFrameResult:GetFacePointsInColorSpace()") != 0) {
								// nothing
							}
							else {
								tFP = true;
								for (uint count = 0; count < FacePointType::FacePointType_Count; count++) {
									if (tFacePoints[count].X == 0 && tFacePoints[count].Y == 0) {
										tFP = false;
									}
								}
							}

							// get rotation quaternion of face
							if (checkResult(
								tFaceResult->get_FaceRotationQuaternion(&tQuaternion),
								"IFaceFrameResult:get_FaceRotationQuaternion()") != 0) {
								// nothing
							}
							else {
								tQ = true;
							}

							// get
							// tFaceResult->get_FaceFrameFeatures
							if (checkResult(
								tFaceResult->GetFaceProperties(
								FaceProperty::FaceProperty_Count, tFaceProperty),
								"IFaceFrameResult:GetFaceProperties()") != 0) {
								// nothing
							}
							else {
								tProp = true;
							}

							// save face in record
							if (tBB && tFP && tQ && tProp &&
								pFaceCount[count] % cFaceCountBase == 0) {
								string tStamp = GetTimestampStr();
								string tFaceImgFile = pDataRecordDir[count] + \
									"face_" + tStamp + ".jpg";
								string tFacePropFile = pDataRecordDir[count] + \
									"face_" + tStamp + ".prop_data";
								std::fstream tFaceProp = \
									std::fstream(tFacePropFile, std::fstream::out);
								// save bounding box
								cv::Mat tFaceImg;
								pColorMat(tSubRegion).copyTo(tFaceImg);
								cv::imwrite(tFaceImgFile, tFaceImg);
								tFaceProp << "[FaceJoint]: ";
								for (uint sub = 0; sub < FacePointType::FacePointType_Count; sub ++) {
									tFaceProp
										<< static_cast<int>(tFacePoints[sub].X) - tBoundingBox.Left + left_offset
										<< ','
										<< static_cast<int>(tFacePoints[sub].Y) - tBoundingBox.Top + up_offset
										<< ";";
								}
								tFaceProp << '\n';
								tFaceProp << "[Quaternion]: ";
								tFaceProp << tQuaternion.w << ','
									<< tQuaternion.x << ','
									<< tQuaternion.y << ','
									<< tQuaternion.z << '\n';
								tFaceProp << "[Property]: ";
								for (uint sub = 0; sub < FaceProperty::FaceProperty_Count; sub++){
									tFaceProp << tFaceProperties[sub] << ":";
									switch (tFaceProperty[sub]){
									case DetectionResult::DetectionResult_Unknown:
										tFaceProp << "Unknown";
										break;
									case DetectionResult::DetectionResult_Yes:
										tFaceProp << "Yes";
										break;
									case DetectionResult::DetectionResult_No:
										tFaceProp << "No";
										break;
									case DetectionResult::DetectionResult_Maybe:
										tFaceProp << "Maybe";
										break;
									default:
										break;
									}
									tFaceProp << ';';
								} tFaceProp << '\n';
								tFaceProp << "[Reference]: " << 
									pSkeletonFrameCount[count] << '\n';
								tFaceProp.close();
							}
							if (tBB) {
								cv::rectangle(pColorMat, tSubRegion,
									cv::Scalar(0., 0., 255.), 2);
							} // draw bounding box

							if (tFP) {
								for (uint count = 0; count < FacePointType::FacePointType_Count; count++) {
									cv::circle(pColorMat,
										cv::Point(
										static_cast<int>(tFacePoints[count].X),
										static_cast<int>(tFacePoints[count].Y)),
										2, cv::Scalar(0., 0., 255.), 2);
								}
							} // draw face points

							// update face counting
							if (tBB && tFP && tQ && tProp) {
								pFaceCount[count] ++;
								if (pFaceCount[count] == cFaceCountLimit) {
									pFaceCount[count] = 0;
								}
							}
						} // face frame result
						SafeRelease(tFaceResult);
					} // face tracked
				} // get tracking id valid
			} // acquire latest frame
			SafeRelease(tFaceFrame);
		} // for body count

		// HD Face Frame
		if (pUseHDFace) {
			bool tHDFaceOK = true;
			IHighDefinitionFaceFrame * tHDFaceFrame = nullptr;

			for (uint count = 0; count < BODY_COUNT; count++) {
				// acquire latest HD face frame
				if (checkResult(
					pHDFaceReader[count]->AcquireLatestFrame(&tHDFaceFrame),
					"IHighDefinitionFaceFrameReader::AcquireLatestFrame()") != 0) {
					tHDFaceOK = false;
					goto RELEASE_HD_FACE_FRAME;
				}
				else {
					// TODO
					BOOLEAN tHDFaceTrackingIdValid = true, tHDFaceTracked = true;
					// get tracking id valid or not
					if (checkResult(
						tHDFaceFrame->get_IsTrackingIdValid(&tHDFaceTrackingIdValid),
						"IHighDefinitionFaceFrame:get_IsTrackingIdValid()") != 0) {
						tHDFaceOK = false;
						goto RELEASE_HD_FACE_FRAME;
					}
					else {
						// get HD face tracked or not
						if (checkResult(
							tHDFaceFrame->get_IsFaceTracked(&tHDFaceTracked),
							"IHighDefinitionFaceFrame:get_IsFaceTracked()") != 0) {
							tHDFaceOK = false;
							goto RELEASE_HD_FACE_FRAME;
						}
						else {
							// std::cout << "HD tracked: " << (int) tHDFaceTracked << std::endl;
							if (tHDFaceTracked) {
								// HD face tracked
								// get HD face result
								// IFaceModel
								//tHDFaceFrame->get_FaceModel()
								// CreateFaceModel()
							}
							else {
							} // HD face not tracked
						} // get HD face tracked or not
					} // get tracking id valid
				} // acquire latest frame OK
				SafeRelease(tHDFaceFrame);
			} // for BODY_COUNT

		RELEASE_HD_FACE_FRAME:
			SafeRelease(tHDFaceFrame);
			if (tHDFaceOK == false) {
				goto RELEASE_BODY_AND_FACE_FRAME;
			}
		}

	RELEASE_BODY_AND_FACE_FRAME:
		// body
		SafeRelease(tBodyFrame);
		for (uint count = 0; count < BODY_COUNT; count++) {
			SafeRelease(tBodies[count]);
		}

		// face
		SafeRelease(tFaceResult);
		SafeRelease(tFaceFrame);

		if (tBodyOK == false || tFaceOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	static void HandleLabeledData(const string & mSkeletonFileName,
		const string & mLabelFileName,
		const string & mTrainingDataFileName) {
		std::fstream mSkeletonFile = std::fstream(mSkeletonFileName, std::fstream::in),
			mLabelFile = std::fstream(mLabelFileName, std::fstream::in),
			mTrainingDataFile = std::fstream(mTrainingDataFileName, std::fstream::out);

		vector < vector <double> > raw_data;
		vector <double> raw_item;

		string tInput[3];
		while (getline(mSkeletonFile, tInput[0])) {
			getline(mSkeletonFile, tInput[1]);
			// cout << 0 << ' ' << tInput[0] << endl;
			// cout << 1 << ' ' << tInput[1] << endl;
			std::stringstream ss(tInput[0]);
			int tEventId;
			ss >> tEventId;

			if (tInput[1] != "#-1") {
				std::cout << "# " << tEventId << std::endl;
				getline(mSkeletonFile, tInput[2]);
				// cout << 2 << ' ' << tInput[2] << endl;
				ss = stringstream(tInput[2]);
				string sub;
				int tCnt = 0;
				while (std::getline(ss, sub, ';')) {
					tCnt ++;
					// std::cout << sub << std::endl;
					stringstream sss(sub);
					double pos[3];
					string ssub;
					for (int _ = 0; _ < 3; _++) {
						std::getline(sss, ssub, ',');
						pos[_] = std::stod(ssub);
						raw_item.push_back(pos[_]);
					}
				}
				/*for (auto c : raw_item) {
					std::cout << c << ' ';
				}*/
				raw_data.push_back(std::move(raw_item));
				// system("pause");
			}
			else {
				raw_data.push_back(std::move(raw_item));
			}
		}
		std::cout << "raw data size: " << raw_data.size() << std::endl;
		bool tLabel;
		int tStart, tEnd;
		while (mLabelFile >> tLabel >> tStart >> tEnd) {
			vector < vector<double> > tTrainingData;
			for (int i = tStart; i <= tEnd; i++) {
				if (raw_data[i].size() != 0) {
					tTrainingData.push_back(raw_data[i]);
				}
			}
			std::cout << tLabel << ' ' << tStart << ' ' << tEnd << ' ' << tTrainingData.size() << std::endl;

			mTrainingDataFile << tLabel << ' '
				<< tTrainingData[0].size() << ' ' << tTrainingData.size() << '\n';
			for (uint j = 0; j < tTrainingData[0].size(); j++) {
				for (uint i = 0; i < tTrainingData.size(); i++) {
					mTrainingDataFile << tTrainingData[i][j] << ' ';
				}
				mTrainingDataFile << '\n';
			}
		}

		mSkeletonFile.close();
		mLabelFile.close();
		mTrainingDataFile.close();
	}

	static void ClusterTrainingData(const string & mPreHandledFileName,
		const string & mTrainingDataFileName) {
		std::fstream mPreHandledFile= std::fstream(mPreHandledFileName, std::fstream::in),
			mTrainingDataFile = std::fstream(mTrainingDataFileName, std::fstream::out);

		vector < vector < vector < vector <double> > > > tDataSet[2]; // 3 * tJoints * nFrames * nSamples
		bool label;
		int row, col;
		while (mPreHandledFile >> label >> row >> col) {
			vector < vector < vector <double> > > tSample(col);
			for (int i = 0; i < col; i++) {
				tSample[i].resize(row / 3);
				for (int j = 0; j < row / 3; j++) {
					tSample[i][j].resize(3);
				}
			}
			for (int i = 0; i < row; i++) {
				int tJoint = i / 3, tCoor = i % 3;
				for (int j = 0; j < col; j++) {
					mPreHandledFile >> tSample[j][tJoint][tCoor];
				}
			}
			/*for (int i = 0; i < tSample.size(); i++) {
				for (int j = 0; j < tSample[i].size(); j++) {
					std::cout << "(";
					for (int k = 0; k < 3; k++) {
						std::cout << tSample[i][j][k] << "; ";
					} 
					std::cout << ") ";
				}
				std::cout << tSample[i].size() << std::endl;
			}*/
			// system("pause");
			tDataSet[label].push_back(tSample);
		}
		for (int i = 0; i < 2; i++) {
			std::cout << i << ' ' << tDataSet[i].size() << std::endl;
			for (uint j = 0; j < tDataSet[i].size(); j++) {
				std::cout << tDataSet[i][j].size() << std::endl;
			}
		}
		mPreHandledFile.close();
		mTrainingDataFile.close();
	}

	void RecordPrestoredBodyData() {
		// record skeleton data
		std::cout << "[INFO] Begin Recording ..." << std::endl;
		int tFrameCount = 0;
		if (!pBodyFrameFile.is_open()) {
			return ;
		}
		for (auto tBodies : pBodyQ) {
			pBodyFrameFile << tFrameCount << ' '
				<< std::to_string(pTimestampQ[tFrameCount]) << '\n';
			// << GetTimestampStr(pTimestampQ[tFrameCount]) << '\n';

			bool tBodyDetected = false;
			tFrameCount++;
			for (uint i = 0; i < BODY_COUNT; i++) {
				BOOLEAN tIsTracked = false;
				if (checkResult(
					tBodies[i]->get_IsTracked(&tIsTracked),
					"IBody::get_IsTracked()") == 0) {
					// nothing
				}
				if (tIsTracked) {
					tBodyDetected = true;
					pBodyFrameFile << '#' << i << '\n';
					// get joint position
					Joint pJoints[JointType::JointType_Count];
					if (checkResult(
						tBodies[i]->GetJoints(JointType::JointType_Count, pJoints),
						"IBody::GetJoints()") == 0) {
						// nothing
					}

					// get joint orientation
					JointOrientation pOrientations[JointType::JointType_Count];
					if (checkResult(
						tBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations),
						"IBody::GetJointOrientations()") == 0) {
						// nothing
					}
					for (int j = 0; j < JointType::JointType_Count; j++) {
						const Joint& tJoint = pJoints[j];
						const JointOrientation& tOrientation = pOrientations[j];
						pBodyFrameFile << JointStr(tJoint) << ", " <<
							JointOrientationStr(tOrientation) << ", " <<
							(tJoint.TrackingState != TrackingState_NotTracked) << ';';
					}
					pBodyFrameFile << '\n';
				}
			}
			if (!tBodyDetected) {
				pBodyFrameFile << '#' << -1 << '\n';
			}
		}
		std::cout << "[INFO] Recording done!" << std::endl;

	}

	int work(chat_client & _c) {
		/*// test playing sound 
		PlaySound(TEXT("mywavsound.wav"), NULL, SND_FILENAME | SND_ASYNC);
		int a;
		std::cin >> a;

		PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC);
		return 0;*/

		// HRESULT hResult = S_OK;

		// handle training data
		/*if (handle_training_data == 1) {
			string tDirName = "./data/2015_06_30_21_12_11";
			HandleLabeledData(tDirName + "/skeleton.raw_data",
				tDirName + "/20150630_210408_00.label",
				tDirName + "/training.data");
			return 0;
		}
		else if (handle_training_data == 2) {
			string tDirName = "./data";
			ClusterTrainingData(tDirName + "/handled.data",
				tDirName + "/training.data");
			return 0;
		}*/

		// init
		if (Init() != 0) {
			return -1;
		}
		std::cout << "[INFO] Init done!" << std::endl;

		// debug window, in case there is no window visually
		// cv::namedWindow("debug");
		// open window
		if (visual_debug) {
			OpenWindows();
		}

		// infinite loop now, event method later
		while (1) {
			if (cv::waitKey(30) == VK_ESCAPE) {
				break;
			}

			if (pUseColor) {
				if (CaptureColorFrame() != 0) {
					// continue;
				}
				else {
					pColorFrameCount++;
				}
			}

			if (pUseDepth) {
				if (CaptureDepthFrame() != 0) {
					// continue;
				}
				else {
					pDepthFrameCount++;
				}
			}

			if (pUseBody) {
				if (CaptureBodyFrame() != 0) {
					// continue;
				}
				else {
					pBodyFrameCount++;
				}
			}

			if (pUseFace && pUseBody) {
				if (CaptureFaceFrame() != 0) {
					// continue;
				}
			}

			if (pUseBodyFaceIntegration) {
				if (CaptureBodyAndFaceFrame() != 0) {
					// continue;
				}
			}
			VisualWindows();
		}
		// system("pause");

		return 0;
	}
};

#endif
