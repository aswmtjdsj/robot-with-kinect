#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <kinect.h>
// #include <Kinect.VisualGestureBuilder.h>

#include <Windows.h>

#include <opencv2/opencv.hpp>

#include "chat.hpp"

class Controller {

private:
	// Sensor
	IKinectSensor * pSensor; // kinect

	// Source
	IColorFrameSource * pColorSource; // color frame source
	IDepthFrameSource * pDepthSource; // depth frame source
	IBodyFrameSource * pBodySource; // body frame source
	IBodyIndexFrameSource * pBodyIndexSource; // body index frame source

	// Reader
	IColorFrameReader * pColorReader; // color frame reader
	IDepthFrameReader * pDepthReader; // depth frame reader
	IBodyFrameReader * pBodyReader; // body frame reader
	IBodyIndexFrameReader * pBodyIndexReader; // body index frame reader

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

	// bool: use color, depth, body or body index data
	bool pUseColor, pUseDepth, pUseBody, pUseBodyIndex;

	// matrix for view
	cv::Mat pDepthBufferMat, pDepthMat, // cut mat later
		pColorMat, pVisualMat, pMapperMat;

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
	deque <clock_t> pTimestampQ;
	deque <bool> pOwnMethodResults, pLibraryMethodResults;
	int pMaxQSize;

	bool visual_debug;

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
		pUseColor(false), pUseDepth(true), pUseBody(true), pUseBodyIndex(false),
		pColorRGBX(nullptr),
		pImageFormat(ColorImageFormat_None),
		colorSpacePts(nullptr),
		pMapper(nullptr),
		pMaxQSize(100),
		visual_debug(true) {

		pDepthQ.clear();
		pBodyQ.clear();
		pTimestampQ.clear();
		pOwnMethodResults.clear();
		pLibraryMethodResults.clear();
	}

	~Controller() {
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

	static inline void printJointInfo(const Joint & jt,
		const JointOrientation & jt_or,
		const string & jt_name) {
		std::cerr << "<" << jt_name <<
			"> position: (" << JointStr(jt) <<
			")\t orietation: (" << JointOrientationStr(jt_or) << ")\n";
	}

	static inline int checkResult(const HRESULT & hr, const string & prompt, bool display = true) {
		if (FAILED(hr)) {
			if (display) {
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

	int colorInit() {

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

		return 0;
	}

	int depthInit() {

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

	int bodyInit() {

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

	int bodyIndexInit() {

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

	void visualWindows() {
		if (pUseColor) {
			cv::imshow("Color", pColorMat);
		}
		if (pUseDepth) {
			cv::imshow("Depth", pDepthMat);
		}
	}

	int init() {
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
		if (pUseColor) {
			colorInit();
		}
		if (pUseDepth) {
			depthInit();
		}
		if (pUseBody) {
			bodyInit();
		}
		if (pUseBodyIndex) {
			bodyIndexInit();
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

	void openWindows() {
		if (pUseColor) {
			cv::namedWindow("Color");
		}
		if (pUseDepth) {
			cv::namedWindow("Depth");
		}
	}

	int captureColorFrame() {
		IColorFrame* pColorFrame = nullptr;
		cv::Mat tColorResizedMat;
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

		if (visual_debug) {
			tColorResizedMat = cv::Mat(pColorMat.rows / 2, pColorMat.cols / 2, pColorMat.type());
			cv::resize(pColorMat, tColorResizedMat, cv::Size(tColorResizedMat.cols, tColorResizedMat.rows));
			pColorMat = tColorResizedMat;
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

	int captureDepthFrame() {
		IDepthFrame* pDepthFrame = nullptr;
		bool tDepthOK = true;
		if (checkResult(
			pDepthReader->AcquireLatestFrame(&pDepthFrame),
			"IDepthFrameReader::AcquireLatestFrame()", false) != 0) {
				tDepthOK = false;
				goto RELEASE_DEPTH_FRAME;
		}
		else {

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

	int captureBodyFrame() {
		IBodyFrame* pBodyFrame = nullptr;
		bool tBodyOK = true;
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
				IBody ** pBodies = new IBody *[body_count];
				for (int i = 0; i < body_count; i++) {
					pBodies[i] = nullptr;
				}
				if (checkResult(
					pBodyFrame->GetAndRefreshBodyData(body_count, pBodies),
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
					for (uint i = 0; i < BODY_COUNT; i++) {
						BOOLEAN tIsTracked = false;
						if (checkResult(
							pBodies[i]->get_IsTracked(&tIsTracked),
							"IBody::get_IsTracked()") == 0) {
							// nothing
						}

						if (tIsTracked) {
							// get joint position
							Joint pJoints[JointType::JointType_Count];
							if (checkResult(
								pBodies[i]->GetJoints(JointType::JointType_Count, pJoints),
								"IBody::GetJoints()") == 0) {
								// nothing
							}

							// get joint orientation
							JointOrientation pOrientations[JointType::JointType_Count];
							if (checkResult(
								pBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations),
								"IBody::GetJointOrientations()") == 0) {
								// nothing
							}

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

							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& tJoint = pJoints[j];
								const JointOrientation& tOrientation = pOrientations[j];
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
						} // tIsTracked 
					} // for body_count
				}
			}
		}

	RELEASE_BODY_FRAME:
		SafeRelease(pBodyFrame);

		if (tBodyOK == false) {
			return -1;
		}
		else {
			return 0;
		}
	}

	int work(chat_client & _c) {
		// HRESULT hResult = S_OK;

		// init
		if (init() != 0) {
			return -1;
		}

		// open window
		if (visual_debug) {
			openWindows();
		}

		// infinite loop now, event method later
		while (1) {
			if (pUseColor) {
				if (captureColorFrame() != 0) {
					continue;
				}
			}

			if (pUseDepth) {
				if (captureDepthFrame() != 0) {
					continue;
				}
			}

			if (pUseBody) {
				if (captureBodyFrame() != 0) {
					continue;
				}
			}

			visualWindows();

			if (cv::waitKey(30) == VK_ESCAPE) {
				break;
			}
		}

		return 0;
	}
};

#endif
