#ifndef KINECT_SENSOR_HPP
#define KINECT_SENSOR_HPP

#include <kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include <Windows.h>

#include <opencv2/opencv.hpp>

#include "chat.hpp"
using namespace std;
using namespace cv;

void printJointInfo(const Joint & jt, const JointOrientation & jt_or, const string & jt_name) {
	std::cerr << "<" << jt_name <<
		"> position (" << jt.Position.X <<
		", " << jt.Position.Y <<
		", " << jt.Position.Z <<
		")\t orietation (" << jt_or.Orientation.w <<
		", " << jt_or.Orientation.x <<
		", " << jt_or.Orientation.y <<
		", " << jt_or.Orientation.z << ")\n";
}

inline int checkResult(const HRESULT & hr, const string & prompt, bool display = true) {
	if (FAILED(hr)) {
		if (true || display) {
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

void detect(deque <bool> & result,
	const deque <cv::Mat> & depthQ, const deque <IBody **> & bodyQ,
	const deque <clock_t> & timestampQ, int window_size, cv::Mat & curveMap) {

	HRESULT hResult = S_OK;

	for (auto pBodies : bodyQ) {
		bool detected = false;
		for (uint i = 0; i < BODY_COUNT; i++) {
			BOOLEAN is_tracked = false;
			hResult = pBodies[i]->get_IsTracked(&is_tracked);

			if (checkResult(hResult, "IBody::get_IsTracked()") == 0) {
				// nothing
			}

			if (is_tracked) {
				// get joint position
				Joint pJoints[JointType::JointType_Count];
				hResult = pBodies[i]->GetJoints(JointType::JointType_Count, pJoints);
				if (checkResult(hResult, "IBody::GetJoints()") == 0) {
					// nothing
				}
				// get joint orientation
				JointOrientation pOrientations[JointType::JointType_Count];
				hResult = pBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations);
				if (checkResult(hResult, "IBody::GetJointOrientations()") == 0) {
					// nothing
				}

				// printJointInfo(pJoints[JointType::JointType_HandLeft], pOrientations[JointType::JointType_HandLeft], "Left Hand");
				// printJointInfo(pJoints[JointType::JointType_HandRight], pOrientations[JointType::JointType_HandRight], "Right Hand");
				if (pJoints[JointType::JointType_HandRight].Position.Y >
					pJoints[JointType::JointType_Head].Position.Y || 
					pJoints[JointType::JointType_HandLeft].Position.Y >
					pJoints[JointType::JointType_Head].Position.Y) {
					// std::cout << "Right Hand Above Left Hand" << std::endl;
					detected = true;
				}
				else {
					// std::cout << "Right Hand Below Left Hand" << std::endl;
				}
				//if(pJoints[JointType::JointType_HandRight].Position.Z > ;
				/*for (int j = 0; j < JointType::JointType_Count; j++) {
					const Joint& jt = pJoints[j];
					const JointOrientation& jt_or = pOrientations[j];
					} // for joints*/

			} // is_tracked

		} // for i in bodies
		result.push_back(detected);
	}
}

int kinectSensor(chat_client & _c) {
	cv::setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor = nullptr;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (checkResult(hResult, "GetDefaultKinectSensor") != 0) {
		return -1;
	}

	hResult = pSensor->Open();
	if (checkResult(hResult, "IKinectSensor::Open()") != 0) {
		return -1;
	}

	// Source
	IDepthFrameSource* pDepthSource = nullptr; // depth
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	// printf("DepthFrameSource: %16x\n", pDepthSource);
	if (checkResult(hResult, "IKinectSensor::get_DepthFrameSource()") != 0) {
		return -1;
	}

	IColorFrameSource* pColorSource = nullptr; // color
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (checkResult(hResult, "IKinectSensor::get_ColorFrameSource()") != 0) {
		return -1;
	}

	IBodyFrameSource * pBodySource = nullptr; // body
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (checkResult(hResult, "IBodyFrameSource::get_BodyFrameSource()") != 0) {
		return -1;
	}

	IBodyIndexFrameSource * pBodyIndexSource = nullptr; // body index
	hResult = pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	if (checkResult(hResult, "IBodyIndexFrameSource::get_BodyIndexFrameSource()") != 0) {
		return -1;
	}

	// Reader
	IDepthFrameReader* pDepthReader = nullptr;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	// printf("DepthFrameReader: %16x\n", pDepthReader);
	if (checkResult(hResult, "IDepthFrameSource::OpenReader()") != 0) {
		return -1;
	}

	IColorFrameReader* pColorReader = nullptr;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (checkResult(hResult, "IColorFrameSource::OpenReader()") != 0) {
		return -1;
	}

	IBodyFrameReader * pBodyReader = nullptr;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (checkResult(hResult, "IBodyFrameReader::OpenReader()") != 0) {
		return -1;
	}

	IBodyIndexFrameReader * pBodyIndexReader = nullptr;
	hResult = pBodyIndexSource->OpenReader(&pBodyIndexReader);
	if (checkResult(hResult, "IBodyIndexFrameReader::OpenReader()") != 0) {
		return -1;
	}

	// Description
	IFrameDescription *pDepthDescription = nullptr, *pColorDescription = nullptr;
	int depth_width = 0, color_width = 0, depth_height = 0, color_height = 0;

	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (checkResult(hResult, "IDepthFrameSource::get_FrameDescription()") != 0) {
		return -1;
	}

	hResult = pDepthDescription->get_Width(&depth_width);
	if (checkResult(hResult, "IFrameDescription::get_Width()") != 0) {
		return -1;
	}

	hResult = pDepthDescription->get_Height(&depth_height);
	if (checkResult(hResult, "IFrameDescription::get_Height()") != 0) {
		return -1;
	}

	unsigned int depth_point_num = depth_width * depth_height;
	unsigned int bufferDepthSize = depth_point_num * sizeof(unsigned short);
	std::cout << "[INFO] Depth: w_" << depth_width << ", h_" << depth_height << std::endl;
	std::cout << "[INFO] Depth Buffer Size: " << bufferDepthSize << std::endl;

	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (checkResult(hResult, "IColorFrameSource::get_FrameDescription()") != 0) {
		return -1;
	}

	hResult = pColorDescription->get_Width(&color_width);
	if (checkResult(hResult, "IFrameDescription::get_Width()") != 0) {
		return -1;
	}

	hResult = pColorDescription->get_Height(&color_height);
	if (checkResult(hResult, "IFrameDescription::get_Height()") != 0) {
		return -1;
	}

	unsigned int bufferColorSize = color_width * color_height * sizeof(RGBQUAD);
	std::cout << "[INFO] Color: w_" << color_width << ", h_" << color_height << std::endl;
	std::cout << "[INFO] Color Buffer Size: " << bufferColorSize << std::endl;

	// Range
	unsigned short depth_min = 0, depth_max = 0;
	hResult = pDepthSource->get_DepthMinReliableDistance(&depth_min); // 500
	if (checkResult(hResult, "IDepthFrameSource::get_DepthMinReliableDistance()") != 0) {
		return -1;
	}

	hResult = pDepthSource->get_DepthMaxReliableDistance(&depth_max); // 4500
	if (checkResult(hResult, "IDepthFrameSource::get_DepthMaxReliableDistance()") != 0) {
		return -1;
	}
	std::cout << "[INFO] Depth Range : " << depth_min << " - " << depth_max << std::endl;

	RGBQUAD * pColorRGBX = new RGBQUAD[color_width * color_height];

	cv::Mat bufferDepthMat(depth_height, depth_width, CV_16UC1);
	cv::Mat depthMat(depth_height, depth_width, CV_8UC1);
	cv::Mat bufferCutMat(depth_height, depth_width, CV_8UC1);
	unsigned int bufferCutSize = depth_height * depth_width * sizeof(unsigned char);

	cv::Mat colorMat;
	cv::Mat colorShowMat;

	cv::Mat normalMat(depth_height, depth_width, CV_8UC3);
	cv::Mat visualMat(depth_height, depth_width, CV_8UC3);
	cv::Mat mapperMat(depth_height, depth_width, CV_8UC3);
	cv::Mat cutMat(depth_height, depth_width, CV_8UC3);

	std::map<int, cv::Vec3b> color_map;
	std::map<int, cv::Vec3f> re_map;
	std::map<int, int> color_map_count;

	cv::namedWindow("Depth");
	cv::namedWindow("Color");
	// cv::namedWindow("Visual");
	// cv::namedWindow("Mapper");
	// cv::namedWindow("Normal");
	cv::namedWindow("Cut");
	cv::namedWindow("Curve");

	int frameCount = 0;
	DWORD threadID = 0;
	HANDLE handle = NULL;

	/*printf("%x\n", &frameCount);
	handle = CreateThread(
	NULL,
	0,
	CounterThread,
	(LPVOID) &frameCount,
	0,
	&threadID
	);*/

	ColorImageFormat imageFormat = ColorImageFormat_None;

	//ICoordinateMapper
	ICoordinateMapper * mapper = nullptr;
	hResult = pSensor->get_CoordinateMapper(&mapper);
	if (checkResult(hResult, "ICoordinateMapper::get_CoordinateMapper()") != 0) {
		return -1;
	}

	ColorSpacePoint * spacePt = new ColorSpacePoint[depth_height * depth_width];

	// visual gesture builder
	IVisualGestureBuilderDatabase * vgb_database = nullptr;
	IVisualGestureBuilderFrameSource * vgb_source[BODY_COUNT];
	IVisualGestureBuilderFrameReader * vgb_reader[BODY_COUNT];

	for (uint i = 0; i < BODY_COUNT; i++) {
		hResult = CreateVisualGestureBuilderFrameSource(pSensor, 0, &vgb_source[i]);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::CreateVisualGestureBuilderFrameSource()[" + to_string(i) + "]: " + to_string(hResult), false) != 0) {
			return -1;
		}

		hResult = vgb_source[i]->OpenReader(&vgb_reader[i]);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::OpenReader()[" + to_string(i) + "]: " + to_string(hResult), false) != 0) {
			return -1;
		}
	}

	// hand up gesture
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"HandUp.gba", &vgb_database);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile()[\"HandUp.gba\"]: " + to_string(hResult)) != 0) {
		return -1;
	}

	uint gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&gesture_count);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGesturesCount()") != 0) {
		return -1;
	}

	IGesture * g_hand_over_head = nullptr;
	hResult = vgb_database->get_AvailableGestures(gesture_count, &g_hand_over_head);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGestures(): " + to_string(hResult)) != 0) {
		return -1;
	}
	std::cout << "[INFO] gesture count: " << gesture_count << endl;

	GestureType g_type;
	hResult = g_hand_over_head->get_GestureType(&g_type);
	if (checkResult(hResult, "IGesture::get_GestureType()") != 0) {
		return -1;
	}
	std::cout << "[INFO] gesture type: " << g_type << endl;

	wchar_t g_name[1000];
	hResult = g_hand_over_head->get_Name(1000, g_name);
	if (checkResult(hResult, "IGesture::get_Name()") != 0) {
		return -1;
	}
	std::wcout << "[INFO] gesture name: " << g_name << endl;

	SafeRelease(vgb_database);

	// swipe gesture
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"Swipe.gba", &vgb_database);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile()[\"Swipe.gba\"]: " + to_string(hResult)) != 0) {
		return -1;
	}

	gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&gesture_count);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGesturesCount()") != 0) {
		return -1;
	}
	std::cout << "[INFO] gesture count: " << gesture_count << endl;

	IGesture * g_swiping = nullptr;
	hResult = vgb_database->get_AvailableGestures(gesture_count, &g_swiping);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGestures(): " + to_string(hResult)) != 0) {
		return -1;
	}

	hResult = g_swiping->get_GestureType(&g_type);
	if (checkResult(hResult, "IGesture::get_GestureType()", false) != 0) {
		return -1;
	}
	std::cout << "[INFO] gesture type: " << g_type << endl;

	hResult = g_swiping->get_Name(1000, g_name);
	if (checkResult(hResult, "IGesture::get_Name()", false) != 0) {
		return -1;
	}
	std::wcout << "[INFO] gesture name: " << g_name << endl;
	SafeRelease(vgb_database);

	// multiple gestures
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"SampleDatabase.gbd", &vgb_database);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile()[\"SampleDatabase.gbd\"]: " + to_string(hResult)) != 0) {
		return -1;
	}

	uint multi_gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&multi_gesture_count);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGestures(): " + to_string(hResult)) != 0) {
		return -1;
	}
	std::cout << "[INFO] gesture count: " << multi_gesture_count << endl;

	IGesture ** multiple_gestures = new IGesture *[multi_gesture_count];
	hResult = vgb_database->get_AvailableGestures(multi_gesture_count, multiple_gestures);
	if (checkResult(hResult, "IVisualGestureBuilderDatabase::get_AvailableGestures(): " + to_string(hResult)) != 0) {
		return -1;
	}

	std::cout << "[INFO] number of gestures in database: " << multi_gesture_count << endl;
	for (uint g = 0; g < multi_gesture_count; g++) {
		hResult = multiple_gestures[g]->get_GestureType(&g_type);
		if (checkResult(hResult, "IGesture::get_GestureType()", false) != 0) {
			return -1;
		}
		std::cout << "[INFO] gesture type: " << g_type << endl;

		hResult = multiple_gestures[g]->get_Name(1000, g_name);
		if (checkResult(hResult, "IGesture::get_Name()", false) != 0) {
			return -1;
		}
		std::wcout << "[INFO] gesture name: " << g_name << endl;
	}

	for (uint i = 0; i < BODY_COUNT; i++){
		// hResult = vgb_source[i]->AddGestures(gesture_count, gestures);
		hResult = vgb_source[i]->AddGesture(g_hand_over_head);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::AddGesture(): [HandUp] Body #" + to_string(i), false) != 0) {
			return -1;
		}
		hResult = vgb_source[i]->AddGesture(g_swiping);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::AddGesture(): [Swiping] Body #" + to_string(i), false) != 0) {
			return -1;
		}
		for (uint g = 0; g < multi_gesture_count; g++) {
			hResult = vgb_source[i]->AddGesture(multiple_gestures[g]);
			if (checkResult(hResult, "IVisualGestureBuilderFrameSource::AddGesture(): [Multiple] " + to_string(g) + " Body #" + to_string(i), false) != 0) {
				return -1;
			}
		}

		hResult = vgb_source[i]->SetIsEnabled(g_hand_over_head, true);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::SetIsEnabled(): [HandUp] Body #" + to_string(i), false) != 0) {
			return -1;
		}
		hResult = vgb_source[i]->SetIsEnabled(g_swiping, true);
		if (checkResult(hResult, "IVisualGestureBuilderFrameSource::SetIsEnabled(): [HandUp] Body #" + to_string(i), false) != 0) {
			return -1;
		}
		for (uint g = 0; g < multi_gesture_count; g++) {
			hResult = vgb_source[i]->SetIsEnabled(multiple_gestures[g], true);
			if (checkResult(hResult, "IVisualGestureBuilderFrameSource::SetIsEnabled(): [Multiple] " + to_string(g) + " Body #" + to_string(i), false) != 0) {
				return -1;
			}
		}
	}

	// store time-window data, including frames of depth, body, (or color?), result of detection
	deque <cv::Mat> depthQ;
	deque <IBody **> bodyQ;
	deque <clock_t> timestampQ;
	deque <bool> result, result_comp;
	int max_q_size = 30;

	// forever loop, activation maybe a better choice
	while (1){
		// Frame
		//frameCount++;
		//printf("%d\n", *(&frameCount));
		IDepthFrame* pDepthFrame = nullptr;
		IColorFrame* pColorFrame = nullptr;
		IBodyFrame * pBodyFrame = nullptr; // body
		IBodyIndexFrame * pBodyIndexFrame = nullptr; // body index
		bool has_color = false;

		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		//printf("ColorFrame: %16x\n", pColorFrame);
		if (checkResult(hResult, "IColorFrameReader::AcquireLatestFrame()", false) != 0) {
			/*
			* Be sure you release the color frame whenever you're done 
			* ( with myColorFrame.Release() )! 
			* If the color frame you point to has data, 
			* it will return E_PENDING until it's cleared, 
			* which confused me for quite a bit.
			*/
			goto RELEASE_FRAMES;
		}
		hResult = pColorFrame->get_RawColorImageFormat(&imageFormat);
		if (checkResult(hResult, "IColorFrame::get_RawColorImageFormat()", false) != 0) {
			goto RELEASE_FRAMES;
		}

		if (imageFormat == ColorImageFormat_Bgra) {
			std::cout << "[INFO] ColorImageFormat: BGRA" << std::endl;
		}
		else {
			//hResult = pColorFrame->AccessRawUnderlyingBuffer(&bufferColorSize, reinterpret_cast<BYTE**>(&bufferColorMat.data));
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferColorSize, reinterpret_cast<BYTE *>(pColorRGBX), ColorImageFormat_Bgra);
			if (checkResult(hResult, "IColorFrame::CopyConvertedFrameDataToArray()", false) != 0) {
				goto RELEASE_FRAMES;
			}
			else {
				colorMat = cv::Mat(color_height, color_width, CV_8UC4, pColorRGBX);
				//resize(colorMat, colorShowMat, cv::Size(depth_width, depth_height));
				//cv::imshow("Color", colorShowMat);
				has_color = true;
			}
		}
		/*else if (1) {
		std::cout << "[Info] ColorImageFormat: RGBQUAD" << std::endl;
		}*/

		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		//printf("DepthFrame: %16x\n", pDepthFrame);
		if (checkResult(hResult, "IDepthFrameReader::AcquireLatestFrame()", false) == 0) {
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferDepthSize, reinterpret_cast<UINT16**>(&bufferDepthMat.data));
			if (checkResult(hResult, "IDepthFrame::AccessUnderlyingBuffer()", false) != 0) {
				goto RELEASE_FRAMES;
			}
			// store depth data in queue
			// clock_t start = clock();
			cv::Mat depthCopy = bufferDepthMat.clone();
			depthQ.push_back(depthCopy);
			if (depthQ.size() > max_q_size) {
				depthQ.pop_front();
			}
			// clock_t end = clock();
			// std::cout << "[INFO] " << (end - start) * 1. << " ms" << endl;

			// for visualization and normal 
			/*visualMat = cv::Mat(depth_height, depth_width, CV_8UC3);
			normalMat = cv::Mat(depth_height, depth_width, CV_8UC3);
			for (int y = 0; y < depth_height; y++) {
				for (int x = 0; x < depth_width; x++) {
					int z = bufferDepthMat.at<unsigned short>(y, x);
					// std::cout << x << ' ' << y << ' ' << bufferDepthMat.at<unsigned short>(y, x) << std::endl;
					if (z != 0) { // valid depth value
						visualMat.at<cv::Vec3b>(y, x)[0] = x * 255. / depth_width;
						visualMat.at<cv::Vec3b>(y, x)[1] = y * 255. / depth_height;
						visualMat.at<cv::Vec3b>(y, x)[2] = z * 255. / 4500.;

						if (y < depth_height - 1 && x < depth_width - 1) {
							cv::Vec3f down, right;
							down[0] = x;
							down[1] = y + 1;
							down[2] = bufferDepthMat.at<unsigned short>(y + 1, x);
							right[0] = x + 1;
							right[1] = y;
							right[2] = bufferDepthMat.at<unsigned short>(y, x + 1);
							float a, b, c, no;
							a = down[1] * right[2] - down[2] * right[1];
							b = down[2] * right[0] - down[0] * right[2];
							c = down[0] * right[1] - down[1] * right[0];
							if (a * a + b * b + c * c > 1e-6) {
								no = 1. / (sqrt(a * a + b * b + c * c));
								normalMat.at<cv::Vec3b>(y, x)[0] = a * no * 127. + 127.;
								normalMat.at<cv::Vec3b>(y, x)[1] = b * no * 127. + 127.;
								normalMat.at<cv::Vec3b>(y, x)[2] = c * no * 127. + 127.;
							} // if square sum
						} // if x y boundary
					} // if valid depth value
				} // for x
			} // for y */
			// filer normal matrix, using 3x3 avg
			/*for (int y = 0; y < depth_height; y++) {
				for (int x = 0; x < depth_width; x++) {
					cv::Vec3f c;
					c[0] = c[1] = c[2] = 0;
					int t_cnt = 0;
					for (int k = -1; k <= 1; k++) {
						for (int K = -1; K <= 1; K++) {
							int ny = y + k, nx = x + K;
							if (0 <= nx && nx < depth_width && 0 <= ny && ny < depth_height) {
								t_cnt++;
								for (int biu = 0; biu < 3; biu++) {
									c[biu] += normalMat.at<cv::Vec3b>(ny, nx)[biu];
								} // for 3 channel
							} // if nx ny within boundary
						} // for K
					} // for k
					for (int biu = 0; biu < 3; biu++) {//#
						c[biu] /= t_cnt;
					} // regularization for three channel

					normalMat.at<cv::Vec3b>(y, x) = c;
				} // for x
			} // for y
			imshow("Visual", visualMat);
			imshow("Normal", normalMat);*/

			bufferDepthMat.convertTo(depthMat, CV_8U, 255.0f / 4500.0f, .0f); // -255.0f / 4500.0f, 255.0f); // 
			cv::imshow("Depth", depthMat);

			// for mapping, from depth to color
			/*hResult = mapper->MapDepthFrameToColorSpace(depth_height * depth_width, (UINT16 *)bufferDepthMat.data, depth_height * depth_width, spacePt);
			if (checkResult(hResult, "ICoordinateMapper::MapDepthFrameToColorSpace()", false) == 0) {
				if (has_color) {
					mapperMat = cv::Mat(depth_height, depth_width, CV_8UC3);
					for (int y = 0; y < depth_height; y++) {
						for (int x = 0; x < depth_width; x++) {
							ColorSpacePoint pt = spacePt[y * depth_width + x];
							pt.X = floor(pt.X);
							pt.Y = floor(pt.Y);
							if (pt.X >= 0 && pt.X < color_width && pt.Y >= 0 && pt.Y < color_height) {
								for (int k = 0; k < 3; k++) {
									mapperMat.at<cv::Vec3b>(y, x)[k] = colorMat.at<cv::Vec4b>(pt.Y, pt.X)[k];
								}
							}
						}
					}
					cv::imshow("Mapper", mapperMat);
				} // has_color
			}
			else {
				goto RELEASE_FRAMES;
			}*/
		}
		else {
			goto RELEASE_FRAMES;
		}

		// background and foreground
		hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);
		if (checkResult(hResult, "IBodyIndexFrameReader::AcquireLatestFrame()", false) != 0) {
			goto RELEASE_FRAMES;
		}

		hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&bufferCutSize, reinterpret_cast<UCHAR**>(&bufferCutMat.data));
		if (checkResult(hResult, "IBodyIndexFrame::AccessUnderlyingBuffer()", false) != 0) {
			goto RELEASE_FRAMES;
		}
		for (int y = 0; y < depth_height; y++) {
			for (int x = 0; x < depth_width; x++) {
				uchar cut = bufferCutMat.at<UCHAR>(y, x);
				if (0 <= cut && cut < 6) {
					cutMat.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
				}
				else {
					cutMat.at<Vec3b>(y, x) = Vec3b(0.);
				}
			}
		}
		cv::imshow("Cut", cutMat);

		// get body joints
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (checkResult(hResult, "IBodyFrameReader::AcquireLatestFrame()") == 0) {
			int body_count = 0;
			hResult = pBodySource->get_BodyCount(&body_count);
			// std::cout << "[Success] IBodyFrameSource::get_BodyCount() > " << body_count << std::endl;
			if (checkResult(hResult, "IBodyFrameSource::get_BodyCount()") == 0) {
				IBody ** pBodies = new IBody *[body_count];
				for (int i = 0; i < body_count; i++) {
					pBodies[i] = nullptr;
				}
				hResult = pBodyFrame->GetAndRefreshBodyData(body_count, pBodies);

				// store body data in queue
				bodyQ.push_back(pBodies);
				if (bodyQ.size() > max_q_size) {
					IBody ** toRelease = bodyQ.front();
					for (int i = 0; i < BODY_COUNT; i++) {
						SafeRelease(toRelease[i]);
					}
					delete [] toRelease;
					bodyQ.pop_front();
				}
				
				// store timestamp in queue
				timestampQ.push_back(clock());
				if (timestampQ.size() > max_q_size) {
					timestampQ.pop_front();
				}
				std::cout << "[INFO] depth: " << depthQ.size() << " body: " << bodyQ.size()
					<< " result1: " << result.size() << " result2: " << result_comp.size() 
					<< " timestamp: " << timestampQ.size() << " first stamp was: " << (clock() - timestampQ.front()) << " ms ago" << endl;

				if (checkResult(hResult, "IBodyFrame::GetAndRefreshBodyData()") == 0) {
					for (uint i = 0; i < BODY_COUNT; i++) {
						BOOLEAN is_tracked = false;
						hResult = pBodies[i]->get_IsTracked(&is_tracked);

						if (checkResult(hResult, "IBody::get_IsTracked()") == 0) {
							// nothing
						}

						if (is_tracked) {
							// get joint position
							Joint pJoints[JointType::JointType_Count];
							hResult = pBodies[i]->GetJoints(JointType::JointType_Count, pJoints);
							if (checkResult(hResult, "IBody::GetJoints()") == 0) {
								// nothing
							}
							// get joint orientation
							JointOrientation pOrientations[JointType::JointType_Count];
							hResult = pBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations);
							if (checkResult(hResult, "IBody::GetJointOrientations()") == 0) {
								// nothing
							}

							// IMPORTANT!!!
							// Set TrackingID to Detect Gesture
							UINT64 trackingId = _UI64_MAX;
							hResult = pBodies[i]->get_TrackingId(&trackingId);
							if (checkResult(hResult, "IBody::get_TrackingId()") == 0) {
								hResult = vgb_source[i]->put_TrackingId(trackingId);
								if (checkResult(hResult, "IVisualGestureBuilderFrameSource::put_TrackingId()") == 0) {
									// nothing
								}
							}

							for (int j = 0; j < JointType::JointType_Count; j++) {
								const Joint& jt = pJoints[j];
								const JointOrientation& jt_or = pOrientations[j];
								// draw all joints
								if (jt.TrackingState != TrackingState_NotTracked) {
									/*std::cerr << "[Success] #" << i << ' ';
									printJointInfo(jt, jt_or, to_string(j));*/
									// re-mapping the joint position
									CameraSpacePoint cameraPt = jt.Position;
									ColorSpacePoint colorPt;
									int num_pt = 1;
									hResult = mapper->MapCameraPointsToColorSpace(num_pt, &cameraPt, num_pt, &colorPt);
									if (checkResult(hResult, "ICoordinateMapper::MapCameraPointsToColorSpace()") == 0) {
										// nothing
									}
									colorPt.X = floor(colorPt.X);
									colorPt.Y = floor(colorPt.Y);
									if (colorPt.X >= 0 && colorPt.X < color_width
										&& colorPt.Y >= 0 && colorPt.Y < color_height) {
										cv::Point to_draw;
										to_draw.x = colorPt.X;
										to_draw.y = colorPt.Y;
										int radius = 25;
										if (jt.TrackingState == TrackingState_Inferred) {
											circle(colorMat, to_draw, radius, cv::Scalar(0., 0., 255.));
										}
										else {
											circle(colorMat, to_draw, radius, cv::Scalar(0., 0., 255.), -1);
										}
										cv::Point text_corner = to_draw;
										text_corner.x += radius;
										text_corner.y += radius;
										putText(colorMat, to_string(j), text_corner,
											cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0., 255., 255.), 2);
									} // colorPt within boundary
								} // if tracking
							} // for joints

						} // is_tracked

					} // for i in bodies

					/*for (int i = 0; i < body_count; i++) {
						SafeRelease(pBodies[i]);
					} // safe release
					delete[] pBodies;*/

					cv::Mat colorResizedMat = cv::Mat(colorMat.rows / 2, colorMat.cols / 2, colorMat.type());
					cv::resize(colorMat, colorResizedMat, cv::Size(colorResizedMat.cols, colorResizedMat.rows));
					cv::imshow("Color", colorResizedMat);
				} // GetAndRefreshBodyData
				else {
					goto RELEASE_FRAMES;
				}
			} // get_BodyCount
			else {
				goto RELEASE_FRAMES;
			}
		} // AcquireLatestFrame
		else {
			goto RELEASE_FRAMES;
		}

		// detection
		bool head_detected = false;
		for (uint i = 0; i < BODY_COUNT; i++) {
			IVisualGestureBuilderFrame* vgb_frame = nullptr;
			hResult = vgb_reader[i]->CalculateAndAcquireLatestFrame(&vgb_frame);

			if (checkResult(hResult, "IVisualGestureBuilderFrameReader::CalculateAndAcquireLatestFrame(): " + to_string(i)) == 0
				&& vgb_frame != nullptr) {
				BOOLEAN gesture_tracked = false;
				hResult = vgb_frame->get_IsTrackingIdValid(&gesture_tracked);
				if (checkResult(hResult, "IVisualGestureBuilderFrame::get_IsTrackingIdValid()") == 0
					&& gesture_tracked) {
					IDiscreteGestureResult* discrete_result = nullptr;
					// hand over head
					hResult = vgb_frame->get_DiscreteGestureResult(g_hand_over_head, &discrete_result);
					if (checkResult(hResult, "IVisualGestureBuilderFrame::get_DiscreteGestureResult()") == 0
						&& discrete_result != nullptr) {
						BOOLEAN bDetected = false;
						hResult = discrete_result->get_Detected(&bDetected);
						if (checkResult(hResult, "IDiscreteGestureResult::get_Detected()") == 0
							&& bDetected) {
							head_detected = true;
							std::cout << "[INFO] hand over head Gesture detected" << std::endl;
							chat_message msg;
							string head_msg = "[kinect] button";
							msg.body_length(head_msg.size());
							std::memcpy(msg.body(), head_msg.c_str(), msg.body_length());
							msg.body()[msg.body_length()] = 0;
							// cout << head_msg << endl;
							cout << msg.body() << endl;
							msg.encode_header();
							_c.write(msg);
						}
					}
					SafeRelease(discrete_result);

					// Continuous Gesture (Sample Swipe.gba is Action to Swipe the hand in horizontal direction.)
					IContinuousGestureResult* continuous_result = nullptr;
					hResult = vgb_frame->get_ContinuousGestureResult(g_swiping, &continuous_result);
					if (checkResult(hResult, "IVisualGestureBuilderFrame::get_ContinuousGestureResult()") == 0
						&& continuous_result != nullptr){
						float progress = 0.0f;
						hResult = continuous_result->get_Progress(&progress);
						if (checkResult(hResult, "IContinuousGestureResult::get_Progress()") == 0) {
							// std::cout << "Progress: " + std::to_string(progress) << std::endl;
						}
					}
					SafeRelease(continuous_result);

					for (uint g = 0; g < multi_gesture_count; g++) {
						GestureType g_type;
						hResult = multiple_gestures[g]->get_GestureType(&g_type);
						if (checkResult(hResult, "IGesture::get_GestureType()") != 0) {
							// nothing
						}

						if (g_type == GestureType_Discrete) {
							hResult = vgb_frame->get_DiscreteGestureResult(multiple_gestures[g], &discrete_result);
							if (checkResult(hResult, "IVisualGestureBuilderFrame::get_DiscreteGestureResult()") == 0
								&& discrete_result != nullptr) {
								BOOLEAN bDetected = false;
								hResult = discrete_result->get_Detected(&bDetected);
								if (checkResult(hResult, "IDiscreteGestureResult::get_Detected()") == 0
									&& bDetected) {
									wchar_t gesture_name[1000];
									hResult = multiple_gestures[g]->get_Name(1000, gesture_name);
									if (checkResult(hResult, "IGesture::get_Name()") != 0) {
										// nothing
									}
									std::wcout << L"[INFO] " << gesture_name << L'\n';
									chat_message msg;
									// wstring w_msg = gesture_name;
									// std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
									// std::string s_msg = converter.to_bytes(w_msg);
									std::string s_msg = wConvToS(gesture_name);
									string multi_msg = "[kinect] ";
									if (s_msg == "Steer_Left") {
										multi_msg += "left";
									}
									else if (s_msg == "Steer_Right") {
										multi_msg += "right";
									}
									else {
										multi_msg += "forward";
									}
									msg.body_length(multi_msg.size());
									std::memcpy(msg.body(), multi_msg.c_str(), msg.body_length());
									msg.body()[msg.body_length()] = 0;
									msg.encode_header();
									// cout << multi_msg << endl;
									cout << msg.body() << endl;
									_c.write(msg);
								} // get_Detected
							} // get_DiscreteGestureResult
							SafeRelease(discrete_result);
						}
						else if (g_type == GestureType_Continuous) {
							hResult = vgb_frame->get_ContinuousGestureResult(multiple_gestures[g], &continuous_result);
							if (checkResult(hResult, "IVisualGestureBuilderFrame::get_ContinuousGestureResult()") == 0
								&& continuous_result != nullptr){
								float progress = 0.0f;
								hResult = continuous_result->get_Progress(&progress);
								if (checkResult(hResult, "IContinuousGestureResult::get_Progress()") == 0) {
									wchar_t gesture_name[1000];
									hResult = multiple_gestures[g]->get_Name(1000, gesture_name);
									if (checkResult(hResult, "IGesture::get_Name()") != 0) {
										// nothing
									} // get_ContinuousGestureResult
									// std::wcout << gesture_name << L": " << std::endl;
									// std::cout << std::to_string(progress) << std::endl;
								}
							}
							SafeRelease(continuous_result);
						} // continuous gesture
					} // for multiple gestures
				} // get_IsTrackingIdValid
			} // CalculateAndAcquireLatestFrame
			SafeRelease(vgb_frame);
		} // for body count
		result_comp.push_back(head_detected);
		if (result_comp.size() > max_q_size) {
			result_comp.pop_front();
		}

		// new detection
		{
			// own part
			int single_width = 20, y_bar = 120, single_height = 4,
				left_offset_1 = 30, left_offset_2 = 180, up_offset = 25,
				line_offset_1 = 5, line_offset_2 = 155, line_len = 23, line_dis = 20;
			cv::Mat curveMap(y_bar * single_height, max((int)bodyQ.size(), max_q_size) * single_width, CV_8UC3);
			curveMap.setTo(0);

			cv::Point text_corner;
			text_corner.x = left_offset_1;
			text_corner.y = up_offset;
			putText(curveMap, "Our Method", text_corner,
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0., 0., 255.), 1);

			line(curveMap, cv::Point(line_offset_1, line_dis),
				cv::Point(line_offset_1+line_len, line_dis), cv::Scalar(0., 0., 255.), 2);

			text_corner.x = left_offset_2;
			text_corner.y = up_offset;
			putText(curveMap, "Microsoft Method", text_corner,
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0., 255., 255.), 1);

			line(curveMap, cv::Point(line_offset_2, line_dis),
				cv::Point(line_offset_2+line_len, line_dis), cv::Scalar(0., 255., 255.), 2);

			result.clear();
			detect(result, depthQ, bodyQ, timestampQ, 0, curveMap);
			cv::Point to_draw, drawn;
			drawn.x = drawn.y = 0;
			int id = 0;
			for (auto detected : result) {
				to_draw.x = id * single_width + 10;
				if (detected) {
					to_draw.y = 15 * single_height + single_height;
				}
				else {
					to_draw.y = 115 * single_height + single_height;
				}
				int radius = 4;
				circle(curveMap, to_draw, radius, cv::Scalar(0., 0., 255.), -1); // filled
				if (drawn.x != 0 && drawn.y != 0) {
					line(curveMap, drawn, to_draw, cv::Scalar(0., 0., 255.), 2);
				}
				drawn = to_draw;
				id++;
			}

			// microsoft part

			drawn.x = drawn.y = 0;
			id = 0;
			for (auto detected : result_comp) {
				to_draw.x = id * single_width + 10;
				if (detected) {
					to_draw.y = 15 * single_height + single_height;
				}
				else {
					to_draw.y = 115 * single_height + single_height;
				}
				int radius = 4;
				circle(curveMap, to_draw, radius, cv::Scalar(0., 255., 255.), -1); // filled
				if (drawn.x != 0 && drawn.y != 0) {
					line(curveMap, drawn, to_draw, cv::Scalar(0., 255., 255.), 2);
				}
				drawn = to_draw;
				id++;
			}
			
			imshow("Curve", curveMap);
		}

		// for debugging
		/*if (cv::waitKey(30) == 'a'){
			int id = 0;
			for (auto frame : depthQ) {
				frame.convertTo(depthMat, CV_8U, 255.0f / 4500.0f, .0f); // -255.0f / 4500.0f, 255.0f); // 
				imwrite(to_string(id) + ".png", depthMat);
				id++;
			}
			break;
		}*/

	RELEASE_FRAMES:
		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		SafeRelease(pBodyFrame);
		SafeRelease(pBodyIndexFrame);

		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	} // while 1

	// clean junk

	if (spacePt != nullptr) {
		delete[] spacePt;
	}

	SafeRelease(pDepthSource);
	SafeRelease(pDepthReader);
	SafeRelease(pDepthDescription);

	SafeRelease(pColorSource);
	SafeRelease(pColorReader);
	SafeRelease(pColorDescription);

	SafeRelease(pBodySource);
	SafeRelease(pBodyReader);

	SafeRelease(pBodyIndexSource);
	SafeRelease(pBodyIndexReader);

	for (uint i = 0; i < BODY_COUNT; i++) {
		SafeRelease(vgb_source[i]);
		SafeRelease(vgb_reader[i]);
	}
	SafeRelease(vgb_database);

	if (pSensor){
		pSensor->Close();
	}

	SafeRelease(pSensor);
	cv::destroyAllWindows();
	return 0;
}

#endif
