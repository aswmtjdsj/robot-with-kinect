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

int kinectSensor(chat_client & _c) {
	cv::setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor = nullptr;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "[Error] GetDefaultKinectSensor" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] GetDefaultKinectSensor" << std::endl;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::Open()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IKinectSensor::Open()" << std::endl;
	}

	// Source
	IDepthFrameSource* pDepthSource = nullptr; // depth
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	// printf("DepthFrameSource: %16x\n", pDepthSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IKinectSensor::get_DepthFrameSource()" << std::endl;
	}

	IColorFrameSource* pColorSource = nullptr; // color
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IKinectSensor::get_ColorFrameSource()" << std::endl;
	}

	IBodyFrameSource * pBodySource = nullptr; // body
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyFrameSource::get_BodyFrameSource()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IBodyFrameSource::get_BodyFrameSource()" << std::endl;
	}

	IBodyIndexFrameSource * pBodyIndexSource = nullptr; // body index
	hResult = pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IB// delete [] odyIndexFrameSource::get_BodyIndexFrameSource()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IBodyIndexFrameSource::get_BodyIndexFrameSource()" << std::endl;
	}

	// Reader
	IDepthFrameReader* pDepthReader = nullptr;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	// printf("DepthFrameReader: %16x\n", pDepthReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IDepthFrameSource::OpenReader()" << std::endl;
	}

	IColorFrameReader* pColorReader = nullptr;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IColorFrameSource::OpenReader()" << std::endl;
	}

	IBodyFrameReader * pBodyReader = nullptr;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyFrameReader::OpenReader()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IBodyFrameReader::OpenReader()" << std::endl;
	}

	IBodyIndexFrameReader * pBodyIndexReader = nullptr;
	hResult = pBodyIndexSource->OpenReader(&pBodyIndexReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyIndexFrameReader::OpenReader()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IBodyIndexFrameReader::OpenReader()" << std::endl;
	}

	// Description
	IFrameDescription *pDepthDescription = nullptr, *pColorDescription = nullptr;
	int depth_width = 0, color_width = 0, depth_height = 0, color_height = 0;

	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "[_Error] IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IDepthFrameSource::get_FrameDescription()" << std::endl;
	}

	pDepthDescription->get_Width(&depth_width);
	if (FAILED(hResult)){
		std::cerr << "[Error] IFrameDescription::get_Width()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IFrameDescription::get_Width()" << std::endl;
	}
	pDepthDescription->get_Height(&depth_height);
	if (FAILED(hResult)){
		std::cerr << "[Error] IFrameDescription::get_Height()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IFrameDescription::get_Height()" << std::endl;
	}

	unsigned int depth_point_num = depth_width * depth_height;
	unsigned int bufferDepthSize = depth_point_num * sizeof(unsigned short);
	std::cout << "Depth: w_" << depth_width << ", h_" << depth_height << std::endl;
	std::cout << "Depth Buffer Size: " << bufferDepthSize << std::endl;

	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)){
		std::cerr << "[Error] IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IColorFrameSource::get_FrameDescription()" << std::endl;
	}

	hResult = pColorDescription->get_Width(&color_width);
	if (FAILED(hResult)){
		std::cerr << "[Error] IFrameDescription::get_Width()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IFrameDescription::get_Width()" << std::endl;
	}

	pColorDescription->get_Height(&color_height);
	if (FAILED(hResult)){
		std::cerr << "[Error] IFrameDescription::get_Height()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IFrameDescription::get_Height()" << std::endl;
	}
	unsigned int bufferColorSize = color_width * color_height * sizeof(RGBQUAD);
	std::cout << "Color: w_" << color_width << ", h_" << color_height << std::endl;
	std::cout << "Color Buffer Size: " << bufferColorSize << std::endl;

	// Range
	unsigned short depth_min = 0, depth_max = 0;
	pDepthSource->get_DepthMinReliableDistance(&depth_min); // 500
	if (FAILED(hResult)){
		std::cerr << "[Error] IDepthFrameSource::get_DepthMinReliableDistance()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IDepthFrameSource::get_DepthMinReliableDistance()" << std::endl;
	}
	pDepthSource->get_DepthMaxReliableDistance(&depth_max); // 4500
	if (FAILED(hResult)){
		std::cerr << "[Error] IDepthFrameSource::get_DepthMaxReliableDistance()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] IDepthFrameSource::get_DepthMaxReliableDistance()" << std::endl;
	}
	std::cout << "Depth Range : " << depth_min << " - " << depth_max << std::endl;

	RGBQUAD * pColorRGBX = new RGBQUAD[color_width * color_height];

	cv::Mat bufferDepthMat(depth_height, depth_width, CV_16UC1);
	cv::Mat depthMat(depth_height, depth_width, CV_8UC1);

	cv::Mat colorMat;
	cv::Mat colorShowMat;

	cv::Mat normalMat(depth_height, depth_width, CV_8UC3);
	cv::Mat visualMat(depth_height, depth_width, CV_8UC3);
	cv::Mat mapperMat(depth_height, depth_width, CV_8UC3);

	std::map<int, cv::Vec3b> color_map;
	std::map<int, cv::Vec3f> re_map;
	std::map<int, int> color_map_count;

	cv::namedWindow("Depth");
	cv::namedWindow("Color");
	cv::namedWindow("Visual");
	cv::namedWindow("Mapper");
	cv::namedWindow("Normal");

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
	if (FAILED(hResult)) {
		std::cerr << "[Error] ICoordinateMapper::get_CoordinateMapper()" << std::endl;
		return -1;
	}
	else {
		std::cout << "[Success] ICoordinateMapper::get_CoordinateMapper()" << std::endl;
	}

	ColorSpacePoint * spacePt = new ColorSpacePoint[depth_height * depth_width];

	// visual gesture builder
	IVisualGestureBuilderDatabase * vgb_database = nullptr;
	IVisualGestureBuilderFrameSource * vgb_source[BODY_COUNT];
	IVisualGestureBuilderFrameReader * vgb_reader[BODY_COUNT];

	for (uint i = 0; i < BODY_COUNT; i++) {
		hResult = CreateVisualGestureBuilderFrameSource(pSensor, 0, &vgb_source[i]);
		if (FAILED(hResult)){
			// cerr << "[Error] IVisualGestureBuilderFrameSource::CreateVisualGestureBuilderFrameSource()[" << i << "]: " << hResult << endl;
			return -1;
		}
		else {
			// cout << "[Success] IVisualGestureBuilderFrameSource::CreateVisualGestureBuilderFrameSource()[" << i << "]: " << hResult << endl;
		}

		hResult = vgb_source[i]->OpenReader(&vgb_reader[i]);
		if (FAILED(hResult)){
			// cerr << "[Error] IVisualGestureBuilderFrameSource::OpenReader()[" << i << "]: " << hResult << endl;
			return -1;
		}
		else {
			// cout << "[Success] IVisualGestureBuilderFrameSource::OpenReader()[" << i << "]: " << hResult << endl;
		}
	}

	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"HandUp.gba", &vgb_database);
	if (FAILED(hResult)) {
		cerr << "[Error] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
	}

	uint gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&gesture_count);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
	}

	IGesture * g_hand_over_head;
	hResult = vgb_database->get_AvailableGestures(gesture_count, &g_hand_over_head);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
		return 1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
	}

	GestureType g_type;
	g_hand_over_head->get_GestureType(&g_type);
	cerr << "gesture type: " << g_type << endl;

	wchar_t g_name[1000];
	g_hand_over_head->get_Name(1000, g_name);
	wcout << "gesture name: " << g_name << endl;

	SafeRelease(vgb_database);
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"Swipe.gba", &vgb_database);
	if (FAILED(hResult)) {
		cerr << "[Error] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
	}

	gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&gesture_count);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
	}
	IGesture * g_swiping;
	hResult = vgb_database->get_AvailableGestures(gesture_count, &g_swiping);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
		return 1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
	}

	g_swiping->get_GestureType(&g_type);
	cerr << "gesture type: " << g_type << endl;

	g_swiping->get_Name(1000, g_name);
	wcout << "gesture name: " << g_name << endl;

	// multiple gestures
	SafeRelease(vgb_database);
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"SampleDatabase.gbd", &vgb_database);
	if (FAILED(hResult)) {
		cerr << "[Error] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::CreateVisualGestureBuilderDatabaseInstanceFromFile(): " << hResult << endl;
	}

	uint multi_gesture_count = 0;
	hResult = vgb_database->get_AvailableGesturesCount(&multi_gesture_count);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
		return -1;
	}
	else {
		cout << "[Error] IVisualGestureBuilderDatabase::get_AvailableGesturesCount(): " << hResult << endl;
	}
	IGesture ** multiple_gestures = new IGesture *[multi_gesture_count];
	hResult = vgb_database->get_AvailableGestures(multi_gesture_count, multiple_gestures);
	if (FAILED(hResult)){
		cerr << "[Error] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
		return 1;
	}
	else {
		cout << "[Success] IVisualGestureBuilderDatabase::get_AvailableGestures(): " << hResult << endl;
	}

	cerr << "number of gestures in database: " << multi_gesture_count << endl;
	for (uint g = 0; g < multi_gesture_count; g++) {
		multiple_gestures[g]->get_GestureType(&g_type);
		cerr << "gesture type: " << g_type << endl;

		multiple_gestures[g]->get_Name(1000, g_name);
		wcout << "gesture name: " << g_name << endl;
	}

	for (uint i = 0; i < BODY_COUNT; i++){
		// hResult = vgb_source[i]->AddGestures(gesture_count, gestures);
		hResult = vgb_source[i]->AddGesture(g_hand_over_head);
		hResult = vgb_source[i]->AddGesture(g_swiping);
		for (uint g = 0; g < multi_gesture_count; g++) {
			hResult = vgb_source[i]->AddGesture(multiple_gestures[g]);
		}
		if (FAILED(hResult)){
			//std::cerr << "[Error] IVisualGestureBuilderFrameSource::AddGesture() -> " << std::hex << hResult << std::endl;
			return -1;
		}
		else {
			// std::cout << "[Success] IVisualGestureBuilderFrameSource::AddGesture()" << std::endl;
		}

		hResult = vgb_source[i]->SetIsEnabled(g_hand_over_head, true);
		hResult = vgb_source[i]->SetIsEnabled(g_swiping, true);
		for (uint g = 0; g < multi_gesture_count; g++) {
			hResult = vgb_source[i]->SetIsEnabled(multiple_gestures[g], true);
		}
		if (FAILED(hResult)){
			// std::cerr << "[Error] IVisualGestureBuilderFrameSource::SetIsEnabled()" << std::endl;
			return -1;
		}
		else {
			// std::cout << "[Success] IVisualGestureBuilderFrameSource::SetIsEnabled()" << std::endl;
		}
	}

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
		if (FAILED(hResult)){
			// cerr << "[Error] IColorFrameReader::AcquireLatestFrame(): " << hResult << endl;
			SafeRelease(pColorFrame);
			continue;
		}
		hResult = pColorFrame->get_RawColorImageFormat(&imageFormat);
		if (FAILED(hResult)){
			cerr << "[Error] IColorFrame::get_RawColorImageFormat(): " << hResult << endl;
			SafeRelease(pColorFrame);
			continue;
		}

		if (imageFormat == ColorImageFormat_Bgra) {
			std::cout << "[Info] ColorImageFormat: BGRA" << std::endl;
		}
		else {
			//hResult = pColorFrame->AccessRawUnderlyingBuffer(&bufferColorSize, reinterpret_cast<BYTE**>(&bufferColorMat.data));
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferColorSize, reinterpret_cast<BYTE *>(pColorRGBX), ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)){
				colorMat = cv::Mat(color_height, color_width, CV_8UC4, pColorRGBX);
				//resize(colorMat, colorShowMat, cv::Size(depth_width, depth_height));
				//cv::imshow("Color", colorShowMat);
				has_color = true;
			}
			else {
				cerr << "[Error] IColorFrame::CopyConvertedFrameDataToArray(): " << hResult << endl;
				SafeRelease(pColorFrame);
				continue;
			}
		}
		/*else if (1) {
		std::cout << "[Info] ColorImageFormat: RGBQUAD" << std::endl;
		}*/

		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		//printf("DepthFrame: %16x\n", pDepthFrame);

		if (SUCCEEDED(hResult)){
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferDepthSize, reinterpret_cast<UINT16**>(&bufferDepthMat.data));
			if (FAILED(hResult)) {
				cerr << "[Error] IDepthFrame::AccessUnderlyingBuffer(): " << std::hex << hResult << endl;
				continue;
			}

			// for visualization and normal 
			visualMat = cv::Mat(depth_height, depth_width, CV_8UC3);
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
			} // for y

			// filer normal matrix, using 3x3 avg
			for (int y = 0; y < depth_height; y++) {
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
			imshow("Normal", normalMat);

			// for mapping, from depth to co/salute.gbdlor
			hResult = mapper->MapDepthFrameToColorSpace(depth_height * depth_width, (UINT16 *)bufferDepthMat.data, depth_height * depth_width, spacePt);

			if (SUCCEEDED(hResult)) {
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
				cerr << "[Error] ICoordinateMapper::MapDepthFrameToColorSpace(): " << hResult << endl;
				continue;
			}
		}

		if (SUCCEEDED(hResult)){

			hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);

			if (SUCCEEDED(hResult)){
				int body_count = 0;
				hResult = pBodySource->get_BodyCount(&body_count);
				// std::cout << "[Success] IBodyFrameSource::get_BodyCount() > " << body_count << std::endl;
				if (SUCCEEDED(hResult)){
					IBody ** pBodies = new IBody *[body_count];
					for (int i = 0; i < body_count; i++) {
						pBodies[i] = nullptr;
					}
					if (SUCCEEDED(hResult)){
						hResult = pBodyFrame->GetAndRefreshBodyData(body_count, pBodies);
						for (uint i = 0; i < BODY_COUNT; i++) {


							BOOLEAN is_tracked = false;
							hResult = pBodies[i]->get_IsTracked(&is_tracked);
							if (is_tracked) {
								// get joint position
								Joint pJoints[JointType::JointType_Count];
								pBodies[i]->GetJoints(JointType::JointType_Count, pJoints);
								// get joint orientation
								JointOrientation pOrientations[JointType::JointType_Count];
								pBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations);

								// Set TrackingID to Detect Gesture
								UINT64 trackingId = _UI64_MAX;
								hResult = pBodies[i]->get_TrackingId(&trackingId);
								if (SUCCEEDED(hResult)){
									vgb_source[i]->put_TrackingId(trackingId);
								}

								int move[4] = { 0 };
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

						for (int i = 0; i < body_count; i++) {
							SafeRelease(pBodies[i]);
						} // safe release
						delete[] pBodies;

						bufferDepthMat.convertTo(depthMat, CV_8U, 255.0f / 4500.0f, .0f); //-255.0f / 4500.0f, 255.0f);
						cv::imshow("Depth", depthMat);
						cv::Mat colorResizedMat = cv::Mat(colorMat.rows / 2, colorMat.cols / 2, colorMat.type());
						cv::resize(colorMat, colorResizedMat, cv::Size(colorResizedMat.cols, colorResizedMat.rows));
						cv::imshow("Color", colorResizedMat);
					}
					else {
						cerr << "[Error] IBodyFrame::GetAndRefreshBodyData(): " << hResult << endl;
						continue;
					}
				}
				else {
					cerr << "[Error] IBodyFrameSource::get_BodyCount(): " << hResult << endl;
					continue;
				}
			}
			else {
				cerr << "[Error] IBodyFrameReader::AcquireLatestFrame(): " << hResult << endl;
				continue;
			}
		}

		for (uint i = 0; i < BODY_COUNT; i++) {
			IVisualGestureBuilderFrame* vgb_frame = nullptr;
			hResult = vgb_reader[i]->CalculateAndAcquireLatestFrame(&vgb_frame);
			if (SUCCEEDED(hResult) && vgb_frame != nullptr){
				BOOLEAN gesture_tracked = false;
				hResult = vgb_frame->get_IsTrackingIdValid(&gesture_tracked);
				if (SUCCEEDED(hResult) && gesture_tracked){
					IDiscreteGestureResult* discrete_result = nullptr;
					// hand over head
					hResult = vgb_frame->get_DiscreteGestureResult(g_hand_over_head, &discrete_result);
					if (SUCCEEDED(hResult) && discrete_result != nullptr){
						BOOLEAN bDetected = false;
						hResult = discrete_result->get_Detected(&bDetected);
						if (SUCCEEDED(hResult) && bDetected){
							std::cout << "hand over head Gesture detected" << std::endl;
							chat_message msg;
							string head_msg = "[kinect] button";
							msg.body_length(head_msg.size());
							std::memcpy(msg.body(), head_msg.c_str(), msg.body_length());
							cout << head_msg << endl;
							msg.encode_header();
							_c.write(msg);
						}
					}
					SafeRelease(discrete_result);

					// Continuous Gesture (Sample Swipe.gba is Action to Swipe the hand in horizontal direction.)
					IContinuousGestureResult* continuous_result = nullptr;
					hResult = vgb_frame->get_ContinuousGestureResult(g_swiping, &continuous_result);
					// cerr << hex << hResult << endl;
					// cerr << "result is null: " << (pGestureResult == nullptr) << endl;
					if (SUCCEEDED(hResult) && continuous_result != nullptr){
						float progress = 0.0f;
						hResult = continuous_result->get_Progress(&progress);
						if (SUCCEEDED(hResult)){
							// std::cout << "Progress: " + std::to_string(progress) << std::endl;
						}
					}
					SafeRelease(continuous_result);

					for (uint g = 0; g < multi_gesture_count; g++) {
						GestureType g_type;
						multiple_gestures[g]->get_GestureType(&g_type);
						if (g_type == GestureType_Discrete) {
							hResult = vgb_frame->get_DiscreteGestureResult(multiple_gestures[g], &discrete_result);
							if (SUCCEEDED(hResult) && discrete_result != nullptr){
								BOOLEAN bDetected = false;
								hResult = discrete_result->get_Detected(&bDetected);
								if (SUCCEEDED(hResult) && bDetected){
									wchar_t gesture_name[1000];
									multiple_gestures[g]->get_Name(1000, gesture_name);
									std::wcout << gesture_name << std::endl;
									chat_message msg;
									wstring w_msg = gesture_name;
									std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
									std::string s_msg = converter.to_bytes(w_msg);
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
									msg.encode_header();
									cout << multi_msg << endl;
									_c.write(msg);
								}
							}
							SafeRelease(discrete_result);
						}
						else if (g_type == GestureType_Continuous) {
							hResult = vgb_frame->get_ContinuousGestureResult(multiple_gestures[g], &continuous_result);
							float progress = 0.0f;
							hResult = continuous_result->get_Progress(&progress);
							if (SUCCEEDED(hResult)){
								wchar_t gesture_name[1000];
								multiple_gestures[g]->get_Name(1000, gesture_name);
								// std::wcout << gesture_name << L": " << std::endl;
								// std::cout << std::to_string(progress) << std::endl;
							}
							SafeRelease(continuous_result);
						}
					}
				}
			}
			SafeRelease(vgb_frame);
		}


		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		SafeRelease(pBodyFrame);
		SafeRelease(pBodyIndexFrame);

		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

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
