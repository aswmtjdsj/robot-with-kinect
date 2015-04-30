#ifndef KINECT_SENSOR_HPP
#define KINECT_SENSOR_HPP

#include <kinect.h>

#include <Windows.h>

#include <opencv2/opencv.hpp>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int kinectSensor() {
	cv::setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor = nullptr;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "[Error] GetDefaultKinectSensor" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] GetDefaultKinectSensor" << std::endl;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::Open()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IKinectSensor::Open()" << std::endl;
	}

	// Source
	IDepthFrameSource* pDepthSource = nullptr; // depth
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	// printf("DepthFrameSource: %16x\n", pDepthSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IKinectSensor::get_DepthFrameSource()" << std::endl;
	}

	IColorFrameSource* pColorSource = nullptr; // color
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IKinectSensor::get_ColorFrameSource()" << std::endl;
	}

	IBodyFrameSource * pBodySource = nullptr; // body
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyFrameSource::get_BodyFrameSource()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IBodyFrameSource::get_BodyFrameSource()" << std::endl;
	}

	IBodyIndexFrameSource * pBodyIndexSource = nullptr; // body index
	hResult = pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IB// delete [] odyIndexFrameSource::get_BodyIndexFrameSource()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IBodyIndexFrameSource::get_BodyIndexFrameSource()" << std::endl;
	}

	// Reader
	IDepthFrameReader* pDepthReader = nullptr;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	// printf("DepthFrameReader: %16x\n", pDepthReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IDepthFrameSource::OpenReader()" << std::endl;
	}

	IColorFrameReader* pColorReader = nullptr;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IColorFrameSource::OpenReader()" << std::endl;
	}

	IBodyFrameReader * pBodyReader = nullptr;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyFrameReader::OpenReader()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IBodyFrameReader::OpenReader()" << std::endl;
	}

	IBodyIndexFrameReader * pBodyIndexReader = nullptr;
	hResult = pBodyIndexSource->OpenReader(&pBodyIndexReader);
	if (FAILED(hResult)) {
		std::cerr << "[Error] IBodyIndexFrameReader::OpenReader()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IBodyIndexFrameReader::OpenReader()" << std::endl;
	}

	// Description
	IFrameDescription *pDepthDescription = nullptr, *pColorDescription = nullptr;
	int depth_width = 0, color_width = 0, depth_height = 0, color_height = 0;

	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "[Error] IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	} else {
		std::cout << "[Success] IDepthFrameSource::get_FrameDescription()" << std::endl;
	}

	pDepthDescription->get_Width(&depth_width);
	pDepthDescription->get_Height(&depth_height);

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

	pColorDescription->get_Width(&color_width);
	pColorDescription->get_Height(&color_height);
	unsigned int bufferColorSize = color_width * color_height * sizeof(RGBQUAD);
	std::cout << "Color: w_" << color_width << ", h_" << color_height << std::endl;
	std::cout << "Color Buffer Size: " << bufferColorSize << std::endl;

	// Range
	unsigned short depth_min = 0, depth_max = 0;
	pDepthSource->get_DepthMinReliableDistance(&depth_min); // 500
	pDepthSource->get_DepthMaxReliableDistance(&depth_max); // 4500
	std::cout << "Depth Range : " << depth_min << " - " << depth_max << std::endl;

	RGBQUAD * pColorRGBX = new RGBQUAD[color_width * color_height];

	cv::Mat bufferDepthMat(depth_height, depth_width, CV_16UC1);
	cv::Mat depthMat(depth_height, depth_width, CV_8UC1);

	cv::Mat colorMat;
	cv::Mat colorShowMat;

	cv::Mat normalMat(depth_height, depth_width, CV_8UC3);
	cv::Mat visualMat(depth_height, depth_width, CV_8UC3);
	cv::Mat mapperMat(depth_height, depth_width, CV_8UC3);

	std::vector <int> s;
	std::map<int, cv::Vec3b> color_map;
	std::map<int, cv::Vec3f> re_map;
	std::map<int, int> color_map_count;
	s.resize(depth_height * depth_width);

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
			IBodyFrame * pBodyFrame = nullptr; // body
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
		if (SUCCEEDED(hResult)){
			pColorFrame->get_RawColorImageFormat(&imageFormat);
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
			}
			/*else if (1) {
			std::cout << "[Info] ColorImageFormat: RGBQUAD" << std::endl;
			}*/
		}

		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		//printf("DepthFrame: %16x\n", pDepthFrame);

		if (SUCCEEDED(hResult)){
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferDepthSize, reinterpret_cast<UINT16**>(&bufferDepthMat.data));

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
							}
						}
					}
				}
			}

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
								}
							}
						}
					}
					for (int biu = 0; biu < 3; biu++) {
						c[biu] /= t_cnt;
					}
					normalMat.at<cv::Vec3b>(y, x) = c;
				}
			}
			imshow("Visual", visualMat);
			imshow("Normal", normalMat);

			// for mapping, from depth to color
			ColorSpacePoint * spacePt = new ColorSpacePoint[depth_height * depth_width];
			mapper->MapDepthFrameToColorSpace(depth_height * depth_width, (UINT16 *)bufferDepthMat.data, depth_height * depth_width, spacePt);

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
			}
			delete[] spacePt;
		}

		if (SUCCEEDED(hResult)){

			hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);

			int body_count = 0;
			hResult = pBodySource->get_BodyCount(&body_count);
			// std::cout << "[Success] IBodyFrameSource::get_BodyCount() > " << body_count << std::endl;
			
			IBody ** pBodies = new IBody *[body_count];
			for (int i = 0; i < body_count; i++) {
				pBodies[i] = nullptr;
			}

			hResult = pBodyFrame->GetAndRefreshBodyData(body_count, pBodies);
			for (int i = 0; i < body_count; i++) {
				BOOLEAN is_tracked = false;
				hResult = pBodies[i]->get_IsTracked(&is_tracked);

				//  std::cout << i << ' ' << (int)is_tracked << endl;
				if (is_tracked) {
					// std::cout << "[Success] <tracked> " << i << endl;

					// get joint position
					Joint pJoints[JointType::JointType_Count];
					pBodies[i]->GetJoints(JointType::JointType_Count, pJoints);

					// get joint orientation
					JointOrientation pOrientations[JointType::JointType_Count];
					pBodies[i]->GetJointOrientations(JointType::JointType_Count, pOrientations);

					// output HEAD information
					JointType headJointType = JointType::JointType_Head;
					const Joint& headJointPos = pJoints[headJointType];
					const JointOrientation& headJointOri = pOrientations[headJointType];

					if (headJointPos.TrackingState != TrackingState_NotTracked) {
						//std::cout << "[Success] head: " << headJointPos.Position << "\n" << headJointOri.Orientation << endl;
						std::cout << "[Success] <head #" << i << 
							"> position (" << headJointPos.Position.X << 
							", " << headJointPos.Position.Y << 
							", " << headJointPos.Position.Z << 
							")\t orietation (" << headJointOri.Orientation.w << 
							", " << headJointOri.Orientation.x << 
							", " << headJointOri.Orientation.y << 
							", " << headJointOri.Orientation.z << ")\n";
					}

					// output LEFT HAND information
					JointType leftHandJointType = JointType::JointType_HandLeft;
					const Joint& leftHandJointPos = pJoints[leftHandJointType];
					const JointOrientation& leftHandJointOri = pOrientations[leftHandJointType];

					/*if (leftHandJointPos.TrackingState != TrackingState_NotTracked) {
						//std::cout << "[Success] head: " << headJointPos.Position << "\n" << headJointOri.Orientation << endl;
						std::cout << "[Success] <left hand #" << i << 
							"> position (" << leftHandJointPos.Position.X << 
							", " << leftHandJointPos.Position.Y << 
							", " << leftHandJointPos.Position.Z << 
							")\t orietation (" << leftHandJointOri.Orientation.w << 
							", " << leftHandJointOri.Orientation.x << 
							", " << leftHandJointOri.Orientation.y << 
							", " << leftHandJointOri.Orientation.z << ")\n";
					}*/
				}

			}

			/*hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);
			// pBodyIndexFrame->CopyFrameDataToArray()
			// hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&bufferDepthSize, 
			//	reinterpret_cast<UINT16**>(&bufferDepthMat.data));
			*/

			for (int i = 0; i < body_count; i++) {
				SafeRelease(pBodies[i]);
			}
			delete [] pBodies;

			bufferDepthMat.convertTo(depthMat, CV_8U, 255.0f / 4500.0f, .0f); //-255.0f / 4500.0f, 255.0f);
			cv::imshow("Depth", depthMat);
			cv::Mat colorResizedMat = cv::Mat(colorMat.rows / 2, colorMat.cols / 2, colorMat.type());
			cv::resize(colorMat, colorResizedMat, cv::Size(colorResizedMat.cols, colorResizedMat.rows));
			cv::imshow("Color", colorResizedMat);
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

	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();
	return 0;
}

#endif