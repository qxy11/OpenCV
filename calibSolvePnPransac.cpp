
#include <iostream>    
#include <opencv2/opencv.hpp>    
#include <fstream>


using namespace std;
using namespace cv;
void showAxes(Mat &_image, InputArray _cameraMatrix, InputArray _distCoeffs,
	InputArray _rvec, InputArray _tvec, float length) {
	// project points
	vector< Point3f > axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(length, 0, 0));
	axisPoints.push_back(Point3f(0, length, 0));
	axisPoints.push_back(Point3f(0, 0, length));
	vector< Point2f > imagePoints;
	projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

	//draw lines
	line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
	line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
	line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
}

int main(int argc, char **argv) {
	Mat imgMat;
	VideoCapture capture(0);
	ofstream tFile;
	ofstream rFile;
	tFile.open("tvec_points.txt");
	rFile.open("rvec_points.txt");


	if (capture.isOpened()) {
		//chessboard model
		Size boardSize(8, 6);
		float squareSize = 1;
		std::vector<Point3d> objectPoints;
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				objectPoints.push_back(
					Point3d(double(j * squareSize), float(i * squareSize), 0));
			}
		}

		Mat cameraMatrix, distCoeffs;
		FileStorage fs("out_camera_data.yml", FileStorage::READ);
		fs["camera_matrix"] >> cameraMatrix;
		fs["distortion_coefficients"] >> distCoeffs;

		while (true) {
			capture >> imgMat;
			if (imgMat.empty()) {
				break;
			}
			std::vector<Point2f> imagePoints;

			bool found = findChessboardCorners(imgMat, boardSize, imagePoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			if (found) {
				drawChessboardCorners(imgMat, boardSize, Mat(imagePoints), found);
				//solvePnPRansac
				Mat rvec, tvec;
				solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
				//projects axes on screen
				showAxes(imgMat, cameraMatrix, distCoeffs, rvec, tvec, squareSize);
				tFile << tvec << endl;
				rFile << rvec << endl;

			}

			imshow("window", imgMat);
			char ch = waitKey(30);

			if (ch == 27) {
				break;
			}
		}
	}
	else {
		std::cout << "Can't open VideoCapture :(" << std::endl;
	}
	/*
	//getting only z-coordinates
	ofstream myfile;
	myfile.open("zcoords.txt");
	const char* filename = "points.txt";
	std::ifstream inFile(filename);

	if (!inFile) {
		cout << endl << "Failed to open" << filename;
		return -1;
	}
	char str[255];
	long count = 1;
	while (inFile) {
		
		inFile.getline(str, 255);
		if (count % 3 == 0) {
		if (inFile) myfile << str << endl;
		}
		count++;
	}
	inFile.close();*/

	return 0;
}