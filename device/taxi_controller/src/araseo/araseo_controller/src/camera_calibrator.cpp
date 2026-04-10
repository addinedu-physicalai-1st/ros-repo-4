#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "CameraPublisher.hpp"
#include "SensorDataSubscriber.h"
#include "Base64.hpp"

using namespace std;
using namespace cv;

// ==============================================================
// [중요 설정] 생성하신 ChArUco 보드의 사양을 정확히 입력해야 합니다.
// ==============================================================
const int squaresX = 8;
const int squaresY = 11;
const float squareLength = 0.020f; // 20mm
const float markerLength = 0.015f; // 15mm
// ==============================================================

extern std::atomic<bool> g_enable_sensor_log;

int main() {
    // [로그 차단] 센서 수신 로그를 꺼서 화면 도배를 막습니다.
    g_enable_sensor_log = false;

    // 1. ChArUco 보드 설정 (lane_cpp와 동일하게 FIX)
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();

    // 2. DDS 수신기 초기화
    SensorDataSubscriber imgSub;
    if (!imgSub.init()) {
        cerr << "[에러] DDS 수신기를 초기화할 수 없습니다." << endl;
        return -1;
    }

    // 3. 웹 대시보드 스트리밍용 퍼블리셔
    CameraPublisher camPub;
    camPub.init();

    vector<vector<Point2f>> allCharucoCorners;
    vector<vector<int>> allCharucoIds;
    vector<Mat> allImgs;
    Size imgSize;

    cout << "=================================================" << endl;
    cout << " [정밀 보정기 V6.0 - 민감도 강화 모드] " << endl;
    cout << " >> DICT_4X4_50 고정 (lane_cpp 호환)" << endl;
    cout << " >> 코너 4개 이상 발견 시 즉시 자동 캡처" << endl;
    cout << "=================================================" << endl;

    string last_decoded_img = "";
    auto last_capture_time = chrono::steady_clock::now();
    const int TARGET_FRAMES = 20;

    while (true) {
        auto ctrl = imgSub.getControlState();
        string base64_img = ctrl.latest_image_base64;
        if (base64_img == "" || base64_img == last_decoded_img) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        last_decoded_img = base64_img;
        string decoded = base64_decode(base64_img);
        vector<uchar> data(decoded.begin(), decoded.end());
        Mat frame = imdecode(data, IMREAD_COLOR);
        if (frame.empty()) continue;
        imgSize = frame.size();
        Mat frameCopy;
        frame.copyTo(frameCopy);

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        
        // 탐지
        aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, params);
        
        if (markerIds.size() > 0) {
            vector<Point2f> charucoCorners;
            vector<int> charucoIds;
            aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, board, charucoCorners, charucoIds);
            
            // 실시간 상태 출력 (\r 사용하여 도배 방지)
            cout << "\r[상태] 마커 인식 중... 현재 발견된 코너: " << charucoIds.size() << "개    " << flush;

            if (charucoIds.size() >= 4) {
                aruco::drawDetectedMarkers(frameCopy, markerCorners, markerIds);
                aruco::drawDetectedCornersCharuco(frameCopy, charucoCorners, charucoIds, Scalar(255, 0, 0));
                
                auto now = chrono::steady_clock::now();
                double diff = chrono::duration_cast<chrono::milliseconds>(now - last_capture_time).count() / 1000.0;
                
                if (diff > 1.2 && charucoIds.size() >= 4) { // 캡처 간격 살짝 단축
                    allCharucoCorners.push_back(charucoCorners);
                    allCharucoIds.push_back(charucoIds);
                    allImgs.push_back(frame);
                    last_capture_time = now;
                    cout << "\n[캡처 성공!] 수집량: " << allImgs.size() << " / " << TARGET_FRAMES << "장" << endl;
                    bitwise_not(frameCopy, frameCopy);
                }
            }
        } else {
            cout << "\r[상태] 마커를 찾는 중... (종이를 카메라 중앙에 비춰보세요)          " << flush;
        }

        // 대시보드 스트리밍 시각화
        vector<uchar> buf;
        imencode(".jpg", frameCopy, buf);
        string out_base64 = base64_encode(buf.data(), buf.size());
        camPub.publish(out_base64);

        if (allImgs.size() >= TARGET_FRAMES) {
            cout << "\n[연산 시작] 분석 중입니다..." << endl;
            Mat cameraMatrix, distCoeffs;
            vector<Mat> rvecs, tvecs;
            double repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

            FileStorage fs("camera_calib.xml", FileStorage::WRITE);
            fs << "cameraMatrix" << cameraMatrix;
            fs << "distCoeffs" << distCoeffs;
            fs << "reprojectionError" << repError;
            fs.release();
            
            cout << "[저장 완료] camera_calib.xml 생성 성공! (Error: " << repError << ")" << endl;
            break;
        }
    }

    return 0;
}
