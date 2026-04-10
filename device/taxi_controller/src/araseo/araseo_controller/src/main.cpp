#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <ctime>
#include <csignal>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <chrono>
#include "Motor.hpp"
#include "Lidar.hpp"
#include "CameraPublisher.hpp"
#include "IMU.hpp"
#include "SensorDataSubscriber.h"
#include "SensorDataPublisher.h"
#include "DataLogger.hpp"
#include "Base64.hpp"
#include "ADCSensor.hpp"
#include <cstdio>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

using namespace cv;
using namespace std;
using namespace cv::dnn;
using json = nlohmann::json;

// =========================================================
// [0] 시그널 처리 및 전역 상태
// =========================================================
std::atomic<bool> g_running(true);
extern std::atomic<bool> g_enable_sensor_log;
void signal_handler(int signum) {
    std::cout << "\n[System] Signal received (" << signum << "). Exiting..." << std::endl;
    g_running = false;
}

// [신규] 도움말 출력 함수
void print_help() {
    cout << "\n=========================================" << endl;
    cout << " [ROI Control]" << endl;
    cout << "  - w, s    : Top Position (Up/Down)" << endl;
    cout << "  - a, d    : Top Position (Left/Right)" << endl;
    cout << "  - h, j    : Top Width (Narrow/Wide)" << endl;
    cout << "  - z, x    : Bottom Position (Left/Right)" << endl;
    cout << "  - n, m    : Bottom Width (Narrow/Wide)" << endl;
    cout << " [Driving Control]" << endl;
    cout << "  - i, k    : Speed Up/Down" << endl;
    cout << "  - u, o    : Steer Left/Right (Offset)" << endl;
    cout << "  - v       : Toggle AUTO / MANUAL Mode" << endl;
    cout << "  - r       : RESET Everything to Default" << endl;
    cout << " [System]" << endl;
    cout << "  - p, b, c : Toggle PIP Views (Lidar / BEV / Camera)" << endl;
    cout << "  - t, [    : White Threshold Up/Down" << endl;
    cout << "  - g       : Toggle Adaptive ROI (On/Off)" << endl;
    cout << "  - e       : Toggle Sensor Logs (On/Off)" << endl;
    cout << " [AI Load Control]" << endl;
    cout << "  - -, =    : YOLOX Frame Skip (Down/Up) - '-' for faster AI" << endl;
    cout << "  - ,, .    : UFLD Frame Skip  (Down/Up) - ',' for faster Lanes" << endl;
    cout << " [Help]" << endl;
    cout << "  - ?       : Show THIS Help Message" << endl;
    cout << "  - Space   : EMERGENCY STOP" << endl;
    cout << "=========================================" << endl;
}

// =========================================================
// [1] 유틸리티
// =========================================================
bool polyfit(const vector<Point>& points, int order, vector<double>& coeffs) {
    if (points.size() < order + 1) return false;
    Mat X(points.size(), order + 1, CV_64F);
    Mat Y(points.size(), 1, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        double val = points[i].y;
        double val_sq = val * val;
        X.at<double>(i, 0) = val_sq;
        X.at<double>(i, 1) = val;
        X.at<double>(i, 2) = 1.0;
        Y.at<double>(i, 0) = points[i].x;
    }
    Mat K;
    if (!solve(X, Y, K, DECOMP_SVD)) return false;
    coeffs.clear();
    coeffs.push_back(K.at<double>(0, 0));
    coeffs.push_back(K.at<double>(1, 0));
    coeffs.push_back(K.at<double>(2, 0));
    return true;
}

// =========================================================
// [2] Lane System
// =========================================================
class LaneSystem {
public:
    int white_thr = 140; 
    float roi_top_gap = 0.40f; // [수정] 초기값을 좀 더 좁게 설정
    float roi_bot_gap = 0.06f;
    float roi_h_top = 0.65f;
    float roi_h_bot = 0.95f;
    float roi_top_shift = 0.0f; // [신규] 상단 좌우 이동값
    float roi_bot_shift = 0.0f; // [신규] 하단 좌우 이동값
    bool use_adaptive_roi = true; // [신규] 자동 ROI 조절 사용 여부

    // [신규] 주행 및 안전 설정 (파일 저장 대상)
    int auto_speed = 50;
    int manual_speed = 50;
    float lidar_stop_dist = 0.6f;
    int manual_offset = 0;
    int lane_loss_stop_ms = 700;

    vector<double> left_fit_avg;
    vector<double> right_fit_avg;
    bool has_prev = false;
    const double SMOOTH_FACTOR = 0.9; // 더 부드럽게
    
    // [신규] 윈도우 위치/크기 설정
    int win_x_res = 0, win_y_res = 0, win_w_res = 640, win_h_res = 480;
    int win_x_bin = 400, win_y_bin = 0, win_w_bin = 320, win_h_bin = 240;
    int win_x_warp = 800, win_y_warp = 0, win_w_warp = 320, win_h_warp = 240;
    string config_file = "lane_config.yaml";
    Mat orange_mask; // 저속 구간 감지용 마스크
    bool slow_zone = false; // 현재 저속 구간 여부

    LaneSystem() { load_config(); }

    void load_config() {
        FileStorage fs(config_file, FileStorage::READ);
        if (fs.isOpened()) {
            if(!fs["white_thr"].empty()) fs["white_thr"] >> white_thr;
            if(!fs["roi_top_gap"].empty()) fs["roi_top_gap"] >> roi_top_gap;
            if(!fs["roi_bot_gap"].empty()) fs["roi_bot_gap"] >> roi_bot_gap;
            if(!fs["roi_h_top"].empty()) fs["roi_h_top"] >> roi_h_top;
            if(!fs["roi_top_shift"].empty()) fs["roi_top_shift"] >> roi_top_shift;
            if(!fs["roi_bot_shift"].empty()) fs["roi_bot_shift"] >> roi_bot_shift;
            
            // [신규 항목 로드]
            if(!fs["auto_speed"].empty()) fs["auto_speed"] >> auto_speed;
            if(!fs["manual_speed"].empty()) fs["manual_speed"] >> manual_speed;
            
            // [안전 장치] 비정상적인 속도값 보정
            if (auto_speed < 0) auto_speed = 0;
            if (manual_speed < -100) manual_speed = 0; // -160 같은 극단적 값 방지
            
            if(!fs["lidar_stop_dist"].empty()) fs["lidar_stop_dist"] >> lidar_stop_dist;
            if(!fs["manual_offset"].empty()) fs["manual_offset"] >> manual_offset;
            if(!fs["lane_loss_stop_ms"].empty()) fs["lane_loss_stop_ms"] >> lane_loss_stop_ms;
            
            // [창 위치/크기 로드]
            if(!fs["win_x_res"].empty()) fs["win_x_res"] >> win_x_res;
            if(!fs["win_y_res"].empty()) fs["win_y_res"] >> win_y_res;
            if(!fs["win_w_res"].empty()) fs["win_w_res"] >> win_w_res;
            if(!fs["win_h_res"].empty()) fs["win_h_res"] >> win_h_res;
            if(!fs["win_x_bin"].empty()) fs["win_x_bin"] >> win_x_bin;
            if(!fs["win_y_bin"].empty()) fs["win_y_bin"] >> win_y_bin;
            if(!fs["win_w_bin"].empty()) fs["win_w_bin"] >> win_w_bin;
            if(!fs["win_h_bin"].empty()) fs["win_h_bin"] >> win_h_bin;
            if(!fs["win_x_warp"].empty()) fs["win_x_warp"] >> win_x_warp;
            if(!fs["win_y_warp"].empty()) fs["win_y_warp"] >> win_y_warp;
            if(!fs["win_w_warp"].empty()) fs["win_w_warp"] >> win_w_warp;
            if(!fs["win_h_warp"].empty()) fs["win_h_warp"] >> win_h_warp;

            cout << "[System] Config Loaded (Speed/Dist/Offset/Windows included)." << endl;
        }
    }

    void save_config() {
        FileStorage fs(config_file, FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "white_thr" << white_thr;
            fs << "roi_top_gap" << roi_top_gap;
            fs << "roi_bot_gap" << roi_bot_gap;
            fs << "roi_h_top" << roi_h_top;
            fs << "roi_top_shift" << roi_top_shift;
            fs << "roi_bot_shift" << roi_bot_shift;
            
            // [신규 항목 저장]
            fs << "auto_speed" << auto_speed;
            fs << "manual_speed" << manual_speed;
            fs << "lidar_stop_dist" << lidar_stop_dist;
            fs << "manual_offset" << manual_offset;
            fs << "lane_loss_stop_ms" << lane_loss_stop_ms;
            
            // [창 위치/크기 저장]
            fs << "win_x_res" << win_x_res; fs << "win_y_res" << win_y_res;
            fs << "win_w_res" << win_w_res; fs << "win_h_res" << win_h_res;
            fs << "win_x_bin" << win_x_bin; fs << "win_y_bin" << win_y_bin;
            fs << "win_w_bin" << win_w_bin; fs << "win_h_bin" << win_h_bin;
            fs << "win_x_warp" << win_x_warp; fs << "win_y_warp" << win_y_warp;
            fs << "win_w_warp" << win_w_warp; fs << "win_h_warp" << win_h_warp;

            cout << "\n[System] Config Saved (Speed/Dist/Offset/Windows included)." << endl;
        }
    }

    void reset_config() {
        white_thr = 140;
        roi_top_gap = 0.40f;
        roi_bot_gap = 0.06f;
        roi_h_top = 0.65f;
        roi_h_bot = 0.95f;
        roi_top_shift = 0.0f;
        roi_bot_shift = 0.0f;
        auto_speed = 50;
        manual_speed = 50;
        lidar_stop_dist = 0.5f;
        manual_offset = 0;
        lane_loss_stop_ms = 700;
        save_config();
        cout << "\n[System] All Lane Settings Reset to Default." << endl;
    }

    // [신규] 차선 위치에 따라 ROI 하단 너비를 자동으로 조절하는 알고해리즘
    void update_adaptive_roi(const vector<double>& left_fit, const vector<double>& right_fit, int warped_w) {
        if (!use_adaptive_roi || left_fit.empty() || right_fit.empty()) return;

        double y_eval = 160.0;
        double x_l = left_fit[0]*y_eval*y_eval + left_fit[1]*y_eval + left_fit[2];
        double x_r = right_fit[0]*y_eval*y_eval + right_fit[1]*y_eval + right_fit[2];

        bool expanded = false;
        if (x_r > warped_w * 0.85 || x_l < warped_w * 0.15) {
            roi_bot_gap -= 0.002f;
            expanded = true;
        } 
        else if (x_r < warped_w * 0.70 && x_l > warped_w * 0.30) {
            if (roi_bot_gap < 0.15f) roi_bot_gap += 0.001f;
        }

        if (roi_bot_gap < -1.00f) roi_bot_gap = -1.00f;
        
        static auto last_print = chrono::steady_clock::now();
        if (expanded && chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - last_print).count() >= 1) {
            cout << "\r[Adaptive ROI] Expanding Bottom Width (Gap: " << fixed << setprecision(3) << roi_bot_gap << ")    " << flush;
            last_print = chrono::steady_clock::now();
        }
    }

    void sync_window_positions(uint32_t frameCount) {
        bool changed = false;
        try {
            // [복구] GDK_BACKEND=x11 환경에서 더 잘 동작하도록 getWindowImageRect 사용
            Rect r_res = getWindowImageRect("LaneResult");
            Rect r_bin = getWindowImageRect("Binary");
            Rect r_warp = getWindowImageRect("Warped");

            auto update_pos_size = [&](Rect& r, int& cur_x, int& cur_y, int& cur_w, int& cur_h, bool is_main) {
                if (r.width <= 0 || r.height <= 0) return false; 
                
                // 메인 창은 초기에 윈도우 매니저에 의해 강제 배치될 수 있으므로 
                // 초기 90프레임(약 3~4초) 동안은 이동을 무시하여 데이터 오염 방지
                if (is_main && frameCount < 90) return false;

                if (r.x != cur_x || r.y != cur_y || r.width != cur_w || r.height != cur_h) {
                    cur_x = r.x; cur_y = r.y;
                    cur_w = r.width; cur_h = r.height;
                    return true;
                }
                return false;
            };

            if (update_pos_size(r_res, win_x_res, win_y_res, win_w_res, win_h_res, true)) changed = true;
            if (update_pos_size(r_bin, win_x_bin, win_y_bin, win_w_bin, win_h_bin, false)) changed = true;
            if (update_pos_size(r_warp, win_x_warp, win_y_warp, win_w_warp, win_h_warp, false)) changed = true;

        } catch (...) { return; }

        if (changed) {
            save_config();
            cout << "\n[System] Window positions updated & saved: Result(" << win_x_res << "," << win_y_res << ")" << endl;
        }
    }



    Mat thresholding(const Mat& img) {
        Mat hls, gray;
        // HLS 변환
        cvtColor(img, hls, COLOR_BGR2HLS);
        vector<Mat> channels;
        split(hls, channels);

        // 1. White Filter (L channel + Threshold)
        // 임계값을 낮춰도 안 보인다면 L 채널 밝기 자체가 낮은 것일 수 있음
        Mat white_mask;
        threshold(channels[1], white_mask, white_thr, 255, THRESH_BINARY);

        // 2. Yellow Filter (H channel)
        Mat yellow_mask;
        inRange(hls, Scalar(15, 30, 100), Scalar(35, 204, 255), yellow_mask);

        // 3. Sobel Edge (X축) - 차선 윤곽선 감지
        Mat sobel_x, abs_sobel, sobel_mask;
        Sobel(channels[1], sobel_x, CV_64F, 1, 0, 3); // 커널 크기 3으로 축소 (정밀도 향상)
        convertScaleAbs(sobel_x, abs_sobel);
        // Normalize
        double maxVal; 
        minMaxLoc(abs_sobel, NULL, &maxVal);
        abs_sobel.convertTo(abs_sobel, CV_8U, 255.0 / (maxVal + 0.001));
        inRange(abs_sobel, 30, 255, sobel_mask);

        // [통합]
        Mat combined;
        // White or Yellow or Edge
        bitwise_or(white_mask, yellow_mask, combined);
        bitwise_or(combined, sobel_mask, combined);
        
        // 4. "저속 구간" 감지 (주황색/빨간색 계열)
        // HLS에서 주황색은 보통 5-15 범위
        inRange(hls, Scalar(0, 50, 100), Scalar(20, 255, 255), orange_mask);
        
        // 특정 면적 이상의 주황색이 감지되면 저속 구간으로 판단
        slow_zone = (countNonZero(orange_mask) > (img.cols * img.rows * 0.1)); // 10% 이상일 때
        
        return combined;
    }

    vector<Point2f> get_roi_points(int w, int h, Point2f offset = Point2f(0,0)) {
        // [안전장치] 너비 비율이 음수가 되지 않도록 최소값 제한
        float safe_top_gap = max(0.01f, roi_top_gap);
        
        // [수정] 하단은 무조건 상단보다 넓거나 같아야 함 (형태 무결성)
        float safe_bot_gap = max(safe_top_gap, roi_bot_gap);
        
        float center_x = w / 2.0f;
        float top_w = w * safe_top_gap;
        float bot_w = w * safe_bot_gap;
        float t_shift = w * roi_top_shift;
        float b_shift = w * roi_bot_shift;
        
        return {
            Point2f(center_x - top_w / 2 + t_shift, h * roi_h_top) + offset,
            Point2f(center_x + top_w / 2 + t_shift, h * roi_h_top) + offset,
            Point2f(center_x + bot_w / 2 + b_shift, h * roi_h_bot) + offset,
            Point2f(center_x - bot_w / 2 + b_shift, h * roi_h_bot) + offset
        };
    }

    Mat bird_eye_view(const Mat& img, Mat& Minv, Mat& M) {
        int w = img.cols;
        int h = img.rows;
        vector<Point2f> src = get_roi_points(w, h);
        
        // [중요] BEV 변환 후 차선이 11자가 되도록 dst 좌표 고정
        float offset_x = w * 0.25f; // 좌우 여백 25%
        vector<Point2f> dst = {
            Point2f(offset_x, 0),
            Point2f(w - offset_x, 0),
            Point2f(w - offset_x, (float)h),
            Point2f(offset_x, (float)h)
        };

        M = getPerspectiveTransform(src, dst);
        Minv = getPerspectiveTransform(dst, src);
        Mat warped;
        warpPerspective(img, warped, M, Size(w, h), INTER_LINEAR);
        return warped;
    }

    void sliding_window(const Mat& binary_warped, vector<double>& left_fit, vector<double>& right_fit) {
        int rows = binary_warped.rows;
        int cols = binary_warped.cols;

        // [수정] 강제 리턴 조건 제거 (최대한 찾도록 변경)
        // 화면이 너무 밝아도 일단 시도함

        Mat bottom_half = binary_warped(Rect(0, rows / 2, cols, rows / 2));
        Mat hist;
        reduce(bottom_half, hist, 0, REDUCE_SUM, CV_32S);

        int midpoint = hist.cols / 2;
        Point left_max, right_max;
        minMaxLoc(hist.colRange(0, midpoint), NULL, NULL, NULL, &left_max);
        minMaxLoc(hist.colRange(midpoint, hist.cols), NULL, NULL, NULL, &right_max);

        int leftx_current = left_max.x;
        int rightx_current = right_max.x + midpoint;
        
        // 히스토그램 피크가 너무 약하면(검은 화면) 이전 값 사용
        if (hist.at<int>(0, leftx_current) < 50 && hist.at<int>(0, rightx_current) < 50) {
            if(has_prev) {
                left_fit = left_fit_avg;
                right_fit = right_fit_avg;
            }
            return; 
        }

        int nwindows = 9;
        int window_height = rows / nwindows;
        int margin = 50; 
        int minpix = 15; // 민감도 상승

        vector<Point> left_lane_inds, right_lane_inds;
        left_lane_inds.reserve(5000);
        right_lane_inds.reserve(5000);

        for (int w = 0; w < nwindows; ++w) {
            int win_y_low = rows - (w + 1) * window_height;
            int l_x_start = max(0, leftx_current - margin);
            int l_x_end = min(cols, leftx_current + margin);
            int r_x_start = max(0, rightx_current - margin);
            int r_x_end = min(cols, rightx_current + margin);
            
            if (l_x_end > l_x_start) {
                Mat l_roi = binary_warped(Rect(l_x_start, win_y_low, l_x_end - l_x_start, window_height));
                vector<Point> pts;
                findNonZero(l_roi, pts);
                int sum_x = 0;
                for(auto& p : pts) { p.x += l_x_start; p.y += win_y_low; sum_x += p.x; }
                left_lane_inds.insert(left_lane_inds.end(), pts.begin(), pts.end());
                if (pts.size() > minpix) leftx_current = sum_x / pts.size();
            }

            if (r_x_end > r_x_start) {
                Mat r_roi = binary_warped(Rect(r_x_start, win_y_low, r_x_end - r_x_start, window_height));
                vector<Point> pts;
                findNonZero(r_roi, pts);
                int sum_x = 0;
                for(auto& p : pts) { p.x += r_x_start; p.y += win_y_low; sum_x += p.x; }
                right_lane_inds.insert(right_lane_inds.end(), pts.begin(), pts.end());
                if (pts.size() > minpix) rightx_current = sum_x / pts.size();
            }
        }

        vector<double> l_new, r_new;
        bool l_found = polyfit(left_lane_inds, 2, l_new);
        bool r_found = polyfit(right_lane_inds, 2, r_new);

        // [수정] 한쪽 차선만 발견되었을 때 반대편 가상 차선 생성 (우측통행 보조)
        if (l_found && !r_found) {
            r_new = l_new;
            r_new[2] += 200; // 320x240 기준 약 200px 간격 가정
            r_found = true;
        } else if (!l_found && r_found) {
            l_new = r_new;
            l_new[2] -= 200;
            l_found = true;
        }

        if (l_found && r_found) {
            if (!has_prev) {
                left_fit_avg = l_new;
                right_fit_avg = r_new;
                has_prev = true;
            } else {
                for(int i=0; i<3; ++i) {
                    left_fit_avg[i] = left_fit_avg[i] * SMOOTH_FACTOR + l_new[i] * (1.0 - SMOOTH_FACTOR);
                    right_fit_avg[i] = right_fit_avg[i] * SMOOTH_FACTOR + r_new[i] * (1.0 - SMOOTH_FACTOR);
                }
            }
        }
        
        // 이전 값이 있으면 그걸 줌 (Lane Lost 방지)
        if (has_prev) {
            left_fit = left_fit_avg;
            right_fit = right_fit_avg;
        }
    }

    // [수정] 캔버스 오프셋을 고려한 가시화 로직
    Mat draw_lane_and_info(Mat& canvas, const Mat& binary_warped, 
                          const vector<double>& left_fit, const vector<double>& right_fit, 
                          const Mat& Minv_small, float& angle, Point2f offset) {
        
        if (left_fit.empty() || right_fit.empty()) return canvas;

        int h = binary_warped.rows;
        int w = binary_warped.cols;
        
        // 투영용 마스크 (원본 크기)
        Mat color_warp = Mat::zeros(binary_warped.size(), CV_8UC3);
        vector<Point> pts_left, pts_right;
        
        for (int y = 0; y < h; y += 4) {
            double x_l = left_fit[0] * y * y + left_fit[1] * y + left_fit[2];
            double x_r = right_fit[0] * y * y + right_fit[1] * y + right_fit[2];
            pts_left.push_back(Point(x_l, y));
            pts_right.push_back(Point(x_r, y));
        }
        
        vector<Point> pts_poly;
        vector<Point> pts_right_rev = pts_right;
        reverse(pts_right_rev.begin(), pts_right_rev.end());
        pts_poly.insert(pts_poly.end(), pts_left.begin(), pts_left.end());
        pts_poly.insert(pts_poly.end(), pts_right_rev.begin(), pts_right_rev.end());
        fillPoly(color_warp, vector<vector<Point>>{pts_poly}, Scalar(0, 200, 0));
        
        polylines(color_warp, pts_left, false, Scalar(0, 255, 255), 10);
        polylines(color_warp, pts_right, false, Scalar(255, 255, 255), 10);

        // 5. 원본 이미지에 차선 합성 (오프셋 보정)
        Mat newwarp = Mat::zeros(canvas.size(), canvas.type());
        
        // 캔버스 오프셋을 반영하기 위해 변환 행렬에 평행이동 추가
        Mat T = (Mat_<double>(3,3) << 1, 0, offset.x, 0, 1, offset.y, 0, 0, 1);
        Mat M_offset = T * Minv_small;
        
        warpPerspective(color_warp, newwarp, M_offset, canvas.size());
        
        canvas = canvas + newwarp;

        // 조향각 계산
        double lane_center = (pts_left.back().x + pts_right.back().x) / 2.0;
        angle = (lane_center - (w / 2.0)) * 0.5; 

        return canvas;
    }

    void draw_roi_on_img(Mat& canvas, Point2f offset) {
        vector<Point2f> src = get_roi_points(320, 240, offset); // 원본 크기 기준 좌표
        for (int i = 0; i < 4; i++) {
            // 오프셋을 더해 화면 밖(검은 여백)까지 선이 그려지도록 함
            line(canvas, src[i], src[(i+1)%4], Scalar(0, 0, 255), 2);
        }
    }

    void draw_steering_wheel(Mat& canvas, float angle, Point center = Point(-1, -1)) {
        if (center.x == -1) center = Point(canvas.cols / 2, canvas.rows - 70);
        int radius = 50;
        circle(canvas, center, radius, Scalar(200, 200, 200), 5);
        double rad = -angle * CV_PI / 180.0;
        Point p1(center.x - radius * sin(rad), center.y - radius * cos(rad));
        line(canvas, center, p1, Scalar(200, 200, 200), 5);
    }

    Mat draw_lidar_map(Lidar& lidar, int size = 300) {
        Mat map = Mat::zeros(Size(size, size), CV_8UC3);
        int cx = size / 2;
        int cy = size / 2;
        float scale = size / 8.0f; // 8m full range (4m radius)

        // Draw circles for 100cm, 200cm, 300cm
        for (int r = 1; r <= 3; r++) {
            circle(map, Point(cx, cy), r * scale, Scalar(50, 50, 50), 1);
            putText(map, to_string(r) + "m", Point(cx + 5, cy - r * scale + 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(80, 80, 80));
        }
        
        // Draw Sector (Proportional to detection_angle)
        float det_angle = lidar.get_detection_angle();
        vector<Point> sector;
        sector.push_back(Point(cx, cy));
        for(float a = -det_angle; a <= det_angle; a += 1.0f) {
            float rad = (a - 90.0f) * CV_PI / 180.0f;
            sector.push_back(Point(cx + 4.0f * scale * cos(rad), cy + 4.0f * scale * sin(rad)));
        }
        vector<vector<Point>> fill_pts = {sector};
        Mat overlay = map.clone();
        fillPoly(overlay, fill_pts, Scalar(0, 100, 100));
        addWeighted(map, 0.7, overlay, 0.3, 0, map);

        auto nodes = lidar.get_scan_data();
        int points_drawn = 0;
        for (auto& node : nodes) {
            if (node.dist_mm_q2 == 0) continue;
            float angle = node.angle_z_q14 * 90.f / (1 << 14);
            float dist = node.dist_mm_q2 / 4.0f / 1000.0f;
            
            float rad = (angle - 90.0f) * CV_PI / 180.0f;
            int px = cx + (int)(dist * scale * cos(rad));
            int py = cy + (int)(dist * scale * sin(rad));

            if (px >= 0 && px < size && py >= 0 && py < size) {
                // [수정] ROI 범위(detection_angle) 안의 점만 색상 표시, 밖은 어두운 회색
                bool in_roi = (angle > (360.0f - lidar.get_detection_angle()) || angle < lidar.get_detection_angle());
                Scalar color;
                if (in_roi) {
                    color = (dist < 0.6f) ? Scalar(0, 0, 255) : Scalar(0, 255, 255); // 가깝워지면 빨간색, 평시는 노란색
                } else {
                    color = Scalar(60, 60, 60); // [수정] ROI 밖은 어두운 회색 (회피 필요 없음)
                }
                circle(map, Point(px, py), 2, color, -1);
                points_drawn++;
            }
        }
        // Vehicle Center (Green Rectangle)
        rectangle(map, Rect(cx-5, cy-8, 10, 16), Scalar(0, 255, 0), -1);
        
        // 거리 정보 텍스트 추가
        float cur_dist = lidar.get_front_distance();
        string dist_str = (cur_dist > 9.0f) ? "INF" : cv::format("%.2fm", cur_dist);
        putText(map, dist_str, Point(cx - 30, cy + 30), FONT_HERSHEY_SIMPLEX, 0.6, (cur_dist < 0.6f ? Scalar(0,0,255) : Scalar(0,255,0)), 2);

        // 디버그 로그 (주기적으로 출력)
        static int drw_cnt = 0;
        if (drw_cnt++ % 50 == 0) {
            std::cout << "[Radar] Drawn " << points_drawn << " points. FrontDist: " << dist_str << std::endl;
        }

        return map;
    }
};

class YOLOXDetector {
    Net net;
    const float INPUT_W = 640.0;
    const float INPUT_H = 640.0;
    const float CONF_THRES = 0.25; 
    const float NMS_THRES = 0.45;
    Mat blob;
    vector<string> classes;

    std::thread worker;
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> running{true};
    Mat current_frame;
    bool frame_ready = false;

    vector<int> last_class_ids;
    vector<float> last_confidences;
    vector<Rect> last_boxes;

    // [신규] 메인 루프용 스냅샷 변수 (동기화 문제 해결)
    vector<int> loop_class_ids;
    vector<float> loop_confidences;
    vector<Rect> loop_boxes;

public:
    YOLOXDetector(string model_path) {
        classes = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
                   "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
                   "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                   "skis", "snowboard", "sports ball", "Kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
                   "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
                   "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
                   "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
                   "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
                   "hair drier", "toothbrush"};

        net = readNetFromONNX(model_path);
        net.setPreferableBackend(DNN_BACKEND_OPENCV);
        net.setPreferableTarget(DNN_TARGET_CPU);

        // 인식 스레드 시작
        worker = std::thread(&YOLOXDetector::worker_thread, this);
    }

    ~YOLOXDetector() {
        running = false;
        cv.notify_all();
        if (worker.joinable()) worker.join();
    }

    void worker_thread() {
        while (running) {
            Mat frame_to_process;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this] { return !running || frame_ready; });
                if (!running) break;
                
                current_frame.copyTo(frame_to_process);
                frame_ready = false;
            }

            if (!frame_to_process.empty()) {
                perform_detect(frame_to_process);
            }
            std::this_thread::yield();
        }
    }

    void push_frame(const Mat& frame) {
        {
            std::unique_lock<std::mutex> lock(mtx);
            // 이미 처리 중이거나 대기 중인 프레임이 있으면 스킵하여 최신성 유지
            if (frame_ready) return; 
            frame.copyTo(current_frame);
            frame_ready = true;
        }
        cv.notify_one();
    }

    void perform_detect(const Mat& frame) {
        blobFromImage(frame, blob, 1.0, Size(INPUT_W, INPUT_H), Scalar(114, 114, 114), true, false);
        net.setInput(blob);
        Mat outputs = net.forward();

        // [Robust] 형상에 관계없이 [N, 85] 행렬로 변환
        Mat detectionMat = outputs.reshape(1, outputs.total() / 85);
        
        int total_anchors = detectionMat.rows; // 8400
        float x_scale = (float)frame.cols / INPUT_W;
        float y_scale = (float)frame.rows / INPUT_H;

        vector<int> class_ids;
        vector<float> confidences;
        vector<Rect> boxes;

        // YOLOX 전용 디코딩 (Strides: 8, 16, 32)
        int strides[] = {8, 16, 32};
        int grid_sizes[] = {80, 40, 20}; // 640/8, 640/16, 640/32
        int data_ptr = 0;

        for (int s = 0; s < 3; ++s) {
            int stride = strides[s];
            int grid_h = grid_sizes[s];
            int grid_w = grid_sizes[s];

            for (int g_y = 0; g_y < grid_h; ++g_y) {
                for (int g_x = 0; g_x < grid_w; ++g_x) {
                    float* data = detectionMat.ptr<float>(data_ptr);
                    float obj_conf = data[4];
                    
                    if (obj_conf >= CONF_THRES) {
                        float* classes_scores = data + 5;
                        float max_class_score = 0.0;
                        int class_id = 0;
                        for (int k = 0; k < 80; ++k) {
                            if (classes_scores[k] > max_class_score) {
                                max_class_score = classes_scores[k];
                                class_id = k;
                            }
                        }
                        float final_score = obj_conf * max_class_score;
                        if (final_score > CONF_THRES) {
                            // Raw data decoding (grid offset + stride)
                            float cx = (data[0] + g_x) * stride;
                            float cy = (data[1] + g_y) * stride;
                            float w = exp(data[2]) * stride;
                            float h = exp(data[3]) * stride;

                            int i_width = cvRound(w * x_scale);
                            int i_height = cvRound(h * y_scale);
                            int i_left = cvRound((cx - 0.5 * w) * x_scale);
                            int i_top = cvRound((cy - 0.5 * h) * y_scale);

                            // Clipping
                            i_left = std::max(0, std::min(i_left, frame.cols - 1));
                            i_top = std::max(0, std::min(i_top, frame.rows - 1));
                            i_width = std::max(1, std::min(i_width, frame.cols - i_left));
                            i_height = std::max(1, std::min(i_height, frame.rows - i_top));

                            class_ids.push_back(class_id);
                            confidences.push_back(final_score);
                            boxes.push_back(Rect(i_left, i_top, i_width, i_height));
                        }
                    }
                    data_ptr++;
                }
            }
        }

        vector<int> nms_result;
        NMSBoxes(boxes, confidences, CONF_THRES, NMS_THRES, nms_result);

        {
            std::lock_guard<std::mutex> lock(mtx);
            last_class_ids.clear();
            last_confidences.clear();
            last_boxes.clear();
            for (int idx : nms_result) {
                last_class_ids.push_back(class_ids[idx]);
                last_confidences.push_back(confidences[idx]);
                last_boxes.push_back(boxes[idx]);
            }
        }
    }

    // [신규] 루프 시작 시점에 상태를 고정하는 메서드
    void update_snapshot() {
        std::lock_guard<std::mutex> lock(mtx);
        loop_class_ids = last_class_ids;
        loop_confidences = last_confidences;
        loop_boxes = last_boxes;
    }

    void draw(Mat& frame, const vector<Point2f>& roi = {}) {
        for (size_t i = 0; i < loop_boxes.size(); ++i) {
            const Rect& box = loop_boxes[i];
            int cls_id = loop_class_ids[i];
            float conf = loop_confidences[i];
            
            // [수정] ROI 영역과 BBox가 겹치는 비율 계산 (20% 임계값)
            bool is_significant = true;
            if (!roi.empty() && box.area() > 0) {
                // 상자 내부에서 ROI와 겹치는 픽셀 계산
                Mat mask = Mat::zeros(box.size(), CV_8U);
                vector<Point> roi_int;
                for (auto& p : roi) {
                    roi_int.push_back(Point(p.x - box.x, p.y - box.y));
                }
                fillPoly(mask, {roi_int}, Scalar(255));
                int overlap_pixels = countNonZero(mask);
                float overlap_ratio = (float)overlap_pixels / box.area();
                is_significant = (overlap_ratio > 0.20f);
            }

            string label = classes[cls_id] + ":" + to_string(int(conf * 100)) + "%";
            Scalar color;
            Scalar gray(100, 100, 100);

            // 1. Vehicles (Car, Bus, Truck)
            if (cls_id == 2 || cls_id == 5 || cls_id == 7) { 
                 color = is_significant ? Scalar(255, 100, 0) : gray; 
                 rectangle(frame, box, color, 2);
                 putText(frame, label, Point(box.x, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

                 if (is_significant && box.width > frame.cols * 0.2) {
                     putText(frame, "WARNING", Point(box.x, box.y - 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                     rectangle(frame, box, Scalar(0, 0, 255), 3);
                 }
            }
            // 2. Traffic Light
            else if (cls_id == 9) {
                color = is_significant ? Scalar(0, 255, 255) : gray;
                rectangle(frame, box, color, 2);
                putText(frame, label, Point(box.x, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
            // 3. Person (NEW)
            else if (cls_id == 0) {
                color = is_significant ? Scalar(0, 0, 255) : gray;
                rectangle(frame, box, color, 2);
                putText(frame, "PERSON:" + to_string(int(conf * 100)) + "%", Point(box.x, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, is_significant ? 2 : 1);
            }
            // 4. Animals (NEW)
            else if (cls_id >= 14 && cls_id <= 23) {
                color = is_significant ? Scalar(255, 255, 0) : gray;
                string animal_name = classes[cls_id];
                if (!animal_name.empty()) animal_name[0] = toupper(animal_name[0]);
                
                rectangle(frame, box, color, 2);
                putText(frame, animal_name + ":" + to_string(int(conf * 100)) + "%", Point(box.x, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, is_significant ? 2 : 1);
            }
        }
    }


    bool is_person_or_animal_detected() {
        for (int cls_id : loop_class_ids) {
            // person(0) or animals(14-23)
            if (cls_id == 0 || (cls_id >= 14 && cls_id <= 23)) return true;
        }
        return false;
    }

    bool is_person_detected() {
        for (int cls_id : loop_class_ids) {
            if (cls_id == 0) return true;
        }
        return false;
    }

    bool is_animal_detected() {
        for (int cls_id : loop_class_ids) {
            if (cls_id >= 14 && cls_id <= 23) return true;
        }
        return false;
    }

    bool is_car_detected() {
        for (int cls_id : loop_class_ids) {
            // car(2), bus(5), truck(7) 도 차량으로 간주하여 저장
            if (cls_id == 2 || cls_id == 5 || cls_id == 7) return true;
        }
        return false;
    }

    string get_detection_names() {
        string names = "";
        for (int cls_id : loop_class_ids) {
            if (cls_id == 0 || (cls_id >= 14 && cls_id <= 23)) {
                if (!names.empty()) names += "_";
                names += classes[cls_id];
            }
        }
        return (names.empty() ? "none" : names);
    }

    // [신규] 현재 스냅샷 결과를 JSON으로 반환
    json get_json_results() {
        json j = json::array();
        for (size_t i = 0; i < loop_boxes.size(); ++i) {
            json det;
            det["label"] = classes[loop_class_ids[i]];
            det["class_id"] = loop_class_ids[i];
            det["confidence"] = loop_confidences[i];
            det["bbox"] = {loop_boxes[i].x, loop_boxes[i].y, loop_boxes[i].width, loop_boxes[i].height};
            j.push_back(det);
        }
        return j;
    }
};

// [신규] 탐지 결과 JSON 저장 헬퍼
void save_detection_json(const string& img_path, const json& detections) {
    string json_path = img_path;
    size_t lastdot = json_path.find_last_of(".");
    if (lastdot != string::npos) json_path = json_path.substr(0, lastdot);
    json_path += ".json";
    
    ofstream o(json_path);
    if (o.is_open()) {
        json j;
        j["image"] = img_path;
        j["timestamp"] = time(0);
        j["detections"] = detections;
        o << std::setw(4) << j << std::endl;
        o.close();
    }
}

// =========================================================
// [신규] ArUco 기반 맵 측위기 (Map Localizer)
// =========================================================
class ArUcoLocalizer {
public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Mat cameraMatrix, distCoeffs;
    bool has_calib = false;

    // [글로벌 환경 측정치]
    // 맵의 실제 가로/세로 길이 (단위: 미터)
    float MAP_WIDTH = 1.86f;  // 1860 mm
    float MAP_HEIGHT = 1.40f; // 1400 mm

    // 사진(맵) 상의 코너 마커 ID와 절대 좌표 (Top-Left 0번을 원점으로 가정)
    std::map<int, Point2f> globalMarkerMap = {
        {0, Point2f(0.0f, 0.0f)},                 // Top-Left (원점)
        {1, Point2f(MAP_WIDTH, 0.0f)},            // Top-Right
        {2, Point2f(MAP_WIDTH, MAP_HEIGHT)},      // Bottom-Right
        {3, Point2f(0.0f, MAP_HEIGHT)}            // Bottom-Left
    };

    // [신규] 중복 출력 방지용 이전 좌표 저장소
    std::map<int, Point3f> last_printed_pos;

    ArUcoLocalizer() {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        parameters = cv::aruco::DetectorParameters::create();

        // [신규] 캘리브레이션 파일 로드
        FileStorage fs("camera_calib.xml", FileStorage::READ);
        if (fs.isOpened()) {
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"] >> distCoeffs;
            has_calib = true;
            cout << "[ArUco] Precise Calibration file loaded. Correcting lens distortion." << endl;
        } else {
            has_calib = false;
            cout << "[ArUco] Warning: No camera_calib.xml found. Using approximate parameters." << endl;
        }
    }

    void detectAndDraw(Mat& frame) {
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;
        
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            
            // [수정] 캘리브레이션 데이터 사용 (없으면 임시 파라미터 생성)
            Mat camMat = cameraMatrix;
            Mat distMat = distCoeffs;
            if (!has_calib) {
                double fx = frame.cols * 0.8;
                camMat = (Mat_<double>(3,3) << fx, 0, frame.cols/2.0, 0, fx, frame.rows/2.0, 0, 0, 1.0);
                distMat = Mat::zeros(4, 1, CV_64F);
            }

            // 실제 맵에 부착된 마커의 한 변의 길이 (단위: 미터 m) - 45mm
            float markerLength = 0.045f; 

            // ArUco 마커의 3D 공간 상 모서리 로컬 좌표체계 (좌상, 우상, 우하, 좌하)
            vector<Point3f> objPoints;
            float half = markerLength / 2.0f;
            objPoints.push_back(Point3f(-half,  half, 0));
            objPoints.push_back(Point3f( half,  half, 0));
            objPoints.push_back(Point3f( half, -half, 0));
            objPoints.push_back(Point3f(-half, -half, 0));
            
            for (size_t i = 0; i < markerIds.size(); i++) {
                // PnP(Perspective-n-Point)로 카메라와 마커 간의 3D 회전(rvec)과 위치(tvec) 도출
                Mat rvec, tvec;
                solvePnP(objPoints, markerCorners[i], camMat, distMat, rvec, tvec);
                
                // 마커 중심에 3D X, Y, Z 방향 축(Axis) 그리기
                cv::drawFrameAxes(frame, camMat, distMat, rvec, tvec, markerLength * 0.5f);
                
                // 카메라로부터 마커까지의 실제 Z거리 (미터) 및 X 이동량
                double dist_x = tvec.at<double>(0, 0);
                double dist_z = tvec.at<double>(2, 0); 
                
                // [역변환 계산] 마커 기준 카메라(로봇)의 상대적 3D 위치 산출
                Mat R;
                Rodrigues(rvec, R); // 회전 벡터를 3x3 회전 행렬로 변환
                Mat R_inv = R.t(); // 역행렬 (Transposed)
                Mat cam_pos = -R_inv * tvec; // 로봇의 중심 좌표 [x, y, z] (마커 원점 기준)
                
                double cam_x = cam_pos.at<double>(0, 0);
                double cam_y = cam_pos.at<double>(1, 0);
                double cam_z = cam_pos.at<double>(2, 0); // 로봇 지붕에서 바닥까지의 대략적 높이
                
                // [수정] 1cm 이상 이동 시에만 즉시 로그 출력 (Timer 제거로 딜레이 없음)
                bool pos_changed = true;
                if (last_printed_pos.find(markerIds[i]) != last_printed_pos.end()) {
                    Point3f last_pos = last_printed_pos[markerIds[i]];
                    double dist_movement = std::sqrt(std::pow(cam_x - last_pos.x, 2) + 
                                                     std::pow(cam_y - last_pos.y, 2) + 
                                                     std::pow(cam_z - last_pos.z, 2));
                    if (dist_movement < 0.01) pos_changed = false; // 1cm 미만 움직임은 무시
                }
                
                // 마커 중심 픽셀 정보 추출
                Point2f center(0, 0);
                for(int c = 0; c < 4; c++) center += markerCorners[i][c];
                center.x /= 4; center.y /= 4;
                
                putText(frame, "ID:" + to_string(markerIds[i]) + " D:" + to_string(dist_z).substr(0,4) + "m", 
                        Point(center.x - 30, center.y - 40), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                        
                if (pos_changed) {
                    last_printed_pos[markerIds[i]] = Point3f(cam_x, cam_y, cam_z); // 출력 위치 갱신
                    
                    std::cout << "\n[ArUco 3D] Camera Pos from ID " << markerIds[i] 
                              << " -> X: " << fixed << setprecision(2) << cam_x 
                              << "m, Y: " << cam_y 
                              << "m, Z(Height): " << cam_z << "m" << std::endl;
                              
                    // 글로벌 좌표 계산 로직 (임시)
                    if (globalMarkerMap.find(markerIds[i]) != globalMarkerMap.end()) {
                        double global_x = globalMarkerMap[markerIds[i]].x + cam_x;
                        double global_y = globalMarkerMap[markerIds[i]].y + cam_y;
                        std::cout << "[RViz2 Map] Robot Global X: " << global_x << "m, Y: " << global_y << "m" << std::endl;
                    }
                }
            }
        }
    }
};

// =========================================================
// [신규] 동적 위치 추정기 (Dynamic Odometry Estimator)
// =========================================================
class DynamicOdometryEstimator {
public:
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0; // 맵 기준 절대 각도 (도 Degree)
    
    // IMU 센서가 바라보는 방향과 지도(RViz) 상의 0도 간의 차이
    double imu_yaw_offset = 0.0; 
    bool is_initialized = false;

    // 1. ArUco 정답지 획득 시 동기화 (Sync with Ground Truth)
    void resetFromArUco(double abs_x, double abs_y, double abs_yaw, double raw_imu_yaw) {
        x = abs_x;
        y = abs_y;
        yaw = abs_yaw;
        
        // 축 정렬: 맵의 절대 각도에서 현재 IMU가 말하는 각도를 뺌
        imu_yaw_offset = abs_yaw - raw_imu_yaw; 
        is_initialized = true;
    }

    // 2. 고속 이동 등 마커 소실 시 궤적 추정 (Dead Reckoning)
    void updateOdometry(double dt_sec, double speed_m_s, double raw_imu_yaw) {
        if (!is_initialized) return; 

        // 오프셋을 반영한 진짜 방향 산출
        yaw = raw_imu_yaw + imu_yaw_offset;
        
        // 속도 벡터 분해 (Kinematic Update) -> X 전진, Y 좌우
        x += speed_m_s * cos(yaw * M_PI / 180.0) * dt_sec;
        y += speed_m_s * sin(yaw * M_PI / 180.0) * dt_sec;
    }
};

int main() {
    cv::setNumThreads(1);
    
    // [System] Create detection folders under training_data
    system("mkdir -p training_data/person training_data/animal training_data/car training_data/event");
    
    cout << "=========================================" << endl;
    cout << " [System] PHASE 1: Hardware Initialization" << endl;
    cout << "=========================================" << endl;
    
    // 0. 하드웨어 안정화를 위한 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    LaneSystem laneSys;
    Motor motor;
    Lidar lidar;

    // 1. Lidar 초기화 (Thread 부하 전)
    if (lidar.init()) {
        lidar.start_scan();
        cout << "[Phase 1] Lidar: OK" << endl;
    } else {
        cout << "[Phase 1] Lidar: FAILED (Running without AEB)" << endl;
    }

    // 2. 모터 초기화
    if (motor.init()) {
        cout << "[Phase 1] Motors: OK" << endl;
    } else {
        cout << "[Phase 1] Motors: FAILED" << endl;
    }

    // 3. 카메라 초기화 (가장 민감한 자원)
    VideoCapture cap;
#ifdef PLATFORM_RASPI
    // [System] Verified working pipeline from DXL project
    string pipeline = "libcamerasrc ! video/x-raw, format=YUY2, width=320, height=240 ! videoconvert ! appsink drop=true max-buffers=1";
    cout << "[Phase 1] Attempting Camera (verified pipeline)..." << endl;
    cap.open(pipeline, CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        cout << "[Phase 1] GStreamer failed. Trying explicit V4L2 (index 0)..." << endl;
        cap.open(0, CAP_V4L2); 
    }
#else
    string video_path = "test_video.mp4";
    cap.open(video_path);
    if (!cap.isOpened()) {
        video_path = "/home/pinky/dev_ws/Lane_CPP_raspi_WHL_v1/tools/test_video.mp4";
        cap.open(video_path);
    }
#endif

    if (cap.isOpened()) {
        cout << "[Phase 1] Camera: OK" << endl;
    } else {
        cerr << "[Phase 1] Camera: FATAL ERROR - Exiting." << endl;
        return -1;
    }

    cout << "\n=========================================" << endl;
    cout << " [System] PHASE 2: AI & Network Startup" << endl;
    cout << "=========================================" << endl;

    // 4. YOLO 객체 탐지 초기화
    cout << "[Phase 2] Loading YOLOX Model..." << endl;
    YOLOXDetector yolo("training_data/yolox_s.onnx"); 
    cout << "[Phase 2] YOLO: OK" << endl;

    // 4.1 ArUco 로컬라이저 초기화
    cout << "[Phase 2] Initializing ArUco Marker Localizer..." << endl;
    ArUcoLocalizer arucoLocalizer;
    cout << "[Phase 2] ArUco: OK" << endl;

    // 5. DDS 통신 초기화
    CameraPublisher cameraPub;
    if (cameraPub.init()) {
        cout << "[Phase 2] Camera Streaming: OK" << endl;
    } else {
        cout << "[Phase 2] Camera Streaming: FAILED" << endl;
    }

    SensorDataSubscriber controlSub;
    if (controlSub.init()) {
        cout << "[Phase 2] Control Subscriber: OK" << endl;
    } else {
        cerr << "[Phase 2] Control Subscriber: FATAL ERROR - Exiting." << endl;
        return -1;
    }

    SensorDataPublisher fusedControlPub(100);
    if (fusedControlPub.init()) {
        cout << "[Phase 2] Fused Control Publisher: OK" << endl;
    } else {
        cout << "[Phase 2] Fused Control Publisher: FAILED" << endl;
    }
    
    // 5.5. IMU 초기화 (BNO055)
    std::unique_ptr<IMU> imuPtr;
    try {
        imuPtr = std::make_unique<IMU>(0, "ndof");
        cout << "[Phase 2] IMU (BNO055): OK" << endl;
    } catch (const std::exception& e) {
        cerr << "[Phase 2] IMU Error: " << e.what() << " (Continuing without IMU)" << endl;
    }

    // 5.6. IR/ADC 센서 초기화 (I2C-1, 0x08)
    std::unique_ptr<ADCSensor> adcPtr;
    try {
        adcPtr = std::make_unique<ADCSensor>(1, 0x08);
        cout << "[Phase 2] ADC Sensors (IR/US/Batt): OK" << endl;
    } catch (const std::exception& e) {
        cerr << "[Phase 2] ADC Error: " << e.what() << " (Continuing without ADC Sensors)" << endl;
    }

    // 6. 데이터 로거 초기화
    DataLogger dataLogger("training_data");
    bool is_logging = false;

    cout << "\n[System] All Systems Ready. Entering Control Loop." << endl;
    print_help(); // [수정] 함수 호출로 변경

    Mat frame, frame_small;
    Mat binary, Minv_small, M_small, warped, result;
    vector<double> left_fit, right_fit;
    
    TickMeter tm;
    float fps = 0.0;
    uint32_t frameCount = 0;

    int manual_offset_val = laneSys.manual_offset;
    bool g_auto_mode = true; // [수정] 전역 스코프 변수명 변경하여 쉐도잉 방지
    bool show_lidar_pip = true; 
    bool show_bev_pip = true; 
    bool show_camera_pip = true;
    
    bool last_lidar_dist_up_btn = false;
    bool last_lidar_dist_down_btn = false;
    bool last_lidar_btn = false;
    bool last_camera_btn = false;
    bool last_bev_btn = false;
    bool last_mode_btn = false;
    bool last_speed_up_btn = false;
    bool last_speed_down_btn = false;
    bool last_stop_btn = false;
    
    // [신규] 토글 상태 및 에지 검출용 변수
    bool g_save_detections = false; // [수정] 초기값 OFF
    bool last_l1_r1_combined = false;
    bool last_l2_r2_combined = false;
    string toggle_msg = "";
    auto toggle_msg_time = chrono::steady_clock::now();
    bool last_log_btn_comb = false;
    auto last_lane_seen_time = chrono::steady_clock::now();

    string last_error = "";
    auto last_error_time = chrono::steady_clock::now();

    lidar.set_stop_distance(laneSys.lidar_stop_dist);

    namedWindow("LaneResult", WINDOW_NORMAL); moveWindow("LaneResult", laneSys.win_x_res, laneSys.win_y_res); resizeWindow("LaneResult", laneSys.win_w_res, laneSys.win_h_res);
    namedWindow("Binary", WINDOW_NORMAL); moveWindow("Binary", laneSys.win_x_bin, laneSys.win_y_bin); resizeWindow("Binary", laneSys.win_w_bin, laneSys.win_h_bin);
    namedWindow("Warped", WINDOW_NORMAL); moveWindow("Warped", laneSys.win_x_warp, laneSys.win_y_warp); resizeWindow("Warped", laneSys.win_w_warp, laneSys.win_h_warp);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGABRT, signal_handler);

    while (g_running) {
        auto now = chrono::steady_clock::now();
        tm.start();
        
        cap >> frame;
        if (frame.empty()) {
            static int empty_count = 0;
            empty_count++;
            if (empty_count > 100) {
                cap.release();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                cap.open(0, CAP_V4L2);
                empty_count = 0;
            }
            continue;
        }
        
        rotate(frame, frame, ROTATE_180);
        Mat clean_frame = frame.clone();
        
        if (frameCount % 10 == 0) {
            yolo.push_frame(frame);
        }
        
        yolo.update_snapshot();
        // YOLO 및 ArUco 그리기도 오프셋 적용이 필요할 수 있으나 일단 원본 프레임에 그림
        yolo.draw(frame, laneSys.get_roi_points(frame.cols, frame.rows));
        arucoLocalizer.detectAndDraw(frame);
        
        frameCount++;

        // [신규] 가상 공간(화면 밖) 시각화를 위한 캔버스 생성
        int pad_w = frame.cols * 0.4; // 좌우 20%씩 확장
        int pad_h = frame.rows * 0.2; // 상하 10%씩 확장
        Mat canvas = Mat::zeros(frame.rows + pad_h, frame.cols + pad_w, frame.type());
        Point2f pad_off(pad_w / 2.0f, pad_h / 2.0f);
        frame.copyTo(canvas(Rect(pad_off.x, pad_off.y, frame.cols, frame.rows)));

        if (imuPtr) {
            try {
                auto imuData = imuPtr->readData();
                cameraPub.publishIMU(imuData.euler[0], imuData.euler[1], imuData.euler[2], imuData.acc[2]);
            } catch (...) {}
        }

        if (adcPtr && frameCount % 5 == 0) {
            try {
                auto adcData = adcPtr->readData();
                cameraPub.publishADC(adcData);
            } catch (...) {}
        }

        auto ctrl = controlSub.getControlState();
        static bool first_ctrl_received = false;
        if (!first_ctrl_received && ctrl.samples > 0) {
            first_ctrl_received = true;
        }

        // [수정] 조이스틱 입력이 있을 때, 상태가 '변경'될 때만 모드 동기화 (키보드 입력 덮어쓰기 방지)
        static bool last_joy_manual = false;
        if (ctrl.samples > 0 && ctrl.manual_mode != last_joy_manual) {
            g_auto_mode = !ctrl.manual_mode;
            last_joy_manual = ctrl.manual_mode;
            cout << "\n[System] Mode Synced from Controller: " << (g_auto_mode ? "AUTO" : "MANUAL") << endl;
        }
        
        float joy_steer = ctrl.steering;
        float joy_gas = ctrl.accelerator;
        float joy_brake = ctrl.brake;
        int blinker = ctrl.blinker;

        static float last_roi_h = 0;
        static float last_roi_g = 0;

        if (ctrl.roi_h_axis < -30000 && last_roi_h >= -30000) { laneSys.roi_h_top -= 0.01; }
        if (ctrl.roi_h_axis > 30000 && last_roi_h <= 30000) { laneSys.roi_h_top += 0.01; }
        if (ctrl.roi_g_axis < -30000 && last_roi_g >= -30000) { laneSys.roi_top_gap -= 0.01; }
        if (ctrl.roi_g_axis > 30000 && last_roi_g <= 30000) { laneSys.roi_top_gap += 0.01; }
        last_roi_h = ctrl.roi_h_axis;
        last_roi_g = ctrl.roi_g_axis;

        if (ctrl.lidar_btn && !last_lidar_btn) { show_lidar_pip = !show_lidar_pip; }
        if (ctrl.camera_btn && !last_camera_btn) { show_camera_pip = !show_camera_pip; }
        if (ctrl.bev_btn && !last_bev_btn) { show_bev_pip = !show_bev_pip; }
        
        if (ctrl.speed_up_btn && !last_speed_up_btn) {
            if (g_auto_mode) { laneSys.auto_speed += 5; }
            else { laneSys.manual_speed += 5; }
        }
        if (ctrl.speed_down_btn && !last_speed_down_btn) {
            if (g_auto_mode) { laneSys.auto_speed -= 5; }
            else { laneSys.manual_speed -= 5; }
        }
        
        if (ctrl.lidar_dist_up_btn && !last_lidar_dist_up_btn) {
            laneSys.lidar_stop_dist += 0.1f;
            lidar.set_stop_distance(laneSys.lidar_stop_dist);
        }
        if (ctrl.lidar_dist_down_btn && !last_lidar_dist_down_btn) {
            laneSys.lidar_stop_dist = max(0.1f, laneSys.lidar_stop_dist - 0.1f);
            lidar.set_stop_distance(laneSys.lidar_stop_dist);
        }

        if (ctrl.stop_btn && !last_stop_btn) {
            laneSys.manual_speed = 0; 
            laneSys.auto_speed = 0;
            manual_offset_val = 0;
            laneSys.manual_offset = 0;
            g_auto_mode = false;
            motor.stop(); 
        }

        last_lidar_dist_up_btn = ctrl.lidar_dist_up_btn;
        last_lidar_dist_down_btn = ctrl.lidar_dist_down_btn;
        last_lidar_btn = ctrl.lidar_btn;
        last_camera_btn = ctrl.camera_btn;
        last_bev_btn = ctrl.bev_btn;
        last_mode_btn = ctrl.mode_btn;
        last_speed_up_btn = ctrl.speed_up_btn;
        last_speed_down_btn = ctrl.speed_down_btn;
        last_stop_btn = ctrl.stop_btn;

        // [신규] 조이스틱 버튼 조합 (Start + Select) -> 센서 로그 토글
        bool current_log_btn_comb = (ctrl.lidar_dist_up_btn && ctrl.lidar_dist_down_btn);
        if (current_log_btn_comb && !last_log_btn_comb) {
            g_enable_sensor_log = !g_enable_sensor_log;
            if (!g_enable_sensor_log.load()) {
                cout << "\r                                                                                \r" << flush;
            }
            toggle_msg = "SENSOR LOG: " + string(g_enable_sensor_log ? "ON" : "OFF");
            toggle_msg_time = chrono::steady_clock::now();
            cout << "\n[Joy] " << toggle_msg << endl;
        }
        last_log_btn_comb = current_log_btn_comb;

        // [수정] L1 + R1 (Blinker L + Blinker R) -> YOLOX 저장 토글
        bool current_l1_r1 = (ctrl.l1_btn && ctrl.r1_btn);
        if (current_l1_r1 && !last_l1_r1_combined) {
            g_save_detections = !g_save_detections;
            toggle_msg = "YOLOX SAVE: " + string(g_save_detections ? "ON" : "OFF");
            toggle_msg_time = chrono::steady_clock::now();
            cout << "\n[System] " << toggle_msg << endl;
        }
        last_l1_r1_combined = current_l1_r1;

        // [신규] L2 + R2 (Brake + Gas) -> DataLogger 토글
        bool current_l2_r2 = (ctrl.brake > 0.5 && ctrl.accelerator > 0.5);
        if (current_l2_r2 && !last_l2_r2_combined) {
            if (!is_logging) {
                dataLogger.start();
                is_logging = true;
                cout << "\n[System] DataLogger: STARTED" << endl;
            } else {
                dataLogger.stop();
                is_logging = false;
                cout << "\n[System] DataLogger: STOPPED" << endl;
            }
        }
        last_l2_r2_combined = current_l2_r2;

        // [삭제] 기존 매뉴얼 모드 시 자동 로깅 로직 제거 (버튼 조합으로 변경됨)
        
        frame_small = frame.clone();
        binary = laneSys.thresholding(frame_small);
        warped = laneSys.bird_eye_view(binary, Minv_small, M_small);
        laneSys.sliding_window(warped, left_fit, right_fit);

        // [신규] 유동적 ROI 알고리즘 적용
        if (!left_fit.empty() && !right_fit.empty()) {
            laneSys.update_adaptive_roi(left_fit, right_fit, warped.cols);
        }

        float angle = 0;
        // 차선 그리기 (캔버스 오프셋 전달)
        laneSys.draw_lane_and_info(canvas, warped, left_fit, right_fit, Minv_small, angle, pad_off);
        laneSys.draw_roi_on_img(canvas, pad_off);
        
        // 조향 휠 위치 하단 조정
        laneSys.draw_steering_wheel(canvas, angle + manual_offset_val, Point(canvas.cols/2, canvas.rows - 70));

        result = canvas; // 결과물을 캔버스로 교체

        float obstacle_dist = lidar.get_front_distance();
        bool obstacle_stop = lidar.is_obstacle_detected();
        bool lane_ok = (!left_fit.empty() && !right_fit.empty());
        if (lane_ok) {
            last_lane_seen_time = chrono::steady_clock::now();
        }
        auto lane_loss_ms = chrono::duration_cast<chrono::milliseconds>(
            chrono::steady_clock::now() - last_lane_seen_time).count();
        bool lane_loss_stop = g_auto_mode && !lane_ok && lane_loss_ms >= laneSys.lane_loss_stop_ms;

        // [수정] 조향 오프셋 동기화 (키보드 조작값 반영)
        manual_offset_val = laneSys.manual_offset;
        float applied_angle = (g_auto_mode ? angle : (joy_steer * 45.0f)) + manual_offset_val;

        // [수정] 수동 모드 시 조이스틱 입력이 있으면 사용, 없으면 키보드 설정값(manual_speed) 사용
        float base_speed = 0.0f;
        if (g_auto_mode) {
            base_speed = (float)laneSys.auto_speed;
        } else {
            if (abs(joy_gas) > 0.05f || abs(joy_brake) > 0.05f) {
                base_speed = (joy_gas * 100.0f - joy_brake * 100.0f);
            } else {
                base_speed = (float)laneSys.manual_speed;
            }
        }
        
        bool is_reversing = false;
        if (!g_auto_mode && ctrl.l1_btn) {
            base_speed = -joy_gas * 100.0f; 
            is_reversing = true;
            if (abs(joy_gas) > 0.1) {
                putText(result, "REVERSE GEAR (L1)", Point(result.cols/2 - 130, 150), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
            }
        }

        if (g_auto_mode && laneSys.slow_zone) {
            base_speed *= 0.5f;
            putText(result, "SLOW ZONE", Point(result.cols/2 - 70, 70), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 165, 255), 2);
        }
        
        float applied_speed = base_speed;
        float avoidance_offset = 0.0f;
        float lidar_avoid_offset = 0.0f;

        // 1. AI (Person/Animal) Avoidance
        bool obstacle_ai = yolo.is_person_or_animal_detected();
        if (g_auto_mode && obstacle_ai) {
            if (blinker == 0 || blinker == 3) {
                applied_speed = 0;
                // [수정] 메인 알림 메시지 위치 상향 (조향 휠 가림 방지)
                putText(result, "PERSON/ANIMAL DETECTED!", Point(result.cols/2 - 130, 85), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            } else if (blinker == 1) {
                avoidance_offset = -20.0f;
                putText(result, "AVOIDING (LEFT)", Point(result.cols/2 - 80, 100), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            } else if (blinker == 2) {
                avoidance_offset = 20.0f;
                putText(result, "AVOIDING (RIGHT)", Point(result.cols/2 - 80, 100), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            }
        }

        // 2. Lidar Wall/Obstacle Avoidance
        float left_dist = lidar.get_left_distance();
        float right_dist = lidar.get_right_distance();
        // 설정된 정지 거리의 2배 거리부터 회피 시작
        float lidar_avoid_dist = laneSys.lidar_stop_dist * 2.5f;

        // [수정] 라이더 감지 각도 동기화 (ROI gap에 비례)
        float current_det_angle = 30.0f * (laneSys.roi_top_gap / 0.40f);
        lidar.set_detection_angle(current_det_angle);

        if (g_auto_mode && !obstacle_ai) { // AI 회피가 우선순위가 높으므로 AI 미감지 시에만 Lidar 회피 작동
            if (left_dist < lidar_avoid_dist && left_dist < right_dist) {
                lidar_avoid_offset = 30.0f * (1.0f - (left_dist / lidar_avoid_dist));
                // [수정] 폰트 크기 및 위치 조정 (OSD 겹침 방지)
                putText(result, format("LIDAR AVOIDING (RIGHT) %.1fm", left_dist), Point(result.cols/2 - 90, 110), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1);
            } else if (right_dist < lidar_avoid_dist && right_dist < left_dist) {
                lidar_avoid_offset = -30.0f * (1.0f - (right_dist / lidar_avoid_dist));
                // [수정] 폰트 크기 및 위치 조정
                putText(result, format("LIDAR AVOIDING (LEFT) %.1fm", right_dist), Point(result.cols/2 - 90, 110), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1);
            }
        }

        // [신규] 이벤트 이미지 저장 (객체 감지 시) - 2초 쿨다운 & 저장 옵션 체크
        static auto last_event_save = chrono::steady_clock::now();
        if (g_save_detections && obstacle_ai && chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - last_event_save).count() >= 2) {
            last_event_save = chrono::steady_clock::now();
            
            // 1. Person Detection
            if (yolo.is_person_detected()) {
                string filename = format("training_data/person/person_%ld.jpg", time(0));
                cv::imwrite(filename, clean_frame);
                save_detection_json(filename, yolo.get_json_results());
                cout << "\n[System] Person image + JSON saved (Clean): " << filename << endl;
            }
            
            // 2. Animal Detection
            if (yolo.is_animal_detected()) {
                string filename = format("training_data/animal/animal_%ld.jpg", time(0));
                cv::imwrite(filename, clean_frame);
                save_detection_json(filename, yolo.get_json_results());
                cout << "\n[System] Animal image + JSON saved (Clean): " << filename << endl;
            }

            // 3. Car Detection
            if (yolo.is_car_detected()) {
                string filename = format("training_data/car/car_%ld.jpg", time(0));
                cv::imwrite(filename, clean_frame);
                save_detection_json(filename, yolo.get_json_results());
                cout << "\n[System] Car image + JSON saved (Clean): " << filename << endl;
            }
            
            // 4. Fallback (General Event)
            if (!yolo.is_person_detected() && !yolo.is_animal_detected() && !yolo.is_car_detected()) {
                string names = yolo.get_detection_names();
                string filename = format("training_data/event/event_%ld_%s.jpg", time(0), names.c_str());
                cv::imwrite(filename, clean_frame);
                save_detection_json(filename, yolo.get_json_results());
                cout << "\n[System] Event image + JSON saved (Clean): " << filename << endl;
            }
        }

        // 최종 조향각 계산 (모든 회피 오프셋 합산)
        float final_applied_angle = applied_angle + avoidance_offset + lidar_avoid_offset;
        if (lane_loss_stop) {
            applied_speed = 0.0f;
            final_applied_angle = 0.0f;
            putText(result, "LANE LOST - AUTO STOP", Point(result.cols/2 - 110, 130), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 255), 2);
        }

        float L_speed = applied_speed + (final_applied_angle * (g_auto_mode ? 0.5f : 1.0f));
        float R_speed = applied_speed - (final_applied_angle * (g_auto_mode ? 0.5f : 1.0f));
        
        // 데이터 로깅 (수동 운전 시 차선 중앙 유지 데이터 수집)
        if (is_logging && joy_gas > 0.1) {
            dataLogger.log(frame_small, joy_steer, joy_gas, joy_brake, blinker);
        }

        static bool last_obstacle_stop = false;
        if (obstacle_stop && !last_obstacle_stop) {
            if (g_enable_sensor_log.load()) {
                std::cout << "\n[Radar] OBSTACLE DETECTED! Stopping motors. Dist: " << fixed << setprecision(2) << obstacle_dist << "m" << std::endl;
            }
        } else if (!obstacle_stop && last_obstacle_stop) {
            if (g_enable_sensor_log.load()) {
                std::cout << "\n[Radar] Obstacle cleared. Resuming." << std::endl;
            }
        }
        last_obstacle_stop = obstacle_stop;

        // [수정] 라이더 근접 범위(Stop)를 벗어났다면 조금씩 전진하며 움직임
        if ((!is_reversing && obstacle_stop) || joy_brake > 0.5 || lane_loss_stop) {
            motor.stop();
            if (!is_reversing && obstacle_stop) putText(result, "OBSTACLE DETECTED!", Point(result.cols/2 - 70, 100), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1);
        } else if (g_auto_mode || joy_gas > 0.1) {
            // 장애물이 가깝지만 Stop 거리보다는 먼 경우 서행 전진 (Creep)
            float speed_factor = 1.0f;
            if (g_auto_mode && obstacle_dist < lidar_avoid_dist) {
                speed_factor = 0.5f; // 감지 범위 내에서는 조금씩 천천히
            }
            motor.move(L_speed * speed_factor, R_speed * speed_factor);
        } else {
            motor.stop();
        }
        if (frameCount % 30 == 0 && g_enable_sensor_log.load()) {
            std::cout << "[Debug] Motor control done." << std::endl;
        }

        // 깜빡이 표시
        if (blinker == 1 || (frameCount % 10 < 5 && blinker == 3)) { // 좌측 또는 비상등
            arrowedLine(result, Point(100, 100), Point(20, 100), Scalar(0, 255, 255), 5);
        }
        if (blinker == 2 || (frameCount % 10 < 5 && blinker == 3)) { // 우측 또는 비상등
            arrowedLine(result, Point(result.cols - 100, 100), Point(result.cols - 20, 100), Scalar(0, 255, 255), 5);
        }

        // ── HUD Overlay (Layout 0.18) ──
        // Top Left: System Info
        putText(result, "CAM: 5MP . 60FPS", Point(15, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 200, 255), 1);
        
        string lane_status = lane_ok ? "LANE: DETECTED" : "LANE: SEARCHING...";
        Scalar lane_color = lane_ok ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
        putText(result, lane_status, Point(15, 50), FONT_HERSHEY_SIMPLEX, 0.4, lane_color, 1);
        
        string mode_str = g_auto_mode ? "MODE: AUTO" : "MODE: MANUAL";
        Scalar mode_color = g_auto_mode ? Scalar(0, 255, 0) : Scalar(0, 255, 255);
        putText(result, mode_str, Point(15, 70), FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2);

        // Top Right: Driving Info (Moved further right to avoid overlap)
        int col_r = result.cols - 110; 
        string dist_str = format("DIST: %.2fm", obstacle_dist);
        string speed_str = format("SPD: %.1fm/s", (float)laneSys.auto_speed * 0.003f);
        putText(result, "WP: 2 AHEAD", Point(col_r, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 200, 255), 1);
        putText(result, dist_str, Point(col_r, 50), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 200, 255), 1);
        putText(result, speed_str, Point(col_r, 70), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 200, 255), 1);

        // Bottom Center: Version & REC Status
        string ver_str = "REC. ARASEO-DALIMI v0.17";
        if (!g_enable_sensor_log.load()) ver_str += " (LOG OFF)";
        putText(result, ver_str, Point(result.cols/2 - 70, result.rows - 10), FONT_HERSHEY_SIMPLEX, 0.22, Scalar(0, 200, 255), 1);

        // [신규] 토글 메시지 화면 출력 (2초간)
        if (!toggle_msg.empty() && chrono::duration_cast<chrono::seconds>(now - toggle_msg_time).count() < 2) {
            putText(result, toggle_msg, Point(result.cols/2 - 100, result.rows/2), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 3);
        }

        // PIP (BEV & Raw Camera)
        Mat bev_visual;
        warpPerspective(frame_small, bev_visual, M_small, frame_small.size());
        
        // 1. BEV PIP (Right-Top)
        if (show_bev_pip) {
            Mat small_map;
            resize(bev_visual, small_map, Size(), 0.3, 0.3); 
            Rect roi_rect(result.cols - small_map.cols - 20, 20, small_map.cols, small_map.rows);
            small_map.copyTo(result(roi_rect));
            rectangle(result, roi_rect, Scalar(0, 255, 0), 2);
            putText(result, "BEV", Point(roi_rect.x, roi_rect.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
        }

        // 2. Raw Camera PIP (Left-Top)
        if (show_camera_pip) {
            Mat raw_pip;
            resize(frame_small, raw_pip, Size(), 0.3, 0.3); 
            Rect raw_roi(20, result.rows - raw_pip.rows - 20, raw_pip.cols, raw_pip.rows); // 좌측 하단 배치
            raw_pip.copyTo(result(raw_roi));
            rectangle(result, raw_roi, Scalar(255, 255, 255), 2);
            putText(result, "Camera", Point(raw_roi.x, raw_roi.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
        }

        // 3. Lidar Radar Map PIP (Right-Bottom)
        if (show_lidar_pip) {
            Mat lidar_map = laneSys.draw_lidar_map(lidar, 200);
            Rect lidar_roi(result.cols - lidar_map.cols - 20, result.rows - lidar_map.rows - 20, lidar_map.cols, lidar_map.rows);
            lidar_map.copyTo(result(lidar_roi));
            rectangle(result, lidar_roi, Scalar(0, 255, 255), 2);
            putText(result, "Radar", Point(lidar_roi.x, lidar_roi.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }

        // [DDS] 카메라 스트리밍 (Base64 Encode & Publish) - CPU 부하 조절을 위해 3프레임당 1회
        if (frameCount % 3 == 0) {
            cv::Mat streamImg;
            // [수정] 캘리브레이션을 위해 320x240 원본 화질 전송 (품질 40으로 상향)
            cv::resize(clean_frame, streamImg, cv::Size(320, 240)); 
            
            std::vector<uchar> buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 40}; // 품질 상향
            if (cv::imencode(".jpg", streamImg, buf, params)) {
                std::string base64_str = base64_encode(buf.data(), buf.size());
                
                if (frameCount % 30 == 0 && g_enable_sensor_log.load()) {
                    std::cout << "[DDS] Publishing Frame (High Quality). Size: " << base64_str.length() << " chars." << std::endl;
                }
                
                cameraPub.publish(base64_str);
            } else {
                if (g_enable_sensor_log.load()) {
                    std::cerr << "[DDS] Error: JPEG encoding failed!" << std::endl;
                }
            }
        }
        
        // [DDS] 배터리 및 차량 상태 전송 (10프레임마다)
        if (frameCount % 10 == 0) {
            cameraPub.publishStatus(g_auto_mode, (int)applied_speed, obstacle_dist);
            SensorData fused;
            fused.sensor_name("FUSED_VEHICLE_CONTROL");
            fused.data({
                final_applied_angle,
                applied_speed,
                L_speed,
                R_speed,
                obstacle_dist,
                lane_ok ? 1.0f : 0.0f,
                obstacle_stop ? 1.0f : 0.0f,
                lane_loss_stop ? 1.0f : 0.0f
            });
            fused.status(g_auto_mode ? "AUTO" : "MANUAL");
            fusedControlPub.publish(fused);
        }

        if (g_enable_sensor_log.load()) {
            cout << "\r[Status] FPS: " << fixed << setprecision(1) << fps 
                 << " | Speed: " << setw(3) << (int)applied_speed
                 << " | Dist: " << fixed << setprecision(2) << obstacle_dist << "m"
                 << " | Thr: " << laneSys.lidar_stop_dist << "m"
                 << " | MOT -> L: " << setw(4) << (int)L_speed << " R: " << setw(4) << (int)R_speed 
                 << (obstacle_stop ? " [BLOCKED]" : " [CLEAR]") << "    " << flush;
        }

        // [복구] LCD 표시를 위한 상태 저장 (0.5초 주기로 파일 쓰기)
        static auto last_lcd_update = chrono::steady_clock::now();
        now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - last_lcd_update).count() > 500) {
            last_lcd_update = now;
            ofstream status_file("/tmp/car_status.json.tmp");
            if (status_file.is_open()) {
                status_file << "{"
                            << "\"fps\":" << fixed << setprecision(1) << fps << ","
                            << "\"dist\":" << setprecision(2) << obstacle_dist << ","
                            << "\"speed\":" << (int)applied_speed << ","
                            << "\"stop_status\":" << (obstacle_stop ? "true" : "false") << ","
                            << "\"auto_mode\":" << (g_auto_mode ? "true" : "false") << ","
                            << "\"error\":\"" << last_error << "\","
                            << "\"lidar_points\":[";
                
                auto nodes = lidar.get_scan_data();
                bool first = true;
                float last_angle = -10.0f;
                for (auto& node : nodes) {
                    if (node.dist_mm_q2 == 0) continue;
                    float angle = node.angle_z_q14 * 90.f / (1 << 14);
                    float dist = node.dist_mm_q2 / 4.0f / 1000.0f;
                    if (abs(angle - last_angle) >= 5.0f) {
                        if (!first) status_file << ",";
                        status_file << "[" << fixed << setprecision(1) << angle << "," << setprecision(2) << dist << "]";
                        first = false;
                        last_angle = angle;
                    }
                }
                status_file << "]}";
                status_file.close();
                std::rename("/tmp/car_status.json.tmp", "/tmp/car_status.json");
            }
        }

        imshow("LaneResult", result); 
        imshow("Binary", binary);
        imshow("Warped", warped);
        
        if (frameCount % 30 == 0) {
            laneSys.sync_window_positions(frameCount);
        }

        char key = (char)cv::waitKey(1);
        if (key != -1) {
            // [ROI Control] 상단
            if (key == 'w') { laneSys.roi_h_top -= 0.01; cout << "\r[ROI] Top Height: " << fixed << setprecision(2) << laneSys.roi_h_top << "    " << flush; }
            if (key == 's') { laneSys.roi_h_top += 0.01; cout << "\r[ROI] Top Height: " << fixed << setprecision(2) << laneSys.roi_h_top << "    " << flush; }
            if (key == 'a') { laneSys.roi_top_shift -= 0.01; cout << "\r[ROI] Top Shift : " << fixed << setprecision(2) << laneSys.roi_top_shift << "    " << flush; }
            if (key == 'd') { laneSys.roi_top_shift += 0.01; cout << "\r[ROI] Top Shift : " << fixed << setprecision(2) << laneSys.roi_top_shift << "    " << flush; }
            if (key == 'h') { laneSys.roi_top_gap = max(0.01f, laneSys.roi_top_gap - 0.01f); cout << "\r[ROI] Top Width : " << fixed << setprecision(2) << laneSys.roi_top_gap << "    " << flush; }
            if (key == 'j') { 
                // 상단 넓힐 때 하단에 막힘
                laneSys.roi_top_gap = min(laneSys.roi_bot_gap, laneSys.roi_top_gap + 0.01f); 
                cout << "\r[ROI] Top Width : " << fixed << setprecision(2) << laneSys.roi_top_gap << "    " << flush; 
            }

            // [ROI Control] 하단
            if (key == 'z') { laneSys.roi_bot_shift -= 0.01; cout << "\r[ROI] Bot Shift : " << fixed << setprecision(2) << laneSys.roi_bot_shift << "    " << flush; }
            if (key == 'x') { laneSys.roi_bot_shift += 0.01; cout << "\r[ROI] Bot Shift : " << fixed << setprecision(2) << laneSys.roi_bot_shift << "    " << flush; }
            if (key == 'n') { 
                // 하단 좁힐 때 상단에 막힘
                laneSys.roi_bot_gap = max(laneSys.roi_top_gap, laneSys.roi_bot_gap - 0.02f); 
                cout << "\r[ROI] Bot Width : " << fixed << setprecision(2) << laneSys.roi_bot_gap << "    " << flush; 
            }
            if (key == 'm') { laneSys.roi_bot_gap += 0.02; cout << "\r[ROI] Bot Width : " << fixed << setprecision(2) << laneSys.roi_bot_gap << "    " << flush; }

            // [Driving Control]
            if (key == 'i') { 
                if (g_auto_mode) laneSys.auto_speed += 5; 
                else laneSys.manual_speed += 5;
                cout << "\r[Speed] Set: " << (g_auto_mode ? laneSys.auto_speed : laneSys.manual_speed) << "    " << flush;
            }
            if (key == 'k') { 
                if (g_auto_mode) laneSys.auto_speed -= 5;
                else laneSys.manual_speed -= 5;
                cout << "\r[Speed] Set: " << (g_auto_mode ? laneSys.auto_speed : laneSys.manual_speed) << "    " << flush;
            }
            if (key == 'u') { laneSys.manual_offset -= 2; cout << "\r[Steer] Offset: " << laneSys.manual_offset << "    " << flush; }
            if (key == 'o') { laneSys.manual_offset += 2; cout << "\r[Steer] Offset: " << laneSys.manual_offset << "    " << flush; }
            if (key == 'v') { g_auto_mode = !g_auto_mode; cout << "\n[System] Toggle AUTO: " << (g_auto_mode ? "ON" : "OFF") << endl; }
            if (key == 'r') { laneSys.reset_config(); cout << "\n[System] Parameters RESET." << endl; }
            
            // [System Control]
            if ((key & 0xFF) == 'p') { show_lidar_pip = !show_lidar_pip; }
            if ((key & 0xFF) == 'b') { show_bev_pip = !show_bev_pip; }
            if ((key & 0xFF) == 'c') { show_camera_pip = !show_camera_pip; }
            if ((key & 0xFF) == '?' || (key & 0xFF) == 'h') { print_help(); }
            
            if ((key & 0xFF) == 't') { laneSys.white_thr += 5; cout << "\r[Thr] White: " << laneSys.white_thr << "    " << flush; }
            if ((key & 0xFF) == '[') { laneSys.white_thr -= 5; cout << "\r[Thr] White: " << laneSys.white_thr << "    " << flush; }
            
            if ((key & 0xFF) == 'g') { 
                laneSys.use_adaptive_roi = !laneSys.use_adaptive_roi; 
                cout << "\n[System] Adaptive ROI: " << (laneSys.use_adaptive_roi ? "ON" : "OFF") << endl; 
            }
            if ((key & 0xFF) == 'e') { 
                g_enable_sensor_log = !g_enable_sensor_log; 
                if (!g_enable_sensor_log.load()) cout << "\r                                              \r" << flush;
                cout << "\n[System] Sensor Log: " << (g_enable_sensor_log ? "ON" : "OFF") << endl; 
            }
            if ((key & 0xFF) == '1') { laneSys.lidar_stop_dist -= 0.05; cout << "\r[Lidar] Stop Dist: " << laneSys.lidar_stop_dist << "m    " << flush; }
            if ((key & 0xFF) == '2') { laneSys.lidar_stop_dist += 0.05; cout << "\r[Lidar] Stop Dist: " << laneSys.lidar_stop_dist << "m    " << flush; }

            if ((key & 0xFF) == ' ') { laneSys.auto_speed = 0; laneSys.manual_speed = 0; motor.stop(); cout << "\n[System] !EMERGENCY STOP!" << endl; }
            if ((key & 0xFF) == 'q' || (key & 0xFF) == 27) break;
        }

        tm.stop();
        fps = tm.getFPS();
        tm.reset();
        frameCount++;
    }
    
    motor.stop();
    motor.close();
    lidar.close();
    laneSys.save_config();
    return 0;
}
