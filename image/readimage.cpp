//
// Created by mlx on 3/15/21.
//

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;

int main() {
    std::string image_path = samples::findFile("starry_night.jpg");
    Mat img = imread(image_path, IMREAD_COLOR);
    if (img.empty()) {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    imshow("Display window", img);
    int k = waitKey(0); // Wait for a keystroke in the window
    if (k == 's') {
        imwrite("starry_night.png", img);
    }
    return 0;
}

void to_top(const cv::Mat &src, cv::Mat &top) {

//    相机内参
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                                        1681.52901998965, 0, 667.945786496189,
            0, 1510.91314479132, 375.142239238734,
            0, 0, 1);
//  注：世界坐标 z超下，所以将外参z反了一下
    cv::Mat Rot = (cv::Mat_<double>(4, 4) <<
                                          1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1);
// 外参
    cv::Mat extrinsic = (cv::Mat_<double>(4, 4) <<
                                                -0.003859404269123161, -0.9999420008273281, -0.01005484858798479, -0.11,
            0.01403883800665975, 0.009999753335606716, -0.9998514469463201, 1.51,
            0.999894002395309, -0.003999989333341806, 0.01399943067495647, 0.59,
            0, 0, 0, 1) * Rot;
// 添加平移
    cv::Mat T = (cv::Mat_<double>(3, 1) << -15, 0, 15);
// 目标顶视图相机虚拟内存那
    cv::Mat K_top = (cv::Mat_<double>(3, 3) <<
                                            320, 0, 320,
            0, 320, 320,
            0, 0, 1);
// 原始相机在地平面里的高度
    double d = 1.51;
    double invd = 1.0 / d;
// 原始相机坐标系中，地平面的法向量
    cv::Mat n = extrinsic(cv::Rect(0, 0, 3, 3)) * (cv::Mat_<double>(3, 1) << 0, 0, 1);
    std::cout << "n = " << std::endl << n << std::endl;
    cv::Mat nT;
    cv::transpose(n, nT);
    cv::Mat H = extrinsic(cv::Rect(0, 0, 3, 3)).inv() + T * nT * invd;
    cv::Mat Homography = K_top * H * K.inv();

    cv::warpPerspective(src, top2, Homography, cv::Size(640, 640));
}