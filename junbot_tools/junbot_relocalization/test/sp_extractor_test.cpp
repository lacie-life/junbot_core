//
// Created by lacie on 22/11/2022.
//

#include "SPDetector.h"
#include "Tools.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**  You need to modify the path below that corresponds to your dataset and weight path. **/
const std::string weight_dir = "./data/weights/superpoint.pt";
void test();


int main(const int argc, char* argv[])
{
    /** From the main argument, Retrieve waiting period to control Displaying.**/
    int ms = 100;

    std::cout << "Frame rate is " << ms << "ms.\n";

    /** Initialize VideoSteamer and SuperPoint Object **/
     VideoStreamer vs(argv[1]);
    // VideoStreamer vs(0);
    // VideoStreamer vs(kitti_dir);
    // vs.setImageSize(cv::Size(720, 240));


    /** Superpoint Detector **/
    SPDetector SPF(argv[2], torch::cuda::is_available());
    std::cout << "VC created, SPDetector Constructed.\n";

    cv::namedWindow("superpoint", cv::WINDOW_AUTOSIZE);
    long long idx=0;
    while(++idx){
        // Capture frame-by-frame
        // Image's size is [640 x 480]
        if(!vs.next_frame()) { std::cout << "main -- Video End\n"; break; }

        std::vector<cv::KeyPoint> Keypoints;
        cv::Mat Descriptors;

        auto start = std::chrono::system_clock::now();
        SPF.detect(vs.input, Keypoints, Descriptors);
        auto end = std::chrono::system_clock::now();
        std::chrono::milliseconds mill  = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        /* Logging */
        std::cout << idx << "th ProcessTime: " << mill.count() << "ms\n";
        std::cout << "Keypoint num: " << Keypoints.size() << std::endl;

        /* Visualization */
        float x_scale(vs.W_scale), y_scale(vs.H_scale);
        auto kpt_iter = Keypoints.begin();
        for(; kpt_iter != Keypoints.end(); kpt_iter++)
        {
            float X(kpt_iter->pt.x), Y(kpt_iter->pt.y);
            double conf(kpt_iter->response);
            cv::circle(vs.img, cv::Point(int(X*x_scale), int(Y*y_scale)), 1, cv::Scalar(0, 0, (255 * conf * 10)), 2);
        }

        /* Display the resulting frame */
        cv::imshow( "superpoint", vs.img );

        // Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(ms);
        if(c==27){ break; }
    }

    // Closes all the frames
    cv::destroyAllWindows();
}