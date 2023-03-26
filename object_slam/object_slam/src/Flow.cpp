//
// Created by lacie on 28/01/2023.
//

#include "Flow.h"

namespace semantic_slam
{
    Flow::Flow()
    {
        // mBInaryThreshold=4.0;
    }

    // Without considering the camera movement, directly calculate the optical flow
    void Flow::ComputeMask(const cv::Mat& GrayImg, cv::Mat& mask,
                           float BInaryThreshold)
    {
        if(!GrayImg.empty())
        {
            if(BInaryThreshold<40.0) BInaryThreshold=40.0;
            mask = cv::Mat::ones(GrayImg.rows,GrayImg.cols, CV_8U);// Default 1, static object
            pyrDown(GrayImg, mImGrayCurrent, cv::Size(GrayImg.cols / 2, GrayImg.rows / 2));
            // Down-sampling to speed up
            if( mImGrayLast.data )
            {
                cv::Mat flow,flow2;
                calcOpticalFlowFarneback(mImGrayLast, mImGrayCurrent, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
                // Upsample back to original size
                pyrUp(flow, flow2, cv::Size(flow.cols * 2, flow.rows * 2));

                for(int y=0; y<flow2.rows; y++) // Every 5th row y+= 5
                {
                    for(int x=0; x<flow2.cols; x++)
                    {
                        const cv::Point2f xy = flow2.at<cv::Point2f>(y, x);
                        float tep2 = xy.x * xy.x + xy.y * xy.y ;
                        // float tep = sqrt(xy.x * xy.x + xy.y * xy.y);
                        // Adjusting this threshold can properly reduce the influence of camera movement,
                        // but it will reduce the sensitivity of small moving objects. .
                        if(tep2 < BInaryThreshold) // Eliminate optical flow values with too small optical flow >8*8
                            continue;
                        mask.at<unsigned char>(y,x) = 0;// If the optical flow value is too large, it is a moving object area
                    }
                }
                // Kernel size
                int dilation_size = 10;
                cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                       cv::Point( dilation_size, dilation_size ) );
                cv::erode(mask, mask, kernel);// Corrosion, choose a small
                cv::erode(mask, mask, kernel);// Corrosion, choose a small
                cv::dilate(mask, mask, kernel);// Inflate, choose big

            }
            // Update the grayscale image of the previous frame
            std::swap(mImGrayLast, mImGrayCurrent);
        }
    }

    // The CalcOpticalFlowFarneback() function uses Gunnar Farneback's algorithm,
    // Calculate the global dense optical flow algorithm (that is, the optical flow of all pixels on the image is calculated),
    // Since the optical flow of all points on the image needs to be calculated, the calculation is time-consuming and slow.
    // The parameter description is as follows:
    // _prev0: Input the previous frame image
    // _next0: Enter the next frame of image
    // _flow0: output optical flow
    // pyr_scale: the scale relationship between the upper and lower layers of the pyramid
    // levels: the number of pyramid layers
    // winsize: the average window size, the larger the denoise and the ability to detect fast-moving targets, but it will cause blurred motion areas
    // iterations: number of iterations
    // poly_n: the size of the pixel field, generally 5, 7, etc.
    // poly_sigma: Gaussian label difference, generally 1-1.5
    // flags: calculation method. Mainly including OPTFLOW_USE_INITIAL_FLOW and OPTFLOW_FARNEBACK_GAUSSIAN


    // Use the matching point pair to calculate the unit transformation,
    // perform the anti-homography transformation on mImGrayCurrent,
    // and then perform the optical flow calculation
    // Considering the running speed, I will not write it for the time being
    void Flow::ComputeMask(const cv::Mat& GrayImg, const cv::Mat& Homo,
                           cv::Mat& mask, float BInaryThreshold)
    {
        cv::Mat dest;
        //  myWarpPerspective(GrayImg, dest, Homo);  // Segmentation fault H matrix dimension???
        cv::warpPerspective( GrayImg, dest, Homo, GrayImg.size() );
        ComputeMask(dest, mask, BInaryThreshold);
    }


    // Perform homography matrix inverse transformation on the frame image to remove the influence of camera movement
    void Flow::myWarpPerspective(const cv::Mat& src, cv::Mat& dst, const cv::Mat& H)
    {
        if(!src.empty() && !H.empty())
        {
            // set all to zero first
            dst= cv::Mat::zeros(src.rows,src.cols,src.type());
            // i, j are points on the transformed image; u, v are points on the original image
            float u = 0,v = 0;
            for (int i = 0; i < dst.rows; i++)
            {
                for (int j = 0; j < dst.cols; j++)
                {
                    // There may be problems here
                    // H matrix dimension???
                    u = H.at<float>(0, 0)*i + H.at<float>(0, 1)*j + H.at<float>(0, 2);//cols
                    v = H.at<float>(1, 0)*i + H.at<float>(1, 1)*j + H.at<float>(1, 2);//rows

                    // Find the corresponding point on the original image that is within the range of the original image
                    if (int(u)>=0 && int(v)>=0 && int(u+0.5) < src.rows && int(v+0.5) < src.cols)
                    {
                        if (float(u-int(u))<0.5)
                        {
                            if (float( v - int(v)) < 0.5)
                                // 1 quadrant
                                dst.at<uchar>(i, j) = src.at<uchar>(int(u), int(v));
                            else
                                // 2 quadrant
                                dst.at<uchar>(i, j) = src.at<uchar>(int(u), int(v+0.5));
                        }
                        else
                        {
                            if (float(v - int(v)) < 0.5)
                                // 3 quadrant
                                dst.at<uchar>(i, j) = src.at<uchar>(int(u+0.5), int(v));
                            else
                                // 4 quadrant
                                dst.at<uchar>(i, j) = src.at<uchar>(int(u+0.5), int(v+0.5));
                        }

                    }
                }
            }
        }
    }
}
