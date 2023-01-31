//
// Created by lacie on 28/01/2023.
//

#include "Detector.h"

Detector::Detector(std::string modelPath)
{
    mDetector = new YoloDetection(modelPath);
    mvKeyframes.clear();
    colorImgs.clear();
    mRunThread = std::make_shared<thread>(bind(&Detector::Run, this));
    std::cout << "Yolo Detector Init \n";
}
Detector::~Detector()
{
    delete mDetector;
}

void Detector::insertKFColorImg(KeyFrame* kf, cv::Mat color)
{
    cout << "receive a keyframe, id = " << kf->mnId << endl;

    std::cout << color.size() << "\n";

    unique_lock<mutex> lck(colorImgMutex);
    colorImgs.push_back(color.clone());
    mvKeyframes.push_back(kf);

    colorImgUpdated.notify_one();
}

void Detector::Run(void)
{
    while(1)
    {
//        std::cout << "Detector Running ... \n";
        {
            unique_lock<mutex> lck_colorImgUpdated( colorImgMutex);
            colorImgUpdated.wait( lck_colorImgUpdated );
        }
        size_t N=0;
        {
            unique_lock<mutex> lck(colorImgMutex);
            N = colorImgs.size();
        }
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            std::vector<Object> vobject;
            mDetector->Detect(colorImgs[i], vobject);
            if(vobject.size()>0)
            {
                std::cout << "detect : " << vobject.size() << " obj" << std::endl;
                for(unsigned int j =0; j < vobject.size(); j++)
                {
                    unique_lock<mutex> lckObj(mvKeyframesMutex);
                    mvKeyframes[i]->mvObject.push_back(vobject[j]);
                }
            }
        }

        lastKeyframeSize = N;
    }

}
