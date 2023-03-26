#ifndef CONFIG_H
#define CONFIG_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>


namespace semantic_slam
{

class ViewerConfig
{

};

class CameraConfig
{

};

class ORBExtractorConfig
{

};

class IMUConfig
{

};

class ConfigParser
{
public:
    bool ParseConfigFile(std::string &strConfigFile);

private:

    ViewerConfig mViewerConfig;
    CameraConfig mCameraConfig;
    ORBExtractorConfig mORBConfig;
    IMUConfig mIMUConfig;

};

}

#endif // CONFIG_H
