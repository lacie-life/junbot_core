
#include "Converter.h"

namespace semantic_slam
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &T)
{
    return g2o::SE3Quat(T.unit_quaternion().cast<double>(), T.translation().cast<double>());
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3f& m)
{
    cv::Mat cvMat(3, 3, CV_32F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,4> &m)
{
    cv::Mat cvMat(3,4,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::MatrixXf &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Sophus::SE3f &T)
{
    return Converter::toCvMat(Converter::toSE3Quat(T));
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<float,3,1> Converter::toVector3f(const cv::Mat &cvVector)
{
    Eigen::Matrix<float,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

Eigen::Matrix<float, 3, 3> Converter::toMatrix3f(const cv::Mat& cvMat3)
{
    Eigen::Matrix<float, 3, 3> M;

    M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
        cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
        cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

    return M;
}

Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
    return M;
}

Eigen::Matrix<float,4,4> Converter::toMatrix4f(const cv::Mat &cvMat4)
{
    Eigen::Matrix<float,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
            cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
            cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
            cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

bool Converter::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

std::vector<float> Converter::toEuler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }

    std::vector<float> v_euler(3);
    v_euler[0] = x;
    v_euler[1] = y;
    v_euler[2] = z;

    return v_euler;
}

cv::Mat Converter::toRotationMatrix(const float x, const float y, const float z)
{
    // x, y, z
    // Rz * Ry * Rx
    cv::Mat axis_x = (cv::Mat_<float>(3, 3) << 1.0f, 0.0f, 0.0f,
        0.0f, cos(x), -sin(x),
        0.0f, sin(x), cos(x));
    cv::Mat axis_y = (cv::Mat_<float>(3, 3) << cos(y), 0.0f, sin(y),
        0.0f, 1.0f, 0.0f,
        -sin(y), 0.0f, cos(y));
    cv::Mat axis_z = (cv::Mat_<float>(3, 3) << cos(z), -sin(z), 0.0f,
        sin(z), cos(z), 0.0f,
        0.0f, 0.0f, 1.0f);
    return axis_z * axis_y * axis_x;
}

Sophus::SE3<float> Converter::toSophus(const cv::Mat &T) {
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(T.rowRange(0,3).colRange(0,3));
    Eigen::Quaternionf q(eigMat.cast<float>());

    Eigen::Matrix<float,3,1> t = toVector3d(T.rowRange(0,3).col(3)).cast<float>();

    return Sophus::SE3<float>(q,t);
}

Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
    return Sophus::Sim3f(Sophus::RxSO3d((float)S.scale(), S.rotation().matrix()).cast<float>() ,
                         S.translation().cast<float>());
}

// For 3D cuboid testing
float Converter::bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()+rect2.area()-overlap_area));
}

float Converter::bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect2.area()));
}

float Converter::bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()));
}

Eigen::Matrix4d Converter::cvMattoMatrix4d(const cv::Mat &cvMat4) {
    Eigen::Matrix4d M;

    M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
         cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
         cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
         cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);

    return M;
}


Eigen::Matrix4d Converter::Quation2Eigen(const double qx, const double qy, const double qz, const double qw, const double tx,
                                 const double ty, const double tz) {

    Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
    Eigen::AngleAxisd rotation_vector(quaternion);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(tx, ty, tz));
    Eigen::Matrix4d Pose_eigen = T.matrix();
    return Pose_eigen;
}

cv::Mat Converter::Quation2CvMat(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  ) {
    return toCvMat(
            Quation2Eigen(qx, qy, qz, qw, tx, ty, tz )
    );
}

Eigen::Isometry3d  Converter::Matrix4dtoIsometry3d(const Eigen::Matrix4d &matrix) {
    Eigen::Isometry3d Iso=Eigen::Isometry3d::Identity();                

    Iso(0, 0) = matrix(0, 0), Iso(0, 1) = matrix(0, 1), Iso(0, 2) = matrix(0, 2), Iso(0, 3) = matrix(0, 3);
    Iso(1, 0) = matrix(1, 0), Iso(1, 1) = matrix(1, 1), Iso(1, 2) = matrix(1, 2), Iso(1, 3) = matrix(1, 3);
    Iso(2, 0) = matrix(2, 0), Iso(2, 1) = matrix(2, 1), Iso(2, 2) = matrix(2, 2), Iso(2, 3) = matrix(2, 3);
    Iso(3, 0) = matrix(3, 0), Iso(3, 1) = matrix(3, 1), Iso(3, 2) = matrix(3, 2), Iso(3, 3) = matrix(3, 3);

    return Iso;
}

Eigen::Matrix4d Converter::Isometry3dtoMatrix4d(const Eigen::Isometry3d &Iso ){
    return Iso.matrix();
}

Eigen::Isometry3d Converter::cvMattoIsometry3d(const cv::Mat &cvMat4){
    return Matrix4dtoIsometry3d(
            cvMattoMatrix4d( cvMat4 )
            );
}

Eigen::Quaterniond Converter::ExtractQuaterniond(const Eigen::Isometry3d &Iso){
    Eigen::Quaterniond q = Eigen::Quaterniond(Iso.rotation());
    return q;
}

Eigen::Quaterniond Converter::ExtractQuaterniond(const Eigen::Matrix4d &matrix ){
    return ExtractQuaterniond(
            Matrix4dtoIsometry3d(matrix)
    );
}

Eigen::Quaterniond Converter::ExtractQuaterniond(const cv::Mat &mat ){
    return ExtractQuaterniond(
            cvMattoIsometry3d(mat)
    );
}

} //namespace ORB_SLAM
