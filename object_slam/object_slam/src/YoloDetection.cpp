//
// Created by lacie on 18/01/2023.
//

#include "YoloDetection.h"
#include "yolov5/include/types.h"

YoloDetection::YoloDetection(std::string modelPath, bool isTensorRT)
{
    if(!isTensorRT)
    {
        // std::cout << "Create Torch model \n";
        // mModule = torch::jit::load(modelPath + "/yolov5s.torchscript.pt");
    }
    else
    {
        std::cout << "Create TensorRT Model \n";
        mModel = new YoLoObjectDetection(modelPath + "/yolov5s.engine");
    }

    std::ifstream f(modelPath + "/coco.names");
    std::string name = "";
    while (std::getline(f, name))
    {
        mClassnames.push_back(name);
    }
    mvDynamicNames = {"person", "car", "motorbike", "bus", "train", "truck", "boat", "bird", "cat",
                      "dog", "horse", "sheep", "crow", "bear"};
}

YoloDetection::~YoloDetection()
{

}

bool YoloDetection::Detect()
{

    // cv::Mat img;

    // if(mRGB.empty())
    // {
    //     std::cout << "Read RGB failed!" << std::endl;
    //     return false;
    // }

    // // Preparing input tensor
    // cv::resize(mRGB, img, cv::Size(640, 380));
    // cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    // torch::Tensor imgTensor = torch::from_blob(img.data, {img.rows, img.cols,3},torch::kByte);
    // imgTensor = imgTensor.permute({2,0,1});
    // imgTensor = imgTensor.toType(torch::kFloat);
    // imgTensor = imgTensor.div(255);
    // imgTensor = imgTensor.unsqueeze(0);

    // // preds: [?, 15120, 9]
    // torch::Tensor preds = mModule.forward({imgTensor}).toTuple()->elements()[0].toTensor();
    // std::vector<torch::Tensor> dets = YoloDetection::non_max_suppression(preds, 0.4, 0.5);
    // if (dets.size() > 0)
    // {
    //     // Visualize result
    //     for (size_t i=0; i < dets[0].sizes()[0]; ++ i)
    //     {
    //         float left = dets[0][i][0].item().toFloat() * mRGB.cols / 640;
    //         float top = dets[0][i][1].item().toFloat() * mRGB.rows / 384;
    //         float right = dets[0][i][2].item().toFloat() * mRGB.cols / 640;
    //         float bottom = dets[0][i][3].item().toFloat() * mRGB.rows / 384;
    //         int classID = dets[0][i][5].item().toInt();


    //         cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));
    //         mmDetectMap[mClassnames[classID]].push_back(DetectArea);

    //         if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
    //         {
    //             cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
    //             mvDynamicArea.push_back(DynamicArea);
    //         }

    //     }
    //     if (mvDynamicArea.size() == 0)
    //     {
    //         cv::Rect2i tDynamicArea(1, 1, 1, 1);
    //         mvDynamicArea.push_back(tDynamicArea);
    //     }
    // }
    return true;
}

bool YoloDetection::Detect(const cv::Mat &bgr_img, std::vector<Object> &objects) {

//     cv::Mat img;

//     mRGB = bgr_img.clone();

//     if(bgr_img.empty())
//     {
//         std::cout << "Read RGB failed!" << std::endl;
//         return false;
//     }

//     // Preparing input tensor
//     cv::resize(mRGB, img, cv::Size(640, 380));
//     cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
//     torch::Tensor imgTensor = torch::from_blob(img.data, {img.rows, img.cols,3},torch::kByte);
//     imgTensor = imgTensor.permute({2,0,1});
//     imgTensor = imgTensor.toType(torch::kFloat);
//     imgTensor = imgTensor.div(255);
//     imgTensor = imgTensor.unsqueeze(0);

//     // preds: [?, 15120, 9]
//     torch::Tensor preds = mModule.forward({imgTensor}).toTuple()->elements()[0].toTensor();
//     std::vector<torch::Tensor> dets = YoloDetection::non_max_suppression(preds, 0.4, 0.5);
//     if (dets.size() > 0)
//     {
//         // Visualize result
//         for (size_t i=0; i < dets[0].sizes()[0]; ++ i)
//         {
//             float left = dets[0][i][0].item().toFloat() * mRGB.cols / 640;
//             float top = dets[0][i][1].item().toFloat() * mRGB.rows / 384;
//             float right = dets[0][i][2].item().toFloat() * mRGB.cols / 640;
//             float bottom = dets[0][i][3].item().toFloat() * mRGB.rows / 384;
//             int classID = dets[0][i][5].item().toInt();

//             cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));
//             mmDetectMap[mClassnames[classID]].push_back(DetectArea);

//             Object tmp;
//             tmp.rect = DetectArea;
//             tmp.class_id = classID;
//             tmp.object_name = mClassnames[classID];

//             std::cout << "2d object_roi: " << DetectArea.x     << " "
//                                           << DetectArea.y     << " "
//                                           << DetectArea.width << " "
//                                           << DetectArea.height << std::endl;

//             objects.push_back(tmp);

//             if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
//             {
//                 cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
//                 mvDynamicArea.push_back(DynamicArea);
//             }

//         }
//         if (mvDynamicArea.size() == 0)
//         {
//             cv::Rect2i tDynamicArea(1, 1, 1, 1);
//             mvDynamicArea.push_back(tDynamicArea);
//         }
//     }

// //    std::cout << "Number of objects: " << objects.size() << "\n";

//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         const Object& obj = objects[i];

//         cv::rectangle(mRGB, obj.rect, cv::Scalar(255, 0, 0));
//         char text[256];
//         sprintf(text, "%s %.1f%%", obj.object_name.c_str(), obj.prob * 100);

//         int baseLine = 0;
//         cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

//         int x = obj.rect.x;
//         int y = obj.rect.y - label_size.height - baseLine;
//         if (y < 0)
//             y = 0;
//         if (x + label_size.width > mRGB.cols)
//             x = mRGB.cols - label_size.width;

//         cv::rectangle(mRGB, cv::Rect(cv::Point(x, y),
//                                       cv::Size(label_size.width, label_size.height + baseLine)),
//                       cv::Scalar(255, 255, 255), cv::FILLED);

//         cv::putText(mRGB, text, cv::Point(x, y + label_size.height),
//                     cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//     }

//     // cv::imshow("image", mRGB);

    return true;
}

bool YoloDetection::Detectv2(const cv::Mat &bgr_img, std::vector<Object> &objects)
{
    cv::Mat img;

    img = bgr_img.clone();

    if(bgr_img.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }

    std::vector<Detection> res; 

    mModel->detectObject(img, res);

    for(int i = 0; i < res.size(); i++)
    {
        Object ob;
        ob.rect = mModel->get_rect(img, res[i].bbox);
        std::cout << "Copy end \n";
        std::cout << res[i].class_id << std::endl;
        ob.object_name = mClassnames[(int)res[i].class_id];
        std::cout << "Copy end \n";
        ob.prob = res[i].conf;
        ob.class_id = res[i].class_id;

        objects.push_back(ob);
    }

    return true;
}

// For 3D cuboid
bool YoloDetection::Detectv3(const cv::Mat &bgr_img, std::vector<BoxSE> &objects)
{
    cv::Mat img;

    img = bgr_img.clone();

    if(bgr_img.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }

    std::vector<Detection> res; 

    mModel->detectObject(img, res);

    std::cout << "Object detected: " << res.size() << "\n";

    // TODO: convert to BoxSE
    for(int i = 0; i < res.size(); i++)
    {
        BoxSE ob;
        cv::Rect rec = mModel->get_rect(img, res[i].bbox);

        // BBox
        ob.x = rec.x;
        ob.y = rec.y;
        ob.width = rec.width;
        ob.height = rec.height;

//        std::cout << res[i].class_id << std::endl;
        ob.m_class_name = mClassnames[(int)res[i].class_id];

        ob.m_score = res[i].conf;
        ob.m_class = res[i].class_id;

        objects.push_back(ob);
    }

    std::sort(objects.begin(), objects.end(), [](BoxSE a, BoxSE b)->bool { return a.m_score > b.m_score; });

    return true;
}

cv::Mat YoloDetection::display(std::vector<Object> &objects) {

    cv::Mat image = mRGB.clone();

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        cv::rectangle(image, obj.rect, cv::Scalar(255, 0, 0));
        char text[256];
        sprintf(text, "%s %.1f%%", obj.object_name.c_str(), obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y),
                                      cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), cv::FILLED);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    cv::imshow("image", image);
//    cv::waitKey(0);
}

// vector<torch::Tensor> YoloDetection::non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh)
// {
//     std::vector<torch::Tensor> output;
//     for (size_t i=0; i < preds.sizes()[0]; ++i)
//     {
//         torch::Tensor pred = preds.select(0, i);

//         // Filter by scores
//         torch::Tensor scores = pred.select(1, 4) * std::get<0>( torch::max(pred.slice(1, 5, pred.sizes()[1]), 1));
//         pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).select(1, 0));
//         if (pred.sizes()[0] == 0) continue;

//         // (center_x, center_y, w, h) to (left, top, right, bottom)
//         pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
//         pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
//         pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
//         pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

//         // Computing scores and classes
//         std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(pred.slice(1, 5, pred.sizes()[1]), 1);
//         pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
//         pred.select(1, 5) = std::get<1>(max_tuple);

//         torch::Tensor  dets = pred.slice(1, 0, 6);

//         torch::Tensor keep = torch::empty({dets.sizes()[0]});
//         torch::Tensor areas = (dets.select(1, 3) - dets.select(1, 1)) * (dets.select(1, 2) - dets.select(1, 0));
//         std::tuple<torch::Tensor, torch::Tensor> indexes_tuple = torch::sort(dets.select(1, 4), 0, 1);
//         torch::Tensor v = std::get<0>(indexes_tuple);
//         torch::Tensor indexes = std::get<1>(indexes_tuple);
//         int count = 0;
//         while (indexes.sizes()[0] > 0)
//         {
//             keep[count] = (indexes[0].item().toInt());
//             count += 1;

//             // Computing overlaps
//             torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1);
//             torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1);
//             torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1);
//             torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1);
//             torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1);
//             torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1);
//             for (size_t i=0; i<indexes.sizes()[0] - 1; ++i)
//             {
//                 lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(), dets[indexes[i + 1]][0].item().toFloat());
//                 tops[i] = std::max(dets[indexes[0]][1].item().toFloat(), dets[indexes[i + 1]][1].item().toFloat());
//                 rights[i] = std::min(dets[indexes[0]][2].item().toFloat(), dets[indexes[i + 1]][2].item().toFloat());
//                 bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(), dets[indexes[i + 1]][3].item().toFloat());
//                 widths[i] = std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
//                 heights[i] = std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
//             }
//             torch::Tensor overlaps = widths * heights;

//             // FIlter by IOUs
//             torch::Tensor ious = overlaps / (areas.select(0, indexes[0].item().toInt()) + torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
//             indexes = torch::index_select(indexes, 0, torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
//         }
//         keep = keep.toType(torch::kInt64);
//         output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
//     }
//     return output;
// }

void YoloDetection::GetImage(cv::Mat &RGB)
{
    mRGB = RGB;
}

void YoloDetection::ClearImage()
{
    mRGB = 0;
}

void YoloDetection::ClearArea()
{
    mvPersonArea.clear();
}
