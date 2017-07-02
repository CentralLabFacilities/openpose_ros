#define USE_CAFFE
#include "ros/ros.h"

#include <openpose/pose/poseExtractor.hpp>
#include <openpose/pose/poseExtractorCaffe.hpp>
#include <openpose/pose/poseParameters.hpp>
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/init.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <openpose_ros/DetectPeople.h>
#include <openpose_ros/PersonDetection.h>
#include <openpose_ros/BodyPartDetection.h>


std::shared_ptr<op::PoseExtractor> poseExtractor;
op::PoseModel  poseModel;
op::Point<int> netInputSize;
op::Point<int> netOutputSize;
op::Point<int> outputSize;
std::map<unsigned int, std::string> cocoBodyParts;
int scaleNumber;
double scaleGap;
cv::Mat inputImage;
std::mutex imageMutex;
std::mutex pointcloudMutex;
bool visualize;
sensor_msgs::PointCloud2 pointcloud;
tf::StampedTransform transform;

openpose_ros::BodyPartDetection initBodyPartDetection();
openpose_ros::PersonDetection initPersonDetection();

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cvBridge;
  try
  {
    cvBridge = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge failed to convert sensor msg: %s", e.what());
    return;
  }
  imageMutex.lock();
  inputImage = cvBridge->image;
  if(visualize){
    cv::imshow("input",inputImage);
    cv::waitKey(3);
  }
  imageMutex.unlock();
}

void pointcloudCb(const sensor_msgs::PointCloud2 pCloud){
    pointcloudMutex.lock();
    pointcloud = pCloud;
    pointcloudMutex.unlock();
}

bool detectPeopleCb(openpose_ros::DetectPeople::Request& req, openpose_ros::DetectPeople::Response& res)
{
    ROS_INFO("Called detect people service.");
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    op::CvMatToOpInput cvMatToOpInput{netInputSize, scaleNumber, (float)scaleGap};
    ROS_INFO("Converting cv image to openpose array.");
    imageMutex.lock();
    pointcloudMutex.lock();
    ROS_INFO("Im cols %d, im rows %d", inputImage.cols, inputImage.rows);
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput.format(inputImage);
    ROS_INFO("Detect poses using forward pass.");
    poseExtractor->forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
    const auto poseKeypoints = poseExtractor->getPoseKeypoints();

    if(visualize){
        op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, true, (float)0.6};
        op::OpOutputToCvMat opOutputToCvMat{outputSize};
        double scaleInputToOutput;
        op::Array<float> outputArray;
        op::CvMatToOpOutput cvMatToOpOutput{outputSize};
        std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(inputImage);
        poseRenderer.renderPose(outputArray, poseKeypoints);
        auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

        cv::imshow("Detections",outputImage);
        cv::waitKey(3);
    }

    ROS_INFO("Extracted %d people.",poseKeypoints.getSize(0));
    for(size_t i = 0; i<poseKeypoints.getSize(0); ++i) {

        openpose_ros::PersonDetection person = initPersonDetection();
        for(size_t j = 0; j<poseKeypoints.getSize(1); ++j) {

            size_t bodypart_id = 3*(i*poseKeypoints.getSize(1) + j);
            openpose_ros::BodyPartDetection bodypart = initBodyPartDetection();
            bodypart.confidence = poseKeypoints[bodypart_id + 2];

            //credit to Saurav Agarwal (http://sauravag.com/2016/11/how-to-get-depth-xyz-of-a-2d-pixel-from-pointcloud2-or-kinect-data/)
            //for conversion method
            int u = poseKeypoints[bodypart_id];
            int v = poseKeypoints[bodypart_id + 1];

            int pcWidth = pointcloud.width;
            int pcHeight = pointcloud.height;

            int arrayPosition = v*pointcloud.row_step + u*pointcloud.point_step;
            int arrayPosX = arrayPosition + pointcloud.fields[0].offset; // X has an offset of 0
            int arrayPosY = arrayPosition + pointcloud.fields[1].offset; // Y has an offset of 4
            int arrayPosZ = arrayPosition + pointcloud.fields[2].offset; // Z has an offset of 8

            float X = 0.0;
            float Y = 0.0;
            float Z = 0.0;

            memcpy(&X, &pointcloud.data[arrayPosX], sizeof(float));
            memcpy(&Y, &pointcloud.data[arrayPosY], sizeof(float));
            memcpy(&Z, &pointcloud.data[arrayPosZ], sizeof(float));

            tf::Transform baseToCam;
            tf::Transform bodyPartTransform;
            bodyPartTransform.setOrigin(tf::Vector3(X,Y,Z));

            baseToCam.mult(bodyPartTransform,transform.inverse());

            bodypart.pos.x = baseToCam.getOrigin().x();
            bodypart.pos.y = baseToCam.getOrigin().y();
            bodypart.pos.z = baseToCam.getOrigin().z();

            std::string bodypart_name = cocoBodyParts[j];


            if (bodypart_name == "Nose") person.Nose = bodypart;
            else if (bodypart_name == "Neck") person.Neck = bodypart;
            else if (bodypart_name == "RShoulder") person.RShoulder = bodypart;
            else if (bodypart_name == "RElbow") person.RElbow = bodypart;
            else if (bodypart_name == "RWrist") person.RWrist = bodypart;
            else if (bodypart_name == "LShoulder") person.LShoulder = bodypart;
            else if (bodypart_name == "LElbow") person.LElbow = bodypart;
            else if (bodypart_name == "LWrist") person.LWrist = bodypart;
            else if (bodypart_name == "RHip") person.RHip = bodypart;
            else if (bodypart_name == "RKnee") person.RKnee = bodypart;
            else if (bodypart_name == "RAnkle") person.RAnkle = bodypart;
            else if (bodypart_name == "LHip") person.LHip = bodypart;
            else if (bodypart_name == "LKnee") person.LKnee = bodypart;
            else if (bodypart_name == "LAnkle") person.LAnkle = bodypart;
            else if (bodypart_name == "REye") person.REye = bodypart;
            else if (bodypart_name == "LEye") person.LEye = bodypart;
            else if (bodypart_name == "REar") person.REar = bodypart;
            else if (bodypart_name == "LEar") person.LEar = bodypart;
            else if (bodypart_name == "Chest") person.Chest = bodypart;
            else
            {
              ROS_ERROR("Detected Bodypart %s not in COCO model. Is openpose running with different model?", bodypart_name.c_str());
            }
        }
        res.people_list.push_back(person);
    }
    imageMutex.unlock();
    pointcloudMutex.unlock();
    return true;
}

openpose_ros::PersonDetection initPersonDetection() {
    openpose_ros::PersonDetection person;
    person.Nose = initBodyPartDetection();
    person.Neck = initBodyPartDetection();
    person.RShoulder = initBodyPartDetection();
    person.RElbow = initBodyPartDetection();
    person.RWrist = initBodyPartDetection();
    person.LShoulder = initBodyPartDetection();
    person.LElbow = initBodyPartDetection();
    person.LWrist = initBodyPartDetection();
    person.RHip = initBodyPartDetection();
    person.RKnee = initBodyPartDetection();
    person.RAnkle = initBodyPartDetection();
    person.LHip = initBodyPartDetection();
    person.LKnee = initBodyPartDetection();
    person.LAnkle = initBodyPartDetection();
    person.REye = initBodyPartDetection();
    person.LEye = initBodyPartDetection();
    person.REar = initBodyPartDetection();
    person.LEar = initBodyPartDetection();
    person.Chest = initBodyPartDetection();
    return person;
}

openpose_ros::BodyPartDetection initBodyPartDetection() {
    openpose_ros::BodyPartDetection bodypart;
    bodypart.pos.x = 0.0;
    bodypart.pos.y = 0.0;
    bodypart.pos.z = 0.0;
    bodypart.confidence = 0;
    return bodypart;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_people_server");
	
	ros::NodeHandle localNH("~");

    //read in params
    int netInputSizeWidth;
    ros::param::param("net_input_size_width", netInputSizeWidth, 320);
    int netInputSizeHeight;
    ros::param::param("net_input_size_height", netInputSizeHeight, 160);
    netInputSize = op::Point<int>(netInputSizeWidth,netInputSizeHeight);

    int netOutputSizeWidth;
    ros::param::param("net_output_size_width", netOutputSizeWidth, 320);
    int netOutputSizeHeight;
    ros::param::param("net_output_size_height", netOutputSizeHeight, 160);
    netOutputSize = op::Point<int>(netOutputSizeWidth,netOutputSizeHeight);

    int outputSizeWidth;
    ros::param::param("output_size_width", outputSizeWidth, 640);
    int outputSizeHeight;
    ros::param::param("output_size_height", outputSizeHeight, 480);
    outputSize = op::Point<int>(outputSizeWidth,outputSizeHeight);

    ros::param::param("scale_number", scaleNumber, 1);
    ros::param::param("scale_gap", scaleGap, 0.3);

    poseModel = op::PoseModel::COCO_18;

    std::string modelsFolder;
    ros::param::param("models_folder", modelsFolder, std::string("/vol/robocup/nightly/share/openpose/models/"));

    int gpuId;
    ros::param::param("gpu_id", gpuId, 0);

    cocoBodyParts = op::POSE_COCO_BODY_PARTS;

    ros::param::param("visualize", visualize, false);

    //init openpose
    poseExtractor = std::shared_ptr<op::PoseExtractorCaffe>(new op::PoseExtractorCaffe(netInputSize, netOutputSize,
                        outputSize, scaleNumber, poseModel, modelsFolder, gpuId));
    poseExtractor->initializationOnThread();

    ros::NodeHandle n;
    //advertise service to get detected people poses
    ros::ServiceServer service = n.advertiseService("detect_people", detectPeopleCb);
    //subscriber to recieve rgb image
    ros::Subscriber imageSub = n.subscribe("/xtion/rgb/image_raw", 100, imageCb);
    //subscriber to recieve pointcloud
    ros::Subscriber pointcloudSub = n.subscribe("/xtion/depth/points", 100, pointcloudCb);

    //create tf listener to transform positions of detected into robot coordinates
    tf::TransformListener listener;

    std::string cameraFrame;
    ros::param::param("camera_frame", cameraFrame, std::string("xtion_rgb_optical_frame"));
    std::string base_frame;
    ros::param::param("base_frame", base_frame, std::string("base_link"));

    try {
        listener.waitForTransform(cameraFrame, base_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(cameraFrame, base_frame, ros::Time(0), transform);
        ROS_INFO("Got TF");
    } catch (tf::TransformException ex) {
        ROS_ERROR("Could not get transform from camera frame to base frame: %s",ex.what());
        return 1;
    }

    ROS_INFO("Init done. Can start detecting people.");
    ros::spin();

    return 0;
}
