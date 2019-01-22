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

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/Posture.h>
#include <openpose_ros_msgs/Gesture.h>
#include <openpose_ros_msgs/DetectPeople.h>
#include <openpose_ros_msgs/PersonDetection.h>
#include <openpose_ros_msgs/BodyPartDetection.h>
#include <openpose_ros_msgs/GetCrowdAttributesWithPose.h>
#include <openpose_ros_msgs/PersonAttributesWithPose.h>
#include <openpose_ros_msgs/GetFollowRoi.h>
#include <bayes_people_tracker_msgs/PeopleTrackerImage.h>
#include <algorithm>
#include <math.h>
#include <gender_and_age_msgs/GenderAndAgeService.h>

#include <clf_perception_vision_msgs/DoIKnowThatPersonImage.h>
#include <clf_perception_vision_msgs/LearnPersonImage.h>
#include <clf_perception_vision_msgs/LearnPerson.h>


#include <tf/transform_listener.h>

#include <ros/package.h>
#include <cstdint>
#include <opencv2/core/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define PI 3.14159265

enum gesture{POINTING_LEFT = openpose_ros_msgs::Gesture::POINTING_LEFT, POINTING_RIGHT = openpose_ros_msgs::Gesture::POINTING_RIGHT, RAISING_LEFT_ARM = openpose_ros_msgs::Gesture::RAISING_LEFT_ARM, RAISING_RIGHT_ARM = openpose_ros_msgs::Gesture::RAISING_RIGHT_ARM, WAVING = openpose_ros_msgs::Gesture::WAVING, NEUTRAL = openpose_ros_msgs::Gesture::NEUTRAL};
enum posture{SITTING = openpose_ros_msgs::Posture::SITTING, STANDING = openpose_ros_msgs::Posture::STANDING, LYING = openpose_ros_msgs::Posture::LYING};

const char* gesture_name[] = { "POINTING_LEFT", "POINTING_RIGHT", "RAISING_LEFT_ARM", "RAISING_RIGHT_ARM", "WAVING", "NEUTRAL" } ;
const char* posture_name[] = { "SITTING", "STANDING", "LYING" } ;
int gpu_id;
std::string models_folder;
std::shared_ptr <op::PoseExtractor> pose_extractor;

// OP
op::PoseModel pose_model;
op::Point<int> net_input_size;
op::Point<int> net_output_size;
op::Point<int> output_size;
// OP IN
op::CvMatToOpInput *cvMatToOpInput;
// OP OUT
op::CvMatToOpOutput cvMatToOpOutput;
// OP EXTRACT
op::PoseExtractorCaffe *poseExtractorCaffe;

op::PoseCpuRenderer *pose_renderer;
op::OpOutputToCvMat opOutputToCvMat;

image_transport::Publisher resImgPub;

std::map<unsigned int, std::string> coco_body_parts;
int scale_number;
double scale_gap;

std::string input_image_depth_frame_id;
cv::Mat input_image_rgb_crowd;
cv::Mat input_image_depth_crowd;
std::mutex image_mutex_crowd;

bool visualize;
bool gender_age = false;
bool face_id = false;
bool shirt_color = true;
double SITTINGPERCENT = 0.4;

boost::shared_ptr<ros::ServiceClient> gender_age_ptr;
boost::shared_ptr<ros::ServiceClient> recognize_face_id_ptr;
boost::shared_ptr<ros::ServiceClient> learn_face_id_ptr;

openpose_ros_msgs::PersonAttributesWithPose getPostureAndGesture(openpose_ros_msgs::PersonDetection person);
std::vector<openpose_ros_msgs::PersonAttributesWithPose> getPersonList(cv::Mat color_image, cv::Mat depth_image, std::string frame_id, bool learn_face = false, bool track_body = false);
openpose_ros_msgs::BodyPartDetection initBodyPartDetection();
openpose_ros_msgs::PersonDetection initPersonDetection();
bool getCrowdAttributesCb(openpose_ros_msgs::GetCrowdAttributesWithPose::Request &req, openpose_ros_msgs::GetCrowdAttributesWithPose::Response &res);
void getHeadBounds(openpose_ros_msgs::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image);
std::string getShirtColor(openpose_ros_msgs::PersonDetection person, cv::Mat image);
cv::Rect getUpperBodyRoi( openpose_ros_msgs::PersonDetection person, cv::Mat image );
cv::Rect getCrotchRoi( openpose_ros_msgs::PersonDetection person);

void imagesCb(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg);

tf::TransformListener* listener;

int WHITE, BLACK, GREY, RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, PURPLE = 0;


void initializeOP() {
    cvMatToOpInput = new op::CvMatToOpInput{pose_model};
    poseExtractorCaffe = new op::PoseExtractorCaffe{pose_model, models_folder, gpu_id};
    poseExtractorCaffe->initializationOnThread();
}


bool getCrowdAttributesCb(openpose_ros_msgs::GetCrowdAttributesWithPose::Request &req, openpose_ros_msgs::GetCrowdAttributesWithPose::Response &res) {

    ROS_INFO("\n------------------------- New Crowd Attributes Callback -------------------------\n");
    image_mutex_crowd.lock();
    res.attributes = getPersonList(input_image_rgb_crowd, input_image_depth_crowd, input_image_depth_frame_id);
    image_mutex_crowd.unlock();

    for( size_t i = 0 ; i < res.attributes.size() ; i++) {

        std::cout << "PERSON " << i << ": " << std::endl;
        std::cout << "\t " << "Name: " << res.attributes.at(i).attributes.name << std::endl;
        std::cout << "\t " << "Age: " << res.attributes.at(i).attributes.age_hyp.age << std::endl;
        std::cout << "\t " << "Gender: " << res.attributes.at(i).attributes.gender_hyp.gender << std::endl;
        std::cout << "\t " << "Gesture: " << gesture_name[ res.attributes.at(i).attributes.gestures.at(0).gesture - 1 ] << std::endl;
        std::cout << "\t " << "Posture: " << posture_name[ res.attributes.at(i).attributes.posture.posture - 1 ] << std::endl;
        std::cout << "\t " << "Shirtcolor: " << res.attributes.at(i).attributes.shirtcolor << std::endl;

    }

    return true;
}

bool learnFaceCb(clf_perception_vision_msgs::LearnPerson::Request &req, clf_perception_vision_msgs::LearnPerson::Response &res) {

    ROS_INFO("\n------------------------- New Learn Face Callback -------------------------\n");
    if (face_id) {
        image_mutex_crowd.lock();
        getPersonList(input_image_rgb_crowd, input_image_depth_crowd, req.name, true, false).size() == 0 ? res.success = false : res.success = true;
        image_mutex_crowd.unlock();
    } else {
        ROS_ERROR("Face id must be running to learn faces!");
    }
    return true;
}

bool shirtRoiCb(openpose_ros_msgs::GetFollowRoi::Request &req, openpose_ros_msgs::GetFollowRoi::Response &res) {

    ROS_INFO("\n------------------------- New Shirt Roi Attributes Callback -------------------------\n");
    res.roi.x_offset = 0;
    res.roi.y_offset = 0;
    res.roi.width = 0;
    res.roi.height = 0;

    image_mutex_crowd.lock();
    std::vector<openpose_ros_msgs::PersonAttributesWithPose> person = getPersonList(input_image_rgb_crowd, input_image_depth_crowd, input_image_depth_frame_id);
    image_mutex_crowd.unlock();

    if(person.at(0).pose_stamped.pose.orientation.x != 0) {
        res.roi.x_offset = person.at(0).pose_stamped.pose.orientation.x;
        res.roi.y_offset = person.at(0).pose_stamped.pose.orientation.y;
        res.roi.width = person.at(0).pose_stamped.pose.orientation.w;
        res.roi.height = person.at(0).pose_stamped.pose.orientation.z;
    } else {    
        ROS_ERROR("NO BOUNDING BOX FOR SHIRT ROI!");
        return false;
    }

    return true;
}

// This function luckily already existed in https://github.com/introlab/find-object/blob/master/src/ros/FindObjectROS.cpp (THANKS!)
cv::Vec3f getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy, float fx, float fy) {

    ROS_DEBUG("getDepth called x: %d y: %d", x, y);

    if(!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows))
    {
        ROS_ERROR(">>> Point must be inside the image (x=%d, y=%d), image size=(%d,%d)", x, y, depthImage.cols, depthImage.rows);
        return cv::Vec3f(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN());
    }

    cv::Vec3f pt;

    // Use correct principal point from calibration
    float center_x = cx; //cameraInfo.K.at(2)
    float center_y = cy; //cameraInfo.K.at(5)

    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float unit_scaling = isInMM?0.001f:1.0f;
    float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
    float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    float depth;
    bool isValid;

    if(isInMM) {
        ROS_DEBUG(">>> Image is in Millimeters");

        // Sample fore depth points to the right, left, top and down
        std::vector<float> valueList;
        for (int i=0; i<5; i++) {
            ROS_DEBUG("%f", (float)depthImage.at<uint16_t>(y,x+i));
            if((float)depthImage.at<uint16_t>(y,x+i) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y,x+i));
            }
            ROS_DEBUG("%f", (float)depthImage.at<uint16_t>(y,x-i));
            if((float)depthImage.at<uint16_t>(y,x-i) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y,x-i));
            }
            ROS_DEBUG("%f", (float)depthImage.at<uint16_t>(y+i,x));
            if((float)depthImage.at<uint16_t>(y+i,x) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y+i,x));
            }
            ROS_DEBUG("%f", (float)depthImage.at<uint16_t>(y-i,x));
            if((float)depthImage.at<uint16_t>(y-i,x) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y-i,x));
            }
        }

        ROS_DEBUG("Sampled %d values", valueList.size());

        if(!valueList.empty()) {
            std::sort (valueList.begin(), valueList.end());
            float median = valueList[(int)(valueList.size()/2)];
            depth = median;
            ROS_DEBUG("%f", depth);
            isValid = true;
        } else {
            depth = 0;
            isValid = false;
        }

    } else {
        ROS_DEBUG(">>> Image is in Meters");
        float depth_samples[21];

        // Sample fore depth points to the right, left, top and down
        for (int i=0; i<5; i++) {
            depth_samples[i] = depthImage.at<float>(y,x+i);
            depth_samples[i+5] = depthImage.at<float>(y,x-i);
            depth_samples[i+10] = depthImage.at<float>(y+i,x);
            depth_samples[i+15] = depthImage.at<float>(y-i,x);
        }

        depth_samples[20] = depthImage.at<float>(y,x);

        int arr_size = sizeof(depth_samples)/sizeof(float);
        std::sort(&depth_samples[0], &depth_samples[arr_size]);
        float median = arr_size % 2 ? depth_samples[arr_size/2] : (depth_samples[arr_size/2-1] + depth_samples[arr_size/2]) / 2;

        depth = median;
        ROS_DEBUG("%f", depth);
        isValid = std::isfinite(depth);
    }

    // Check for invalid measurements
    if (!isValid)
    {
        ROS_DEBUG(">>> WARN Image is invalid, whoopsie.");
        pt.val[0] = pt.val[1] = pt.val[2] = bad_point;
    } else{
        // Fill in XYZ
        pt.val[0] = (float(x) - center_x) * depth * constant_x;
        pt.val[1] = (float(y) - center_y) * depth * constant_y;
        pt.val[2] = depth*unit_scaling;
    }

    ROS_DEBUG("DEPTH %f %f %f", pt(0), pt(1), pt(2));

    return pt;
}


std::vector<openpose_ros_msgs::PersonAttributesWithPose> getPersonList(cv::Mat color_image, cv::Mat depth_image, std::string frame_id, bool learn_face, bool track_body) {

    std::vector<openpose_ros_msgs::PersonAttributesWithPose> res;
    op::Array<float> net_input_array;
    std::vector<float> scale_ratios;

    ROS_DEBUG("Converting cv image to openpose array.");
    ROS_DEBUG("Im cols %d, im rows %d", color_image.cols, color_image.rows);
    const op::Point<int> image_size{color_image.cols, color_image.rows};

    std::vector<double> scale_input_to_net_inputs;
    std::vector<op::Point<int>> net_input_sizes;
    double scale_input_to_output;
    op::Point<int> output_resolution;
    op::ScaleAndSizeExtractor scaleAndSizeExtractor(net_input_size, output_size, scale_number, scale_gap);
    std::tie(scale_input_to_net_inputs, net_input_sizes, scale_input_to_output, output_resolution)
        = scaleAndSizeExtractor.extract(image_size);

    const auto netInputArray = cvMatToOpInput->createArray(color_image, scale_input_to_net_inputs, net_input_sizes);

    ROS_DEBUG("Detect poses using forward pass.");
    poseExtractorCaffe->forwardPass(netInputArray, image_size, scale_input_to_net_inputs);
    const auto pose_key_points = poseExtractorCaffe->getPoseKeypoints();


    ROS_DEBUG("Extracted %d people.", pose_key_points.getSize(0));
    if( pose_key_points.getSize(0) == 0 ) {
        return res;
    }
    cv::Mat output_image;

    if( learn_face ) {

        int crop_x, crop_y, crop_width, crop_height;
        float dist = 9999;
        int n = 0;
        clf_perception_vision_msgs::LearnPersonImage learn_face_id_srv;

        std::vector<openpose_ros_msgs::PersonDetection> person_list;

        for (size_t i = 0; i < pose_key_points.getSize(0); ++i) {
            openpose_ros_msgs::PersonDetection person = initPersonDetection();
            for (size_t j = 0; j < pose_key_points.getSize(1); ++j) {
                size_t bodypart_id = 3 * (i * pose_key_points.getSize(1) + j);
                openpose_ros_msgs::BodyPartDetection bodypart = initBodyPartDetection();
                bodypart.confidence = pose_key_points[bodypart_id + 2];
                int u = pose_key_points[bodypart_id];
                int v = pose_key_points[bodypart_id + 1];

                bodypart.u = u;
                bodypart.v = v;

                ROS_DEBUG("u: %d, v: %d", u, v);

                std::string bodypart_name = coco_body_parts[j];

                ROS_DEBUG("BodyPartName: %s", bodypart_name.c_str());
                ROS_DEBUG("Confidence: %f", bodypart.confidence);

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
                else {
                    ROS_ERROR("Detected Bodypart %s not in COCO model. Is openpose running with different model?",
                    bodypart_name.c_str());
                }
            }
            getHeadBounds(person, crop_x, crop_y, crop_width, crop_height, color_image);

            cv::Vec3f pt = getDepth( depth_image, (crop_x + crop_width/2) / (color_image.cols/depth_image.cols), (crop_y + crop_height/2) / (color_image.rows/depth_image.rows),
                                     161.05772510763725, 120.01067491252732, 286.4931637345315, 286.7532312956228 ); //TODO: Remove hardcoding!

            if( pt[0] < dist  && pt[0] != 0.0f ) {
                dist = pt[0];
                n = i;
            }
            person_list.push_back(person);


        }

        try {
            getHeadBounds(person_list.at(n), crop_x, crop_y, crop_width, crop_height, color_image);
            if(crop_x >= 0) {
                cv::Rect roi;
                roi.x = crop_x;
                roi.y = crop_y;
                roi.width = crop_width;
                roi.height = crop_height;
                cv::Mat crop = color_image(roi);
                if(visualize){
                    cv::imshow("CLF Learn Face | FaceID", crop);
                    cv::waitKey(3);
                }
                sensor_msgs::ImagePtr inputImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
                learn_face_id_srv.request.name = frame_id;
                learn_face_id_srv.request.roi = *inputImage_msg;

                learn_face_id_ptr.get()->call(learn_face_id_srv);
            }
        } catch (cv::Exception e) {
                std::cout << "Exception in learn face! ROI could be wrong!" << e.what();
        }

        if(learn_face_id_srv.response.success) {

            openpose_ros_msgs::PersonAttributesWithPose attributes;
            res.push_back(attributes);
        }
        return res;

    } else if(track_body) {

        int crop_x, crop_y, crop_width, crop_height;
        float dist = 9999;
        int n = 0;
        clf_perception_vision_msgs::LearnPersonImage learn_face_id_srv;

        std::vector<openpose_ros_msgs::PersonDetection> person_list;

        for (size_t i = 0; i < pose_key_points.getSize(0); ++i) {
            openpose_ros_msgs::PersonDetection person = initPersonDetection();
            for (size_t j = 0; j < pose_key_points.getSize(1); ++j) {
                size_t bodypart_id = 3 * (i * pose_key_points.getSize(1) + j);
                openpose_ros_msgs::BodyPartDetection bodypart = initBodyPartDetection();
                bodypart.confidence = pose_key_points[bodypart_id + 2];
                int u = pose_key_points[bodypart_id];
                int v = pose_key_points[bodypart_id + 1];

                bodypart.u = u;
                bodypart.v = v;

                ROS_DEBUG("u: %d, v: %d", u, v);

                std::string bodypart_name = coco_body_parts[j];

                ROS_DEBUG("BodyPartName: %s", bodypart_name.c_str());
                ROS_DEBUG("Confidence: %f", bodypart.confidence);

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
                else {
                    ROS_ERROR("Detected Bodypart %s not in COCO model. Is openpose running with different model?",
                    bodypart_name.c_str());
                }
            }
            cv::Rect roi = getUpperBodyRoi(person, color_image);

            cv::Vec3f pt = getDepth( depth_image, (roi.x + roi.width/2) / (color_image.cols/depth_image.cols), (roi.y + roi.height/2) / (color_image.rows/depth_image.rows),
                                     161.05772510763725, 120.01067491252732, 286.4931637345315, 286.7532312956228 ); //TODO: Remove hardcoding!

            if( pt[0] < dist  && pt[0] != 0.0f ) {
                dist = pt[0];
                n = i;
            }
            person_list.push_back(person);
        }

        cv::Rect roi = getCrotchRoi(person_list.at(n));
        openpose_ros_msgs::PersonAttributesWithPose attributes;
        attributes.pose_stamped.pose.orientation.x = roi.x;
        attributes.pose_stamped.pose.orientation.y = roi.y;
        attributes.pose_stamped.pose.orientation.z = roi.height;
        attributes.pose_stamped.pose.orientation.w = roi.width;
        res.push_back(attributes);


    } else {

        gender_and_age_msgs::GenderAndAgeService gender_age_srv;
        clf_perception_vision_msgs::DoIKnowThatPersonImage face_id_srv;
        std::vector<std::string> shirt_list;
        std::vector<openpose_ros_msgs::PersonDetection> person_list;

        if (visualize) {
            pose_renderer = new  op::PoseCpuRenderer{pose_model,0.5,true,0.5,0.5};
            op::Array<float> output_array;
            output_array = cvMatToOpOutput.createArray(color_image, scale_input_to_output, output_resolution);
            pose_renderer->renderPose(output_array,pose_key_points,scale_input_to_output);
            output_image = opOutputToCvMat.formatToCvMat(output_array);
        }

        for (size_t i = 0; i < pose_key_points.getSize(0); ++i) {
            openpose_ros_msgs::PersonDetection person = initPersonDetection();
            for (size_t j = 0; j < pose_key_points.getSize(1); ++j) {
                size_t bodypart_id = 3 * (i * pose_key_points.getSize(1) + j);
                openpose_ros_msgs::BodyPartDetection bodypart = initBodyPartDetection();
                bodypart.confidence = pose_key_points[bodypart_id + 2];
                int u = pose_key_points[bodypart_id];
                int v = pose_key_points[bodypart_id + 1];

                bodypart.u = u;
                bodypart.v = v;

                ROS_DEBUG("u: %d, v: %d", u, v);

                std::string bodypart_name = coco_body_parts[j];

                ROS_DEBUG("BodyPartName: %s", bodypart_name.c_str());
                ROS_DEBUG("Confidence: %f", bodypart.confidence);

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
                else {
                    ROS_ERROR("Detected Bodypart %s not in COCO model. Is openpose running with different model?",
                    bodypart_name.c_str());
                }

            }

            if(gender_age) {
                try {
                    int crop_x, crop_y, crop_width, crop_height;
                    getHeadBounds(person,crop_x, crop_y, crop_width, crop_height, color_image);
                    if(crop_x >= 0) {
                        cv::Rect roi;
                        roi.x = crop_x;
                        roi.y = crop_y;
                        roi.width = crop_width;
                        roi.height = crop_height;
                        cv::Mat crop = color_image(roi);
                        if(visualize){
                            cv::imshow("CLF OpenPose | GA", crop);
                            cv::waitKey(3);
                        }
                        sensor_msgs::ImagePtr inputImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
                        gender_age_srv.request.objects.push_back(*inputImage_msg);
                    }
                } catch (cv::Exception e) {
                        std::cout << "Exception in gender and age! ROI could be wrong!" << e.what();
                }

            }
            if(shirt_color) {
                    try{
                            shirt_list.push_back(getShirtColor(person, color_image));
                    } catch (cv::Exception e) {

                            std::cout << "Exception in Shirt color! ROI could be wrong!" << std::endl;
                    }
            }
            person_list.push_back(person);

        }

        if(gender_age) {
            gender_age_ptr.get()->call(gender_age_srv);
            ROS_DEBUG(">> Gender Age: %u", (int)gender_age_srv.response.gender_and_age_response.gender_and_age_list.size());
            ROS_DEBUG(">> Person Size: %u", (int)person_list.size());
            if((int)gender_age_srv.response.gender_and_age_response.gender_and_age_list.size() == (int)person_list.size()) {
                for (size_t i = 0; i < gender_age_srv.response.gender_and_age_response.gender_and_age_list.size(); ++i) {
                    std::cout << "GENDER HYPOTHESES:\t" << gender_age_srv.response.gender_and_age_response.gender_and_age_list.at(i).gender_probability << std::endl;
                    std::cout << "AGE HYPOTHESES:\t" << gender_age_srv.response.gender_and_age_response.gender_and_age_list.at(i).age_probability << std::endl;
                    person_list.at(i).gender_hyp = gender_age_srv.response.gender_and_age_response.gender_and_age_list.at(i).gender_probability;
                    person_list.at(i).age_hyp = gender_age_srv.response.gender_and_age_response.gender_and_age_list.at(i).age_probability;
                    cv::putText(output_image, std::string(person_list.at(i).gender_hyp.gender), cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v-40),
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));
                    cv::putText(output_image, std::string("A: ") + std::string(person_list.at(i).age_hyp.age), cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v-20),
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));

                }
            }
        }
        if(shirt_color) {
            for (int i = 0; i < shirt_list.size(); i++)	{
                    std::string shirtcolor = shirt_list[i];
                    ROS_DEBUG(">> Shirt color person %d: %s, ", i, shirtcolor.c_str());
                    person_list.at(i).shirtcolor = shirtcolor;
                    cv::putText(output_image, std::string("S: ") + shirtcolor, cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v),
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));
            }
        }

        for( size_t i = 0; i < person_list.size(); i++ ) {

            openpose_ros_msgs::PersonAttributesWithPose attributes = getPostureAndGesture( person_list.at(i) );

            if(visualize) {
                attributes.attributes.posture;
                person_list.at(i).Nose;
                // Draw person gesture and posture
                cv::putText(output_image, std::string("G: ") + gesture_name[ attributes.attributes.gestures.at(0).gesture - 1 ], cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v+20),
                             cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));
                cv::putText(output_image, std::string("P: ") + posture_name[ attributes.attributes.posture.posture - 1 ], cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v+40),
                             cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));
            }


            // HERE DEPTH LOOKUP FOR PERSONS! use FRAMEID FOR TF FROM CAMERA TO MAP!
            cv::Rect roi = getUpperBodyRoi( person_list.at(i),color_image );

            cv::Vec3f pt = getDepth( depth_image, (roi.x + roi.width/2) / (color_image.cols/depth_image.cols), (roi.y + roi.height/2) / (color_image.rows/depth_image.rows),
                                     161.05772510763725, 120.01067491252732, 286.4931637345315, 286.7532312956228 ); //TODO: Remove hardcoding!

            cv::Rect roidepth = cv::Rect(roi.x/2,roi.y/2,roi.width/2, roi.height/2);

            geometry_msgs::PoseStamped camera_pose;
            geometry_msgs::PoseStamped base_link_pose;
            base_link_pose.header.frame_id = "base_link";
            camera_pose.header.frame_id = frame_id;
            camera_pose.header.stamp = ros::Time::now();
            camera_pose.pose.position.x = pt(0);
            camera_pose.pose.position.y = pt(1);
            camera_pose.pose.position.z = pt(2);
            camera_pose.pose.orientation.x = 0.0;
            camera_pose.pose.orientation.y = 0.0;
            camera_pose.pose.orientation.z = 0.0;
            camera_pose.pose.orientation.w = 1.0;

            int crop_x, crop_y, crop_width, crop_height;
            ROS_DEBUG("Calling get head bounds");
            getHeadBounds(person_list.at(i),crop_x, crop_y, crop_width, crop_height, color_image);

            geometry_msgs::PoseStamped camera_pose_head;
            geometry_msgs::PoseStamped base_link_pose_head;
            base_link_pose_head.header.frame_id = "base_link";
            bool gotHead = false;

            if(crop_x >= 0) {
                gotHead = true;
                cv::Rect roiHead;
                roiHead.x = crop_x;
                roiHead.y = crop_y;
                roiHead.width = crop_width;
                roiHead.height = crop_height;

                ROS_DEBUG("Head Roi x: %d y: %d width: %d height: %d", crop_x, crop_y, crop_width, crop_height);



                cv::Vec3f pt_head = getDepth( depth_image, (roiHead.x + roiHead.width/2) /  (color_image.cols/depth_image.cols),
                                              (roiHead.y + roiHead.height/2) /  (color_image.rows/depth_image.rows),
                                         161.05772510763725, 120.01067491252732, 286.4931637345315, 286.7532312956228 ); //TODO: Remove hardcoding!

                cv::Rect roidepthhead = cv::Rect(roiHead.x,roiHead.y,roiHead.width, roiHead.height);


                camera_pose_head.header.frame_id = frame_id;
                camera_pose_head.header.stamp = ros::Time::now();
                camera_pose_head.pose.position.x = pt_head(0);
                camera_pose_head.pose.position.y = pt_head(1);
                camera_pose_head.pose.position.z = pt_head(2);
                camera_pose_head.pose.orientation.x = 0.0;
                camera_pose_head.pose.orientation.y = 0.0;
                camera_pose_head.pose.orientation.z = 0.0;
                camera_pose_head.pose.orientation.w = 1.0;
            } else {
                base_link_pose_head.header.stamp = ros::Time::now();
                base_link_pose_head.pose.position.x = NAN;
                base_link_pose_head.pose.position.y = NAN;
                base_link_pose_head.pose.position.z = NAN;
                base_link_pose_head.pose.orientation.x = 0.0;
                base_link_pose_head.pose.orientation.y = 0.0;
                base_link_pose_head.pose.orientation.z = 0.0;
                base_link_pose_head.pose.orientation.w = 1.0;
            }

            try{
                ROS_DEBUG("Transforming received position into BASELINK coordinate system.");
                listener->waitForTransform(camera_pose.header.frame_id, base_link_pose.header.frame_id, camera_pose.header.stamp, ros::Duration(20.0));
                listener->transformPose(base_link_pose.header.frame_id, ros::Time(0), camera_pose, camera_pose.header.frame_id, base_link_pose);
                if(gotHead) {
                    listener->transformPose(base_link_pose_head.header.frame_id, ros::Time(0), camera_pose_head, camera_pose_head.header.frame_id, base_link_pose_head);
                }
            } catch(tf::TransformException ex) {
                ROS_WARN("Failed transform: %s", ex.what());
                base_link_pose = camera_pose;
                base_link_pose_head = camera_pose_head;
            }

            if(face_id) {
                try {
                    if(crop_x >= 0) {
                        cv::Rect roi;
                        roi.x = crop_x;
                        roi.y = crop_y;
                        roi.width = crop_width;
                        roi.height = crop_height;
                        cv::Mat crop = color_image(roi);
                        sensor_msgs::ImagePtr inputImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
                        face_id_srv.request.roi = *inputImage_msg;
                        recognize_face_id_ptr.get()->call(face_id_srv);
                    }
                } catch (cv::Exception e) {
                        std::cout << "Exception in face id! ROI could be wrong!" << e.what();

                }


                if(face_id_srv.response.known) {
                    attributes.attributes.name = face_id_srv.response.name;
                    // Draw person gesture and posture
                    cv::putText(output_image, std::string("N: ") + attributes.attributes.name, cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v+60),
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));

                } else {
                    attributes.attributes.name = "unknown";
                    cv::putText(output_image, std::string("N: ") + attributes.attributes.name, cv::Point(person_list.at(i).Nose.u , person_list.at(i).Nose.v+60),
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255,153,51));
                }
            }

            if(visualize) {
                cv::imshow("CLF OpenPose", output_image);
                cv::waitKey(3);
            }

            sensor_msgs::ImagePtr res_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
            resImgPub.publish(res_img_msg);

            attributes.pose_stamped = base_link_pose;
            attributes.head_pose_stamped = base_link_pose_head;

            res.push_back( attributes );

        }

    }

    return res;
}

double calcAngle(cv::Point p1, cv::Point p2) {
    return std::abs(atan2(p1.y - p2.y, p1.x - p2.x) * 180 / PI);
}

openpose_ros_msgs::PersonAttributesWithPose getPostureAndGesture(openpose_ros_msgs::PersonDetection person) {
    openpose_ros_msgs::PersonAttributesWithPose attributesWithPose;
    openpose_ros_msgs::PersonAttributes attributes;
    openpose_ros_msgs::Gesture gesture;

    attributes.gender_hyp = person.gender_hyp;
    attributes.age_hyp = person.age_hyp;
    attributes.shirtcolor = person.shirtcolor;
    std::vector<openpose_ros_msgs::Gesture> gestures;

    cv::Point LWrist = cv::Point(person.LWrist.u, person.LWrist.v);
    cv::Point LShoulder = cv::Point(person.LShoulder.u, person.LShoulder.v);
    cv::Point LHip = cv::Point(person.LHip.u, person.LHip.v);
    cv::Point LKnee = cv::Point(person.LKnee.u, person.LKnee.v);
    cv::Point LAnkle = cv::Point(person.LAnkle.u, person.LAnkle.v);

    cv::Point RWrist = cv::Point(person.RWrist.u, person.RWrist.v);
    cv::Point RShoulder = cv::Point(person.RShoulder.u, person.RShoulder.v);
    cv::Point RHip = cv::Point(person.RHip.u, person.RHip.v);
    cv::Point RKnee = cv::Point(person.RKnee.u, person.RKnee.v);
    cv::Point RAnkle = cv::Point(person.RAnkle.u, person.RAnkle.v);

    double Vertical = 90;
    double Horizontal = 180;


    double LShoulderLHipDist = sqrt(pow(LShoulder.y - LHip.y , 2));
    double LShoulderLWristAngle = calcAngle(LShoulder,LWrist);
    double LShoulderLHipAngle = calcAngle(LShoulder,LHip);
    double LKneeLHipDist = sqrt(pow(LKnee.y - LHip.y , 2));
    double LAnkleLHipDist = sqrt(pow(LAnkle.y - LHip.y , 2));


    double RShoulderRHipDist = sqrt(pow(RShoulder.y - RHip.y , 2));
    double RShoulderRWristAngle = calcAngle(RShoulder, RWrist);
    double RShoulderRHipAngle = calcAngle(RShoulder, RHip);
    double RKneeRHipDist = sqrt(pow(RKnee.y - RHip.y , 2));
    double RAnkleRHipDist = sqrt(pow(RAnkle.y - RHip.y , 2));

    if( ( ( LKneeLHipDist < (LAnkleLHipDist * SITTINGPERCENT) || RKneeRHipDist < (RAnkleRHipDist * SITTINGPERCENT) )
            && LKneeLHipDist > 0 && RKneeRHipDist > 0 && person.LAnkle.confidence > 0 && person.RAnkle.confidence > 0 ) ||
            ( ( LKneeLHipDist < (LShoulderLHipDist * SITTINGPERCENT ) || RKneeRHipDist < (RShoulderRHipDist * SITTINGPERCENT) )
            && person.LHip.confidence > 0 && person.RHip.confidence > 0 && person.LKnee.confidence > 0 && person.RKnee.confidence > 0
            && person.LShoulder.confidence > 0 && person.RShoulder.confidence > 0) ) {
            attributes.posture.posture = SITTING;
    } else if( ( std::abs(LShoulderLHipAngle - Horizontal) < std::abs(LShoulderLHipAngle - Vertical) ||
              std::abs(RShoulderRHipAngle - Horizontal) < std::abs(RShoulderRHipAngle - Vertical) ||
              LShoulderLHipAngle < 45 || RShoulderRHipAngle < 45 ) && person.LShoulder.confidence > 0
              && person.LHip.confidence > 0 && person.RHip.confidence > 0 && person.LShoulder.confidence > 0 ) {
           attributes.posture.posture = LYING;
    } else {
        attributes.posture.posture = STANDING;
    }


    if ( ( ( 0 <= RShoulderRWristAngle && RShoulderRWristAngle <= 15 ) || ( 165 <= RShoulderRWristAngle && RShoulderRWristAngle <= 180 ) ) &&
                person.RShoulder.confidence >= 0 && person.RWrist.confidence >= 0 ) {
            gesture = openpose_ros_msgs::Gesture();
            gesture.gesture = openpose_ros_msgs::Gesture::POINTING_RIGHT;
            gestures.push_back(gesture);
    }
    if ( ( ( 0 <= LShoulderLWristAngle && LShoulderLWristAngle <= 15 ) || ( 165 <= LShoulderLWristAngle && LShoulderLWristAngle <= 180 ) ) &&
                person.LShoulder.confidence > 0 && person.LWrist.confidence > 0 ) {
            gesture = openpose_ros_msgs::Gesture();
            gesture.gesture = openpose_ros_msgs::Gesture::POINTING_LEFT;
            gestures.push_back(gesture);
    }
    if (person.LElbow.v < person.LShoulder.v && person.LElbow.v > 0 && person.LShoulder.v > 0 && person.LElbow.confidence > 0 && person.LShoulder.confidence > 0) {
            gesture = openpose_ros_msgs::Gesture();
            gesture.gesture = openpose_ros_msgs::Gesture::RAISING_LEFT_ARM;
            gestures.push_back(gesture);
    }
    if (person.RElbow.v < person.RShoulder.v && person.RElbow.v > 0 && person.RShoulder.v > 0 && person.RElbow.confidence > 0 && person.RShoulder.confidence > 0) {
            gesture = openpose_ros_msgs::Gesture();
            gesture.gesture = openpose_ros_msgs::Gesture::RAISING_RIGHT_ARM;
            gestures.push_back(gesture);
    }
    if ( ( (person.LWrist.v < person.LEar.v && person.LWrist.v > 0 && person.LEar.v > 0) ||
                (person.RWrist.v < person.REar.v && person.RWrist.v > 0 && person.REar.v > 0) ) &&
                 person.LWrist.confidence > 0 && person.LEar.confidence > 0 && person.RWrist.confidence > 0 && person.REar.confidence > 0 ) {
        gesture = openpose_ros_msgs::Gesture();
        gesture.gesture = openpose_ros_msgs::Gesture::WAVING;
        gestures.push_back(gesture);
    }
    if (gestures.size() == 0) {
        gesture = openpose_ros_msgs::Gesture();
        gesture.gesture = openpose_ros_msgs::Gesture::NEUTRAL;
        gestures.push_back(gesture);
    }

    attributes.gestures = gestures;
    attributesWithPose.attributes = attributes;
    return attributesWithPose;
}

openpose_ros_msgs::PersonDetection initPersonDetection() {
    openpose_ros_msgs::PersonDetection person;
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

cv::Rect getCrotchRoi( openpose_ros_msgs::PersonDetection person) {

    cv::Rect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = 0;
    roi.height = 0;

    if(person.RShoulder.confidence <= 0 || person.LShoulder.confidence <= 0 ||
            person.RHip.confidence <= 0 || person.LHip.confidence <= 0) {
        ROS_ERROR("Cant create crotch bounding box!");
        return roi;
    }

    if(person.RShoulder.u < person.LShoulder.u) {
        roi.x = person.RShoulder.u;
        roi.y = person.RShoulder.v + std::abs(person.RHip.v - person.RShoulder.v) * 0.6;
        roi.width = std::abs(person.RShoulder.u - person.LShoulder.u);
        roi.height = std::abs(person.RHip.v - roi.y) * 2;
    } else {
        roi.x = person.LShoulder.u;
        roi.y = person.LShoulder.v + std::abs(person.LHip.v - person.LShoulder.v) * 0.6;
        roi.width = std::abs(person.LShoulder.u - person.RShoulder.u);
        roi.height = std::abs(person.LHip.v - roi.y) * 2;
    }

    if(roi.x = roi.width > 640) {
        roi.width = 640 - roi.x;
    }

    if(roi.y = roi.height > 480) {
        roi.height = 480 - roi.y;
    }

    return roi;
}

cv::Rect getUpperBodyRoi( openpose_ros_msgs::PersonDetection person, cv::Mat image ) {
    int cropx, cropy, cropwidth, cropheight = 1;
    cv::Rect roi;
    if (person.RShoulder.confidence > 0 && person.LShoulder.confidence > 0) {
        cropy = person.RShoulder.v;
            cropwidth = std::abs(person.LShoulder.u - person.RShoulder.u);
        if ((person.RShoulder.u - person.LShoulder.u) < 0) {
            cropx = person.RShoulder.u;
        } else {
            cropx = person.LShoulder.u;
        }

        if (person.RHip.confidence > 0)	{
            cropheight = (person.RHip.v - cropy);
        } else {
            if (person.LHip.confidence > 0)	{
                cropheight = (person.LHip.v - cropy);
            } else {
                printf("no hip found\n");
                cropheight = cropwidth;
            }

        }
    } else {
        if (person.RHip.confidence > 0 && person.LHip.confidence > 0)	{
            if ((person.RShoulder.confidence > 0)^(person.LShoulder.confidence > 0)) {
                cropwidth = std::abs(person.LHip.u - person.RHip.u);
                cropy = person.LShoulder.v + person.RShoulder.v;
                if ((person.RHip.u - person.LHip.u) < 0) {
                    cropx = person.RHip.u;
                    cropheight = person.RHip.v - person.LShoulder.u - person.RShoulder.u; // one of the shoulders values will be 0.
                } else {
                    cropx = person.LHip.u;
                    cropheight = person.LHip.v - person.LShoulder.u - person.RShoulder.u;
                }
            } else {
                cropwidth = std::abs(person.LHip.u - person.RHip.u);
                if ((person.RHip.u - person.LHip.u) < 0) {
                    cropx = person.RHip.u;
                    cropheight = cropwidth;
                    cropy = person.RHip.v - cropheight;
                } else {
                    cropx = person.LHip.u;
                    cropheight = cropwidth;
                    cropy = person.LHip.v - cropheight;
                }
            }

        } else {
            if (person.RHip.confidence > 0)	{
                if (person.RShoulder.confidence > 0)	{
                    cropx = person.RShoulder.u;
                    cropy = person.RShoulder.v;
                    cropheight = std::abs(person.RShoulder.v - person.RHip.v);
                    cropwidth = cropheight * 0.5;
                }
                if (person.LShoulder.confidence > 0)	{
                    cropx = person.RHip.u;
                    cropy = person.LShoulder.v;
                    cropheight = std::abs(person.LShoulder.v - person.RHip.v);
                    cropwidth = std::abs(person.LShoulder.u - person.RHip.u);
                }
            }

            if (person.LHip.confidence > 0)	{
                if (person.RShoulder.confidence > 0)	{
                    cropx = person.RShoulder.u;
                    cropy = person.RShoulder.v;
                    cropheight = std::abs(person.RShoulder.v - person.LHip.v);
                    cropwidth = cropheight * 0.5;
                }
                if (person.LShoulder.confidence > 0)	{
                    cropy = person.LShoulder.v;
                    cropheight = std::abs(person.LShoulder.v - person.LHip.v);
                    cropwidth = cropheight*0.5;
                    cropx = person.LShoulder.u - cropwidth;
                }
            } else {
                printf ("No BB possible: RShoulder.confidence %f, LShoulder.confidence %f, RHip.confidence %f, LHip.confidence %f \n", person.RShoulder.confidence, person.LShoulder.confidence, person.RHip.confidence, person.LHip.confidence);
                roi.x = 0;
                roi.y = 0;
                roi.width = 0;
                roi.height = 0;
                return roi;
            }

        }
    }

    if (cropx + cropwidth >= image.size().width) { cropwidth = cropwidth - std::abs((cropx + cropwidth) - image.size().width); }
    if (cropy + cropheight >= image.size().height) { cropheight = cropheight - std::abs((cropy + cropheight) - image.size().height); }

    if ((cropwidth <= 0) || (cropheight <= 0) || (cropx <= 0) || (cropy <= 0))	{
        printf("width or height <= 0");
        cropx = cropy = cropwidth = cropheight = 0;
    }

    roi.x = cropx;
    roi.y = cropy;
    roi.width = cropwidth;
    roi.height = cropheight;

    return roi;
}

std::string getPixelColorType(cv::Scalar hsv_val)
{
    double H = hsv_val[0];
    double S = hsv_val[1];
    double V = hsv_val[2];

    std::string color;
    if (V < 75)
        color = "black";
    else if (V > 190 && S < 27)
        color = "white";
    else if (S < 60 && V < 190)
        color = "grey";
    else {    // Is a color
        if (H < 14)
            color = "red";
        else if (H < 25)
            color = "orange";
        else if (H < 34)
            color = "yellow";
        else if (H < 73)
            color = "green";
        else if (H < 102)
            color = "cyan";
        else if (H < 127)
            color = "blue";
        else if (H < 149)
            color = "purple";
        else    // full circle
            color = "red";    // back to Red
    }
    return color;
}

std::string getShirtColor(openpose_ros_msgs::PersonDetection person, cv::Mat image)	{
	printf ("getShirtColor() \n");
    cv::Rect roi = getUpperBodyRoi(person, image);

    if (roi.x <= 0 || roi.y <= 0 || roi.width <= 0 || roi.height <= 0 )
        return "NO_BOUNDING_BOX";

    if (roi.width < 10 )
        roi.width = 10;
    if ( roi.height < 10 )
        roi.height = 10;

    cv::Mat crop_img = image;
	ROS_DEBUG("#2: x: %d, y: %d, width: %d, height: %d.\n", roi.x, roi.y, roi.width, roi.height);
    try {
        crop_img = image(roi);
    } catch (cv::Exception e) {
        std::cout << "Exception in Shirt color! ROI could be wrong!" << std::endl;
        return "NO_BOUNDING_BOX";
    }

    if(visualize) {
        cv::imshow("CLF OpenPose | SHIRT", crop_img);
        cv::waitKey(3);
    }

    cv::Mat hsv_crop_img;
    cv::cvtColor(crop_img, hsv_crop_img, CV_BGR2HSV);

     std::map<std::string, int> bin_colors;
     bin_colors["white"] = 0;
     bin_colors["black"] = 0;
     bin_colors["grey"] = 0;
     bin_colors["red"] = 0;
     bin_colors["orange"] = 0;
     bin_colors["yellow"] = 0;
     bin_colors["green"] = 0;
     bin_colors["cyan"] = 0;
     bin_colors["blue"] = 0;
     bin_colors["purple"] = 0;

     cv::Mat mask;

     int GRID_SIZE = floor(crop_img.cols/10);

     for ( int y = 0; y <  crop_img.rows - GRID_SIZE; y += GRID_SIZE ) {
         for ( int x = 0; x < crop_img.cols - GRID_SIZE; x += GRID_SIZE ) {
             mask = cv::Mat::zeros(crop_img.size(), CV_8UC1);
             cv::Rect grid_rect( x, y, GRID_SIZE, GRID_SIZE );
             cv::rectangle( mask, grid_rect, 255, CV_FILLED );     
             bin_colors[getPixelColorType(cv::mean(hsv_crop_img,mask))]++;
         }
     }

    std::string result_color = "no color";
    int max_bin_count = 0;
    for (std::map<std::string,int>::iterator it = bin_colors.begin(); it != bin_colors.end(); ++it) {
        if(max_bin_count < it->second) {
            result_color = it->first;
            max_bin_count = it->second;
        }
    }

    return result_color;
}

void getHeadBounds(openpose_ros_msgs::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image){
    int amount = ceil(person.Nose.confidence) + ceil(person.REar.confidence) + ceil(person.REye.confidence) + ceil(person.LEar.confidence) + ceil(person.LEye.confidence);
    float vf = person.Nose.v + person.REar.v + person.REye.v + person.LEar.v + person.LEye.v;
    int v = floor(vf / amount);

    if (amount <= 1)    {
        x = y = width = height = -1;
        return;
    }

    std::list<int> distlist_x;
    if (person.Nose.u != 0) {distlist_x.push_back(std::abs(person.Nose.u));}
    if (person.REar.u != 0) {distlist_x.push_back(std::abs(person.REar.u));}
    if (person.REye.u != 0) {distlist_x.push_back(std::abs(person.REye.u));}
    if (person.LEar.u != 0) {distlist_x.push_back(std::abs(person.LEar.u));}
    if (person.LEye.u != 0) {distlist_x.push_back(std::abs(person.LEye.u));}
    int max_dist_u = *std::max_element(distlist_x.begin(),distlist_x.end());
    int min_dist_u = *std::min_element(distlist_x.begin(),distlist_x.end());

    x = min_dist_u;
    width = max_dist_u - min_dist_u;
    height = width * 1.5;
    y = v - height/2;

    if(y < 0) {
        height += y;
        y = 0;
    }
    if (image.size().width <= (x+width)) {
        width -= (x+width - image.size().width);
    }
    if (image.size().height <= (y+height)) {
        height -= (y+height - image.size().height);
    }
    return;
}

openpose_ros_msgs::BodyPartDetection initBodyPartDetection() {
    openpose_ros_msgs::BodyPartDetection bodypart;
    bodypart.pos.x = 0.0;
    bodypart.pos.y = 0.0;
    bodypart.pos.z = 0.0;
    bodypart.u = 0;
    bodypart.v = 0;
    bodypart.confidence = 0;
    return bodypart;
}

void imagesCb(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::CameraInfoConstPtr &info_depth_msg) {
    
    cv::Mat color_image;
    cv::Mat depth_image;

    try {
        color_image = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
        depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge failed to convert sensor msg: %s", e.what());
        return;
    }
    
    image_mutex_crowd.lock();
    input_image_rgb_crowd = color_image;
    input_image_depth_crowd = depth_image;
    input_image_depth_frame_id = info_depth_msg->header.frame_id;
    if (visualize) {
        cv::imshow("CLF OpenPose | Crowd", input_image_rgb_crowd);
        cv::resizeWindow("CLF OpenPose | Crowd", 320, 240);
        cv::waitKey(3);
    }
    image_mutex_crowd.unlock();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect_people_server");
    ros::NodeHandle localNH("~");

    image_transport::ImageTransport it(localNH);
    resImgPub = it.advertise("/open_pose/result", 1);

    std::string crowdAttServTopic = "/open_pose/get_crowd_attributes";
    localNH.param("crowd_attribute_service_topic", crowdAttServTopic, crowdAttServTopic);

    std::string learn_face_topic = "/open_pose/learn_face";
    localNH.param("learn_face_service_topic", learn_face_topic, learn_face_topic);

    std::string shirt_roi_service_topic = "/open_pose/shirt_roi";
    localNH.param("shirt_roi_service_topic", shirt_roi_service_topic, shirt_roi_service_topic);

    std::string image_rgb_topic = "/camera/rgb/image_raw";
    localNH.param("image_rgb_topic", image_rgb_topic, image_rgb_topic);

    std::string image_depth_topic = "/camera/depth/image_raw";
    localNH.param("image_depth_topic", image_depth_topic, image_depth_topic);

    std::string camera_depth_info_topic = "/camera/depth/camera_info";
    localNH.param("camera_depth_info_topic", camera_depth_info_topic, camera_depth_info_topic);

    ros::NodeHandle n;

    listener = new tf::TransformListener();

    int netInputSizeWidth;
    localNH.param("net_input_size_width", netInputSizeWidth, 320);
    int netInputSizeHeight;
    localNH.param("net_input_size_height", netInputSizeHeight, 160);
    net_input_size = op::Point<int>(netInputSizeWidth, netInputSizeHeight);

    int netOutputSizeWidth;
    localNH.param("net_output_size_width", netOutputSizeWidth, 320);
    int netOutputSizeHeight;
    localNH.param("net_output_size_height", netOutputSizeHeight, 160);
    net_output_size = op::Point<int>(netOutputSizeWidth, netOutputSizeHeight);

    int outputSizeWidth;
    localNH.param("output_size_width", outputSizeWidth, 640);
    int outputSizeHeight;
    localNH.param("output_size_height", outputSizeHeight, 480);
    output_size = op::Point<int>(outputSizeWidth, outputSizeHeight);

    localNH.param("scale_number", scale_number, 1);
    localNH.param("scale_gap", scale_gap, 0.3);

    pose_model = op::PoseModel::COCO_18;

    std::string package_path = ros::package::getPath("openpose_ros");
    models_folder = package_path + "/../openpose/models/";
    localNH.param("models_folder", models_folder, models_folder);

    localNH.param("gpu_id", gpu_id, 0);

    coco_body_parts = op::getPoseBodyPartMapping(pose_model);

    localNH.param("visualize", visualize, true);

    message_filters::Subscriber<sensor_msgs::Image> image_color_sub(n, image_rgb_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n, image_depth_topic, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_depth_sub(n, camera_depth_info_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ImgSyncPolicy;
	message_filters::Synchronizer<ImgSyncPolicy> sync(ImgSyncPolicy(10), image_color_sub, image_depth_sub, camera_depth_sub);

    sync.registerCallback(boost::bind(&imagesCb, _1, _2, _3));  

    ros::ServiceServer serviceCrowd = n.advertiseService(crowdAttServTopic, getCrowdAttributesCb);
    ros::ServiceServer serviceLearn = n.advertiseService(learn_face_topic, learnFaceCb);
    ros::ServiceServer serviceShirtRoi = n.advertiseService(shirt_roi_service_topic, shirtRoiCb);

    // age and gender detection service
    if(ros::service::exists("clf_gender_age_classify_array", false)) {
        ROS_INFO(">> Gender and age classify service exists.");
        gender_age = true;
        gender_age_ptr.reset(new ros::ServiceClient(n.serviceClient<gender_and_age_msgs::GenderAndAgeService>("clf_gender_age_classify_array")));
    }

    // Recognize and learn face service
    if(ros::service::exists("clf_face_identification_know_image", false)) {
        face_id = true;
        ROS_INFO(">> face identification service exists.");
        recognize_face_id_ptr.reset(new ros::ServiceClient(n.serviceClient<clf_perception_vision_msgs::DoIKnowThatPersonImage>("clf_face_identification_know_image")));
        learn_face_id_ptr.reset(new ros::ServiceClient(n.serviceClient<clf_perception_vision_msgs::LearnPersonImage>("clf_face_identification_learn_image")));
    }

    initializeOP();
    ROS_INFO(">> Init done. Ready.");
    ros::spin();

    return 0;
}
