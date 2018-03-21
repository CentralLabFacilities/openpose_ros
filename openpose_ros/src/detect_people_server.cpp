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
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/DetectPeople.h>
#include <openpose_ros_msgs/PersonDetection.h>
#include <openpose_ros_msgs/BodyPartDetection.h>
#include <openpose_ros_msgs/GetPersonAttributes.h>
#include <openpose_ros_msgs/GetCrowdAttributes.h>
#include <openpose_ros_msgs/PersonAttributes.h>
#include <bayes_people_tracker_msgs/PeopleTrackerImage.h>
#include <algorithm>
#include <math.h>
#include <gender_and_age_msgs/GenderAndAgeService.h>

#include <cstdint>
#include <opencv2/core/core.hpp>

#define PI 3.14159265

enum gesture{POINTING_LEFT = 1, POINTING_RIGHT = 2, RAISING_LEFT_ARM = 3, RAISING_RIGHT_ARM = 4, WAVING = 5, NEUTRAL = 6};
enum posture{SITTING = 1, STANDING = 2, LYING = 3};



std::shared_ptr <op::PoseExtractor> pose_extractor;
op::PoseModel pose_model;
op::Point<int> net_input_size;
op::Point<int> net_output_size;
op::Point<int> output_size;
std::map<unsigned int, std::string> coco_body_parts;
int scale_number;
double scale_gap;
cv::Mat input_image;
cv::Mat input_image_crowd;
bayes_people_tracker_msgs::PeopleTrackerImage people_tracker_iamges;
std::mutex person_mutex;
std::mutex image_mutex;
std::mutex image_mutex_crowd;
bool visualize;
bool gender_age = false;
bool shirt_color = true;

double SITTINGPERCENT = 0.2;

boost::shared_ptr<ros::ServiceClient> face_client_ptr;

openpose_ros_msgs::PersonAttributes getPersonAttributes(openpose_ros_msgs::PersonDetection person);

openpose_ros_msgs::PersonAttributes getAttributes(std::string uuid);

openpose_ros_msgs::BodyPartDetection initBodyPartDetection();

openpose_ros_msgs::PersonDetection initPersonDetection();

bool getPersonAttributesCb(openpose_ros_msgs::GetPersonAttributes::Request &req, openpose_ros_msgs::GetPersonAttributes::Response &res);

bool getCrowdAttributesCb(openpose_ros_msgs::GetCrowdAttributes::Request &req, openpose_ros_msgs::GetCrowdAttributes::Response &res);

void getHeadBounds(openpose_ros_msgs::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image);

std::string getShirtColor(openpose_ros_msgs::PersonDetection person, cv::Mat image);

void peopleExtendedCb(const bayes_people_tracker_msgs::PeopleTrackerImage &msg) {
    //liste von people image, people image hat uuid und image
    person_mutex.lock();
    people_tracker_iamges = msg;
    person_mutex.unlock();
}

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_bridge;
    try {
        cv_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge failed to convert sensor msg: %s", e.what());
        return;
    }
    image_mutex_crowd.lock();
    input_image_crowd = cv_bridge->image;
    if (visualize) {
        cv::imshow("openpose_crowd", input_image_crowd);
        cv::waitKey(3);
    }
    image_mutex_crowd.unlock();
}

bool getImageByUuid(std::string id) {
    cv_bridge::CvImagePtr cv_bridge;
    person_mutex.lock();
    for(int i = 0; i< people_tracker_iamges.trackedPeopleImg.size(); i++){
        ROS_INFO("looking at uuid %s", people_tracker_iamges.trackedPeopleImg.at(i).uuid.c_str());
        if (people_tracker_iamges.trackedPeopleImg.at(i).uuid.compare(id) == 0){
            try {
                cv_bridge = cv_bridge::toCvCopy(people_tracker_iamges.trackedPeopleImg.at(i).image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge failed to convert sensor msg: %s", e.what());
                person_mutex.unlock();
                return false;
            }
            input_image = cv_bridge->image;
            if (visualize) {
                cv::imshow("openpose_person", input_image);
                cv::waitKey(3);
            }
            person_mutex.unlock();
            return true;
        }
    }
    input_image = cv::Mat::zeros(30, 30, input_image.type());
    ROS_WARN("NO PERSON WITH UUID %s FOUND!", id.c_str());
    person_mutex.unlock();
    return false;
}

bool getCrowdAttributesCb(openpose_ros_msgs::GetCrowdAttributes::Request &req, openpose_ros_msgs::GetCrowdAttributes::Response &res) {
    image_mutex_crowd.lock();
    op::Array<float> net_input_array;
    std::vector<float> scale_ratios;
    op::CvMatToOpInput cvMatToOpInput{net_input_size, scale_number, (float) scale_gap};
    ROS_INFO("Converting cv image to openpose array.");
    ROS_INFO("Im cols %d, im rows %d", input_image_crowd.cols, input_image_crowd.rows);
    std::tie(net_input_array, scale_ratios) = cvMatToOpInput.format(input_image_crowd);
    ROS_INFO("Detect poses using forward pass.");
    pose_extractor->forwardPass(net_input_array, {input_image_crowd.cols, input_image_crowd.rows}, scale_ratios);
    const auto pose_key_points = pose_extractor->getPoseKeypoints();

    gender_and_age_msgs::GenderAndAgeService srv;
    std::vector<std::string> shirt_list;
    std::vector<openpose_ros_msgs::PersonDetection> person_list;
    ROS_INFO("Extracted %d people.", pose_key_points.getSize(0));

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

            ROS_INFO("u: %d, v: %d", u, v);

            std::string bodypart_name = coco_body_parts[j];

            ROS_INFO("BodyPartName: %s", bodypart_name.c_str());
            ROS_INFO("Confidence: %f", bodypart.confidence);

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

            printf ("Gender and age is ON");
            int crop_x, crop_y, crop_width, crop_height;
            getHeadBounds(person,crop_x, crop_y, crop_width, crop_height, input_image_crowd);

            cv::Rect roi;
            roi.x = crop_x;
            roi.y = crop_y;
            roi.width = crop_width;
            roi.height = crop_height;
            cv::Mat crop = input_image_crowd(roi);
            sensor_msgs::ImagePtr input_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
            srv.request.objects.push_back(*input_image_msg);

        }
        if(shirt_color) {
                printf("Shirt detection is ON \n");
                try{
                        shirt_list.push_back(getShirtColor(person, input_image_crowd));
                } catch (cv::Exception e) {

                        std::cout << "Exception in Shirt color! ROI could be wrong!" << std::endl;
                }
        }

        person_list.push_back(person);
    }
    image_mutex_crowd.unlock();
    if(gender_age) {
        face_client_ptr.get()->call(srv);
        ROS_INFO("gasize: %u",(int)srv.response.gender_and_age_response.gender_and_age_list.size());
        ROS_INFO("personsize: %u",(int)person_list.size());
        for (size_t i = 0; i < person_list.size(); ++i) {
                person_list.at(i).gender_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).gender_probability;
                person_list.at(i).age_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).age_probability;
        }
    }
    if(shirt_color) {
        for (int i = 0; i < shirt_list.size(); i++)	{
                std::string shirtcolor = shirt_list[i];
                ROS_INFO ("color person %d: %s, ", i, shirtcolor.c_str());
                printf("\n");

                person_list.at(i).shirtcolor = shirtcolor;
        }
    }
    for(int i = 0; i < person_list.size(); i++) {
        openpose_ros_msgs::PersonAttributes attributes = getPersonAttributes(person_list.at(i));
        res.attributes.push_back(attributes);
    }
    return true;
}

bool getPersonAttributesCb(openpose_ros_msgs::GetPersonAttributes::Request &req, openpose_ros_msgs::GetPersonAttributes::Response &res) {
    image_mutex.lock();
    ROS_INFO("Called GetAttributes service.");
    res.attributes = getAttributes(req.personId);
    image_mutex.unlock();
//    if(res.attributes.posture == 0 && res.attributes.gesture = 0)
//        return false;
    return true;
}

openpose_ros_msgs::PersonAttributes getAttributes(std::string uuid) {
    if(getImageByUuid(uuid)) {
        op::Array<float> net_input_array;
        std::vector<float> scale_ratios;
        op::CvMatToOpInput cvMatToOpInput{net_input_size, scale_number, (float) scale_gap};
        ROS_INFO("Converting cv image to openpose array.");
        ROS_INFO("Im cols %d, im rows %d", input_image.cols, input_image.rows);
        std::tie(net_input_array, scale_ratios) = cvMatToOpInput.format(input_image);
        ROS_INFO("Detect poses using forward pass.");
        pose_extractor->forwardPass(net_input_array, {input_image.cols, input_image.rows}, scale_ratios);
        const auto pose_keypoints = pose_extractor->getPoseKeypoints();

        gender_and_age_msgs::GenderAndAgeService srv;

        std::vector<std::string> shirt_list;
        std::vector<openpose_ros_msgs::PersonDetection> person_list;
        ROS_INFO("Extracted %d people.", pose_keypoints.getSize(0));

        double best_confidence = 0;
        double confidence = 0;
        int best_confidence_index = 0;

        for (size_t i = 0; i < pose_keypoints.getSize(0); ++i) {
            openpose_ros_msgs::PersonDetection person = initPersonDetection();
            for (size_t j = 0; j < pose_keypoints.getSize(1); ++j) {
                size_t bodypart_id = 3 * (i * pose_keypoints.getSize(1) + j);
                openpose_ros_msgs::BodyPartDetection bodypart = initBodyPartDetection();
                bodypart.confidence = pose_keypoints[bodypart_id + 2];
                confidence += bodypart.confidence;
                int u = pose_keypoints[bodypart_id];
                int v = pose_keypoints[bodypart_id + 1];

                bodypart.u = u;
                bodypart.v = v;

                ROS_INFO("u: %d, v: %d", u, v);

                std::string bodypart_name = coco_body_parts[j];

                ROS_INFO("BodyPartName: %s", bodypart_name.c_str());
                ROS_INFO("Confidence: %f", bodypart.confidence);

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
                confidence = confidence / pose_keypoints.getSize(1);
                if(confidence > best_confidence) {
                    best_confidence = confidence;
                    best_confidence_index = i;
                }
                confidence = 0;
            }


            if(gender_age) {

                printf ("Gender and age is ON");
                int crop_x, crop_y, crop_width, crop_height;
                getHeadBounds(person,crop_x, crop_y, crop_width, crop_height, input_image);

                cv::Rect roi;
                roi.x = crop_x;
                roi.y = crop_y;
                roi.width = crop_width;
                roi.height = crop_height;
                cv::Mat crop = input_image(roi);
                sensor_msgs::ImagePtr inputImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
                srv.request.objects.push_back(*inputImage_msg);

            }
            if(shirt_color) {
                    printf("Shirt detection is ON \n");
                    try{
                            shirt_list.push_back(getShirtColor(person, input_image));
                    } catch (cv::Exception e) {

                            std::cout << "Exception in Shirt color! ROI could be wrong!" << std::endl;
                    }
            }

            person_list.push_back(person);
        }



        if(gender_age) {
            face_client_ptr.get()->call(srv);
            ROS_INFO("gasize: %u",(int)srv.response.gender_and_age_response.gender_and_age_list.size());
            ROS_INFO("personsize: %u",(int)person_list.size());
            for (size_t i = 0; i < person_list.size(); ++i) {
                    person_list.at(i).gender_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).gender_probability;
                    person_list.at(i).age_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).age_probability;


            }
        }
        if(shirt_color) {
            for (int i = 0; i < shirt_list.size(); i++)	{
                    std::string shirtcolor = shirt_list[i];
                    ROS_INFO ("color person %d: %s, ", i, shirtcolor.c_str());
                    printf("\n");

                    person_list.at(i).shirtcolor = shirtcolor;
            }
        }
        return getPersonAttributes(person_list.at(best_confidence_index));
    }
    openpose_ros_msgs::PersonAttributes attributes;
    return attributes;
}

double calcAngle(cv::Point p1, cv::Point p2) {
    return std::abs(atan2(p1.y - p2.y, p1.x - p2.x) * 180 / PI);
}

openpose_ros_msgs::PersonAttributes getPersonAttributes(openpose_ros_msgs::PersonDetection person) {
    openpose_ros_msgs::PersonAttributes attributes;
    attributes.gender_hyp = person.gender_hyp;
    attributes.age_hyp = person.age_hyp;
    attributes.shirtcolor = person.shirtcolor;

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

    double Vertical = calcAngle(cv::Point(50, 10), cv::Point(50, 50));
    double Horizontal = calcAngle(cv::Point(10, 50), cv::Point(50, 50));

    double LWristLShoulderAngle = calcAngle(LWrist, LShoulder);
    double LHipLShoulderAngle = calcAngle(LHip, LShoulder);
    double LHipLAnkleAngle = calcAngle(LHip,LAnkle);
    double LAnkleLShoulderAngle = calcAngle(LAnkle, LShoulder);
    double LKneeLHipDist = sqrt(pow(LKnee.y - LHip.y , 2));
    double LAnkleLHipDist = sqrt(pow(LAnkle.y - LHip.y , 2));

    double RWristRShoulderAngle = calcAngle(RWrist, RShoulder);
    double RHipRShoulderAngle = calcAngle(RHip, RShoulder);
    double RHipRAnkleAngle = calcAngle(RHip, RAnkle);
    double RAnkleRShoulderAngle = calcAngle(RAnkle, RShoulder);
    double RKneeRHipDist = sqrt(pow(RKnee.y - RHip.y , 2));
    double RAnkleRHipDist = sqrt(pow(RAnkle.y - RHip.y , 2));


    std::cout << "LHipLAnkleAngle: " << LHipLAnkleAngle << std::endl;
    std::cout << "RHipRAnkleAngle: " << RHipRAnkleAngle << std::endl;
    std::cout << "Vertical: " << Vertical << std::endl;
    std::cout << "Horizontal: " << Horizontal << std::endl;
    std::cout << "LKneeLHipDist: " << LKneeLHipDist << std::endl;
    std::cout << "LAnkleLHipDist * SittingPercent: " << LAnkleLHipDist * SITTINGPERCENT << std::endl;
    std::cout << "RKneeRHipDist: " << RKneeRHipDist << std::endl;
    std::cout << "RAnkleRHipDist * sittingPercent: " << RAnkleRHipDist * SITTINGPERCENT << std::endl;


    if(std::abs(LHipLAnkleAngle - Vertical) < std::abs(LHipLAnkleAngle - Horizontal) ||
            std::abs(RHipRAnkleAngle - Vertical) < std::abs(RHipRAnkleAngle - Horizontal)) {
        attributes.posture = LYING;
    } else if(LKneeLHipDist < (LAnkleLHipDist * SITTINGPERCENT) && RKneeRHipDist < (RAnkleRHipDist * SITTINGPERCENT) ) {
        attributes.posture = SITTING;
    } else {
        attributes.posture = STANDING;
    }

    if(person.RElbow.v < person.RShoulder.v && person.RElbow.v > 0 && person.RShoulder.v > 0) {
            attributes.gesture = RAISING_RIGHT_ARM;
    } else if (person.LElbow.v < person.LShoulder.v && person.LElbow.v > 0 && person.LShoulder.v > 0) {
        attributes.gesture = RAISING_LEFT_ARM;
    } else if ((person.LWrist.v < person.LShoulder.v && person.LWrist.v > 0 && person.LShoulder.v > 0) ||
               (person.RWrist.v < person.RShoulder.v && person.RElbow.v > 0 && person.RShoulder.v > 0)) {
        attributes.gesture = WAVING;
    } else {
        attributes.gesture = NEUTRAL;
}

    return attributes;
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


std::string getShirtColor(openpose_ros_msgs::PersonDetection person, cv::Mat image)	{
	printf ("getShirtColor() \n");
	cv::Rect roi;

	int cropx, cropy, cropwidth, cropheight = 1;


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
			}
			
		}
	}


    printf ("#1: person.RShoulder.u: %d, person.RShoulder.v: %d. \n", person.RShoulder.u, person.RShoulder.v);
	printf ("#1: x: %d, y: %d, width: %d, height: %d. \n", cropx, cropy, cropwidth, cropheight);


	if (cropx + cropwidth >= image.size().width) { cropwidth = cropwidth - std::abs((cropx + cropwidth) - image.size().width); }
	if (cropy + cropheight >= image.size().height) { cropheight = cropheight - std::abs((cropy + cropheight) - image.size().height); }

	if ((cropwidth <= 0) || (cropheight <= 0) || (cropx <= 0) || (cropy <= 0))	{
		printf("width or height <= 0");
		cropx = cropy = cropwidth = cropheight = 1;
	}

	roi.x = cropx;
	roi.y = cropy;
	roi.width = cropwidth;
	roi.height = cropheight;

	ROS_INFO("#2: x: %d, y: %d, width: %d, height: %d.\n", roi.x, roi.y, roi.width, roi.height);
    cv::Mat crop_img = image(roi); // todo: no hips or shoulders


    //------HSV------
    cv::Mat hsv_crop_img;
    cv::cvtColor(crop_img, hsv_crop_img, CV_BGR2HSV);
//    cv::imshow("shirtColor", crop_img);
//    cv::waitKey(0);
    cv::Scalar mean_color = cv::mean(hsv_crop_img); // [0] h, [1] s, [2] v

    std::cout << std::endl << std::endl <<  "HSV VALUES: " << mean_color[0] << ":" << mean_color[1] << ":" << mean_color[2] << std::endl << std::endl;


    if ( mean_color[2] > 250 )
        return "white";
    if ( mean_color[2] < 60 )
        return "black";
    if ( mean_color[1] <  60 )
        return "grey";
    if( mean_color[0] < 8  || mean_color[0] >= 170 )
        return "red";
    if( 8 <= mean_color[0]  && mean_color[0] < 19 )
        return "orange";
    if( 19 <= mean_color[0]  && mean_color[0] < 35)
        return "yellow";
    if( 35 <= mean_color[0] && mean_color[0] < 80 )
        return "green";
    if( 80 <=  mean_color[0] && mean_color[0] < 90 )
        return "cyan";
    if( 90 <= mean_color[0]  && mean_color[0] < 130 )
        return "blue";
    if( 130 <= mean_color[0]  && mean_color[0] < 170 )
        return "purple";

//	switch(dominant_color_index) {
//    		case 0 : return "red";
//    		case 1 : return "orange";
//            case 2 : return "yellow";
//            case 3 : return "green";
//            case 4 : return "blue";
//            case 5 : return "purple";
//            case 6 : return "black";
//            case 7 : return "white";
//            case 8 : return "grey";
//	}

	return "no color";
}



void getHeadBounds(openpose_ros_msgs::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image){
    printf ("getHeadBounds()");
    int amount = ceil(person.Nose.confidence) + ceil(person.REar.confidence) + ceil(person.REye.confidence) + ceil(person.LEar.confidence) + ceil(person.LEye.confidence);
    float uf = person.Nose.u + person.REar.u + person.REye.u + person.LEar.u + person.LEye.u;
    float vf = person.Nose.v + person.REar.v + person.REye.v + person.LEar.v + person.LEye.v;

    if (amount <= 1)    {
        x = y = width = height = 1;
        return;
    }
    int u = floor(uf / amount);
    int v = floor(vf / amount);
    printf ("u: %d", u);
    printf ("v: %d", v);
    // u and v are now the center of the head.

    std::list<int> distlist_x;
    std::list<int> distlist_y;
    if (person.Nose.u != 0) {distlist_x.push_back(std::abs(person.Nose.u - u));}
    if (person.REar.u != 0) {distlist_x.push_back(std::abs(person.REar.u - u));}
    if (person.REye.u != 0) {distlist_x.push_back(std::abs(person.REye.u - u));}
    if (person.LEar.u != 0) {distlist_x.push_back(std::abs(person.LEar.u - u));}
    if (person.LEye.u != 0) {distlist_x.push_back(std::abs(person.LEye.u - u));}
    int max_dist_u = *std::max_element(distlist_x.begin(),distlist_x.end());
    printf ("max_dist_u: %d", max_dist_u);
    x = std::max(u - max_dist_u, 0);
    y = std::max(v - (int)ceil(max_dist_u*1.5), 0);
    printf ("x: %d", x);
    printf ("y: %d", y);
    width = max_dist_u*2;
    height = max_dist_u*3;
    printf ("width: %d", width);
    printf ("height: %d", height);

    if (image.size().width <= (x+width)) {
        width -= (x+width - image.size().width);
    }
    if (image.size().height <= (y+height)) {
        height -= (y+height - image.size().height);
    }
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

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect_people_server");

    ros::NodeHandle localNH("~");

    //read in params

    std::string extendedPeopleTopic = "/people_tracker/people/extended";
    localNH.param("extended_people_topic", extendedPeopleTopic, extendedPeopleTopic);
    std::string personAttServTopic = "/open_pose/get_person_attributes";
    localNH.param("person_attribute_service_topic", personAttServTopic, personAttServTopic);
    std::string crowdAttServTopic = "/open_pose/get_crowd_attributes";
    localNH.param("crowd_attribute_service_topic", crowdAttServTopic, crowdAttServTopic);
    std::string imageTopic = "/pepper_robot/sink/front/image_raw";
    localNH.param("image_topic", imageTopic, imageTopic);

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

    std::string modelsFolder;
    localNH.param("models_folder", modelsFolder, std::string("/home/nao/ros_distro/share/openpose/models/"));

    int gpuId;
    localNH.param("gpu_id", gpuId, 0);

    coco_body_parts = op::POSE_COCO_BODY_PARTS;

    localNH.param("visualize", visualize, true);

    //init openpose
    pose_extractor = std::shared_ptr<op::PoseExtractorCaffe>(new op::PoseExtractorCaffe(net_input_size, net_output_size,
                                                                                       output_size, scale_number,
                                                                                       pose_model, modelsFolder, gpuId));
    pose_extractor->initializationOnThread();

    ros::NodeHandle n;
    //advertise service to get detected people poses
    ros::ServiceServer servicePerson = n.advertiseService(personAttServTopic, getPersonAttributesCb);

    ros::ServiceServer serviceCrowd = n.advertiseService(crowdAttServTopic, getCrowdAttributesCb);

    //subscriber to recieve extended person message
    ros::Subscriber extendedPeopleSub = n.subscribe(extendedPeopleTopic, 100, peopleExtendedCb);

    ros::Subscriber imageSub = n.subscribe(imageTopic, 100, imageCb);

    ROS_INFO("Init done. Can start detecting people.");
    ros::spin();


    return 0;
}
