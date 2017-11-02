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
#include <algorithm>

#include <gender_and_age/GenderAndAgeService.h>

#include <cstdint>
#include <opencv2/core/core.hpp>


std::shared_ptr <op::PoseExtractor> poseExtractor;
op::PoseModel poseModel;
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

bool gender_age = false;
bool shirt_color = true;

boost::shared_ptr<ros::ServiceClient> face_client_ptr;

openpose_ros::BodyPartDetection initBodyPartDetection();

openpose_ros::PersonDetection initPersonDetection();

void getHeadBounds(openpose_ros::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image);

std::string getShirtColor(openpose_ros::PersonDetection person, cv::Mat image);

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvBridge;
    try {
        cvBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge failed to convert sensor msg: %s", e.what());
        return;
    }
    imageMutex.lock();
    inputImage = cvBridge->image;
    if (visualize) {
        cv::imshow("input", inputImage);
        cv::waitKey(3);
    }
    imageMutex.unlock();
}

void pointcloudCb(const sensor_msgs::PointCloud2 pCloud) {
    pointcloudMutex.lock();
    pointcloud = pCloud;
    pointcloudMutex.unlock();
}

bool detectPeopleCb(openpose_ros::DetectPeople::Request &req, openpose_ros::DetectPeople::Response &res) {
    ROS_INFO("Called detect people service.");
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    op::CvMatToOpInput cvMatToOpInput{netInputSize, scaleNumber, (float) scaleGap};
    ROS_INFO("Converting cv image to openpose array.");
    imageMutex.lock();
    pointcloudMutex.lock();
    ROS_INFO("Im cols %d, im rows %d", inputImage.cols, inputImage.rows);
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput.format(inputImage);
    ROS_INFO("Detect poseys using forward pass.");
    poseExtractor->forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
    const auto poseKeypoints = poseExtractor->getPoseKeypoints();

    if (visualize) {
        op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, true, (float) 0.6};
        op::OpOutputToCvMat opOutputToCvMat{outputSize};
        double scaleInputToOutput;
        op::Array<float> outputArray;
        op::CvMatToOpOutput cvMatToOpOutput{outputSize};
        std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(inputImage);
        poseRenderer.renderPose(outputArray, poseKeypoints);
        auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

        cv::imshow("Detections", outputImage);
        cv::waitKey(3);
    }

    gender_and_age::GenderAndAgeService srv;
    // never use list, bitch!!!
    std::vector<std::string> shirt_list;

    ROS_INFO("Extracted %d people.", poseKeypoints.getSize(0));
    for (size_t i = 0; i < poseKeypoints.getSize(0); ++i) {
        openpose_ros::PersonDetection person = initPersonDetection();
        for (size_t j = 0; j < poseKeypoints.getSize(1); ++j) {
            size_t bodypart_id = 3 * (i * poseKeypoints.getSize(1) + j);
            openpose_ros::BodyPartDetection bodypart = initBodyPartDetection();
            bodypart.confidence = poseKeypoints[bodypart_id + 2];

            //credit to Saurav Agarwal (http://sauravag.com/2016/11/how-to-get-depth-xyz-of-a-2d-pixel-from-pointcloud2-or-kinect-data/)
            //for conversion method
            int u = poseKeypoints[bodypart_id];
            int v = poseKeypoints[bodypart_id + 1];

            bodypart.u = u;
            bodypart.v = v;

            ROS_INFO("u: %d, v: %d", u, v);

            int pcWidth = pointcloud.width;
            int pcHeight = pointcloud.height;

            int arrayPosition = v * pointcloud.row_step + u * pointcloud.point_step;
            int arrayPosX = arrayPosition + pointcloud.fields[0].offset; // X has an offset of 0
            int arrayPosY = arrayPosition + pointcloud.fields[1].offset; // Y has an offset of 4
            int arrayPosZ = arrayPosition + pointcloud.fields[2].offset; // Z has an offset of 8

            float X = 0.0;
            float Y = 0.0;
            float Z = 0.0;

            memcpy(&X, &pointcloud.data[arrayPosX], sizeof(float));
            memcpy(&Y, &pointcloud.data[arrayPosY], sizeof(float));
            memcpy(&Z, &pointcloud.data[arrayPosZ], sizeof(float));

            ROS_INFO("X: %f, Y: %f, Z: %f", X, Y, Z);
            tf::Transform baseToCam;
            tf::Transform bodyPartTransform;
            bodyPartTransform.setOrigin(tf::Vector3(X, Y, Z));
            bodyPartTransform.setRotation(tf::Quaternion(0, 0, 0, 1));

            baseToCam.mult(transform.inverse(), bodyPartTransform);

            bodypart.pos.x = baseToCam.getOrigin().x();
            bodypart.pos.y = baseToCam.getOrigin().y();
            bodypart.pos.z = baseToCam.getOrigin().z();

            std::string bodypart_name = cocoBodyParts[j];

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
            getHeadBounds(person,crop_x, crop_y, crop_width, crop_height, inputImage);

            cv::Rect roi;
            roi.x = crop_x;
            roi.y = crop_y;
            roi.width = crop_width;
            roi.height = crop_height;
            cv::Mat crop = inputImage(roi);
            sensor_msgs::ImagePtr inputImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
            srv.request.objects.push_back(*inputImage_msg);

        }
	if(shirt_color) {
		printf("Shirt detection is ON \n");
		try{
			shirt_list.push_back(getShirtColor(person, inputImage));
		} catch (cv::Exception e) {
			
			std::cout << "Exception in Shirt color! ROI could be wrong!" << std::endl;
		} 
	}

	
        res.people_list.push_back(person);
    }
    if(gender_age) {
        face_client_ptr.get()->call(srv);
	//ROS_INFO("gasize: %u",srv.response.gender_and_age_response.gender_and_age_list.size());
    	//ROS_INFO("personsize: %u",res.people_list.size());
    	for (size_t i = 0; i < res.people_list.size(); ++i) {
        	res.people_list.at(i).gender_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).gender_probability;
        	res.people_list.at(i).age_hyp = srv.response.gender_and_age_response.gender_and_age_list.at(i).age_probability;


    	}
    }
    if(shirt_color) {
    	for (int i = 0; i < shirt_list.size(); i++)	{
		std::string shirtcolor = shirt_list[i];
		ROS_INFO ("color person %d: %s, ", i, shirtcolor.c_str());
		printf("\n");
		
		res.people_list.at(i).shirtcolor = shirtcolor;
	}    
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


std::string getShirtColor(openpose_ros::PersonDetection person, cv::Mat image)	{
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


	printf ("#1: person.RShoulder.v: %d, person.RShoulder.v: %d. \n", person.RShoulder.u, person.RShoulder.v);	
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
        cv::Mat crop_img = inputImage(roi); // todo: no hips or shoulders
        //cv::imshow("shirtColor", crop_img);

	//cv::waitKey(0);
	//------HSV------
	
	cv::Mat hsv_crop_img;
	cv::cvtColor(crop_img, hsv_crop_img, CV_BGR2HSV);
	
	std::vector<cv::Mat> hsv_planes;
	cv::split(hsv_crop_img, hsv_planes);

	cv::Mat h = hsv_planes[0]; // H channel
    	cv::Mat s = hsv_planes[1]; // S channel
    	cv::Mat v = hsv_planes[2]; // V channel


	int red, orange, yellow, green, blue, purple, black, white, grey, pixel;
	red = orange = yellow = green = blue = purple = black = white = grey = pixel = 0;

	uchar h_pixel;
	uchar s_pixel;
	uchar v_pixel;
	for(int i=0; i<hsv_crop_img.rows; i++)	{
    		for(int j=0; j<hsv_crop_img.cols; j++)	{
        		h_pixel = h.at<uchar>(i, j);
			s_pixel = s.at<uchar>(i, j);
			v_pixel = v.at<uchar>(i, j);

			pixel++;
			if (s_pixel >= 40)	{
				if (((h_pixel >= 0) && (h_pixel <= 10)) || ((h_pixel <= 180 ) && (h_pixel >=165))) { red++;}
				if ((h_pixel >= 7) && (h_pixel <= 22)) { orange++;  }
				if ((h_pixel >= 15) && (h_pixel <= 45))	 { yellow++; }
				if ((h_pixel >= 30) && (h_pixel <= 83))	 { green++; }
				if ((h_pixel >= 75) && (h_pixel <= 135)) { blue++; }
				if ((h_pixel >= 123) && (h_pixel <= 172)) { purple++; }

			}
			if (v_pixel <= 40)	{ black++;}
			//if (v_pixel >= 60)	{ white++;}
			if ((v_pixel >= 30) && (v_pixel <= 70)) { grey++;}
		}
	}
	int color_array[9] = {red,orange,yellow,green,blue,purple,black,white,grey};	
	
/**
	printf ("pixel: %d \n", pixel);
	printf ("red: %d \n", color_array[0]);
	printf ("orange: %d \n", color_array[1]);
	printf ("yellow: %d \n", color_array[2]);
	printf ("green: %d \n", color_array[3]);
	printf ("blue: %d \n", color_array[4]);
	printf ("purple: %d \n", color_array[5]);
	printf ("black: %d \n", color_array[6]);
	printf ("white: %d \n", color_array[7]);
	printf ("grey: %d \n", color_array[8]); */
	
	int max_color = *std::max_element(color_array,color_array+9);
	int dominant_color_index;
	for(int k = 0; k < 8; k++)	{
		if (max_color == color_array[k])	{dominant_color_index = k;}	
	}
	
	switch(dominant_color_index) {
    		case 0 : return "red";
    		case 1 : return "orange";
		case 2 : return "yellow";
		case 3 : return "green";
		case 4 : return "blue";
		case 5 : return "purple";
		case 6 : return "black";
		case 7 : return "white";
		case 8 : return "grey";
	}

	return "no color";
}



void getHeadBounds(openpose_ros::PersonDetection person, int &x, int &y, int &width, int &height, cv::Mat image){
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

openpose_ros::BodyPartDetection initBodyPartDetection() {
    openpose_ros::BodyPartDetection bodypart;
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
    int netInputSizeWidth;
    localNH.param("net_input_size_width", netInputSizeWidth, 320);
    int netInputSizeHeight;
    localNH.param("net_input_size_height", netInputSizeHeight, 160);
    netInputSize = op::Point<int>(netInputSizeWidth, netInputSizeHeight);

    int netOutputSizeWidth;
    localNH.param("net_output_size_width", netOutputSizeWidth, 320);
    int netOutputSizeHeight;
    localNH.param("net_output_size_height", netOutputSizeHeight, 160);
    netOutputSize = op::Point<int>(netOutputSizeWidth, netOutputSizeHeight);

    int outputSizeWidth;
    localNH.param("output_size_width", outputSizeWidth, 640);
    int outputSizeHeight;
    localNH.param("output_size_height", outputSizeHeight, 480);
    outputSize = op::Point<int>(outputSizeWidth, outputSizeHeight);

    localNH.param("scale_number", scaleNumber, 1);
    localNH.param("scale_gap", scaleGap, 0.3);

    poseModel = op::PoseModel::COCO_18;

    std::string modelsFolder;
    localNH.param("models_folder", modelsFolder, std::string("/vol/robocup/nightly/share/openpose/models/"));

    int gpuId;
    localNH.param("gpu_id", gpuId, 0);

    cocoBodyParts = op::POSE_COCO_BODY_PARTS;

    localNH.param("visualize", visualize, true);

    //init openpose
    poseExtractor = std::shared_ptr<op::PoseExtractorCaffe>(new op::PoseExtractorCaffe(netInputSize, netOutputSize,
                                                                                       outputSize, scaleNumber,
                                                                                       poseModel, modelsFolder, gpuId));
    poseExtractor->initializationOnThread();

    ros::NodeHandle n;
    //advertise service to get detected people poses
    ros::ServiceServer service = n.advertiseService("detect_people", detectPeopleCb);
    //subscriber to recieve rgb image
    ros::Subscriber imageSub = n.subscribe("/xtion/rgb/image_raw", 100, imageCb);
    //subscriber to recieve pointcloud
    ros::Subscriber pointcloudSub = n.subscribe("/xtion/depth/points", 100, pointcloudCb);
    //rosservice for age and gender detection
    if(ros::service::exists("gender_and_age",false)) {
        ROS_INFO("gender and age classify service exists.");
        gender_age = true;
        face_client_ptr.reset(new ros::ServiceClient(n.serviceClient<gender_and_age::GenderAndAgeService>("gender_and_age")));
    }
    

    //create tf listener to transform positions of detected into robot coordinates
    tf::TransformListener listener;

    std::string cameraFrame;
    localNH.param("camera_frame", cameraFrame, std::string("xtionupper_rgb_optical_frame"));
    std::string base_frame;
    localNH.param("base_frame", base_frame, std::string("base_link"));

    try {
        listener.waitForTransform(cameraFrame, base_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(cameraFrame, base_frame, ros::Time(0), transform);
        ROS_INFO("Got TF: %f %f %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    } catch (tf::TransformException ex) {
        ROS_ERROR("Could not get transform from camera frame to base frame: %s", ex.what());
        return 1;
    }

    ROS_INFO("Init done. Can start detecting people.");
    ros::spin();
	
	//ros::MultiThreadedSpinner().spin();

    return 0;
}