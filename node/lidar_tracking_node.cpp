#include "ros/ros.h"
#include "lidar_tracking/Tracking.h"
#include "lidar_tracking/TrackingArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <std_msgs/Header.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <time.h>
#include <vector>

#include "coder_array.h"
#include "coder_bounded_array.h"
#include "correction.h"
#include "fusion_object.h"
#include "fusion_track_management.h"
#include "mpower.h"
#include "mtimes.h"
#include "MW_target_hardware_resources.h"
#include "prediction.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "unit_conversion.h"
#include "unit_conversion_data.h"
#include "unit_conversion_initialize.h"
#include "unit_conversion_terminate.h"
#include "unit_conversion_types.h"

using namespace std;

// Global Variables
struct struct0_T LIDAR_DETECTION;
struct struct3_T UNIT_CONVERSION;
struct struct9_T DEFINITION;
struct struct10_T FUSION_TRACK;
struct struct17_T TRACKING;
struct struct27_T ASSOCIATION;
struct struct0_T *LIDAR_DETECTION_ptr;
struct struct3_T *UNIT_CONVERSION_ptr;
struct struct9_T *DEFINITION_ptr;
struct struct10_T *FUSION_TRACK_ptr;
struct struct17_T *TRACKING_ptr;
struct struct27_T *ASSOCIATION_ptr;

double SAMPLE_TIME;
double LIDAR_TRACK_SWITCH;
double FRONT_VISION_TRACK_SWITCH;

double Lidar_Detection_init[256];
double Lidar_Detection[320];
double Fusion_Object[352];
double Association_Map[32];
double Fusion_Track[768];
double P_Fusion_Track[1152];
double Fusion_Track_Predicted[768];
double P_Fusion_Track_Predicted[1152];
double Fusion_Track_Updated[768];
double P_Fusion_Track_Updated[1152];
double Association_Map_Updated[32];

coder::array<double, 2U> Lidar_Detection_out;
coder::array<double, 2U> Fusion_Object_out;
coder::array<double, 2U> Fusion_Track_out;
coder::array<double, 3U> P_Fusion_Track_out;
coder::array<double, 2U> Fusion_Track_Predicted_out;
coder::array<double, 3U> P_Fusion_Track_Predicted_out;
coder::array<double, 3U> P_Fusion_Track_Updated_out;
coder::array<double, 2U> Association_Map_Updated_out;
coder::array<double, 3U> S_CA;
coder::array<double, 2U> Md_FV;

double Association_Map_k_1[32];
double Fusion_Track_k_1[768];
double P_Fusion_Track_k_1[1152];

std_msgs::Header header;


void callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input);
lidar_tracking::TrackingArray result2msg();

void lidar_tracking_process();

void inputInit(const autoware_msgs::DetectedObjectArray::ConstPtr& input);

void coder2vanilla_2U(coder::array<double, 2U> coder, double *vanilla, int size);
void coder2vanilla_3U(coder::array<double, 3U> coder, double *vanilla, int size);

void paramInit();

int main(int argc, char** argv) {
	cout << "[Lidar_Tracking]: Start node" << endl;

	paramInit();

	ros::init(argc, argv, "lidar_tracking_node");
	ros::NodeHandle nh("");

	ros::Publisher pub_tracking = nh.advertise<lidar_tracking::TrackingArray>("/tracking_msg", 1);
	ros::Subscriber sub_object = nh.subscribe("/detection/lidar_objects_test", 1, callback);

	pub_tracking.publish(result2msg());

	ros::spin();

	cout << "[Lidar_Tracking]: End node" << endl;

	return 0;
}

void callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input) {
	cout << "[Lidar_Tracking]: callback test" << endl;

	inputInit(input);

	header = input->header;

	lidar_tracking_process();
}

lidar_tracking::TrackingArray result2msg() {
	lidar_tracking::TrackingArray msg;
	msg.header = header;

	lidar_tracking::Tracking element;

	for (int msg_number = 0; msg_number < 32; msg_number++) {
		element.header = header;
		element.Rel_Pos_X = (float)Fusion_Track[msg_number * 24 + 13];
		element.Rel_Pos_Y = (float)Fusion_Track[msg_number * 24 + 12];
		element.Width = (float)Fusion_Track[msg_number * 24 + 4];
		element.Length = (float)Fusion_Track[msg_number * 24 + 5];
		element.Height = (float)Fusion_Track[msg_number * 24 + 6];
		element.Yaw = (float)Fusion_Track[msg_number * 24 + 7];
		element.ID = (uint8_t)Fusion_Track[msg_number * 24 + 11];
		element.Rel_Vel_X = (float)Fusion_Track[msg_number * 24 + 15];
		element.Rel_Vel_Y = (float)Fusion_Track[msg_number * 24 + 14];
		element.Life_Time = (uint8_t)Fusion_Track[msg_number * 24 + 18];
		element.Class = (uint8_t)Fusion_Track[msg_number * 24 + 8];

		msg.tracks.push_back(element);
	}

	return msg;
}

void lidar_tracking_process() {
	unit_conversion(Lidar_Detection_init, LIDAR_DETECTION_ptr, UNIT_CONVERSION_ptr, DEFINITION_ptr, Lidar_Detection_out);

	coder2vanilla_2U(Lidar_Detection_out, Lidar_Detection, 320);
	fusion_object(Lidar_Detection, FUSION_TRACK_ptr, LIDAR_DETECTION_ptr, Fusion_Object_out);

	prediction(Association_Map_k_1, Fusion_Track_k_1, P_Fusion_Track_k_1, SAMPLE_TIME, FUSION_TRACK_ptr, TRACKING_ptr, Fusion_Track_Predicted_out, P_Fusion_Track_Predicted_out);

	coder2vanilla_2U(Fusion_Object_out, Fusion_Object, 352);
	coder2vanilla_2U(Fusion_Track_Predicted_out, Fusion_Track_Predicted, 768);
	coder2vanilla_3U(P_Fusion_Track_Predicted_out, P_Fusion_Track_Predicted, 1152);
	correction(Fusion_Object, Association_Map_k_1, Fusion_Track_Predicted, P_Fusion_Track_Predicted, FUSION_TRACK_ptr, TRACKING_ptr, ASSOCIATION_ptr, Fusion_Track_Updated, P_Fusion_Track_Updated_out, Association_Map_Updated_out, S_CA, Md_FV);

	coder2vanilla_3U(P_Fusion_Track_Updated_out, P_Fusion_Track_Updated, 768);
	coder2vanilla_2U(Association_Map_Updated_out, Association_Map_Updated, 32);
	fusion_track_management(Fusion_Object, Fusion_Track_Updated, P_Fusion_Track_Updated, Association_Map_Updated, Association_Map_k_1, TRACKING_ptr, DEFINITION_ptr, LIDAR_TRACK_SWITCH, FUSION_TRACK_ptr, ASSOCIATION_ptr, FRONT_VISION_TRACK_SWITCH, Fusion_Track_out, Association_Map, P_Fusion_Track_out);

	coder2vanilla_2U(Fusion_Track_out, Fusion_Track, 768);
	coder2vanilla_3U(P_Fusion_Track_out, P_Fusion_Track, 1152);
	std::copy(Association_Map, Association_Map + 32, Association_Map_k_1);
	std::copy(Fusion_Track, Fusion_Track + 768, Fusion_Track_k_1);
	std::copy(P_Fusion_Track, P_Fusion_Track + 1152, P_Fusion_Track_k_1);
}

void inputInit(const autoware_msgs::DetectedObjectArray::ConstPtr& input) {
	for (int object_number = 0; object_number < 32; object_number++) {
		Lidar_Detection_init[object_number * 32 + 0] = (double)input->objects[object_number].pose.position.x;
		Lidar_Detection_init[object_number * 32 + 1] = (double)input->objects[object_number].pose.position.y;
		Lidar_Detection_init[object_number * 32 + 2] = (double)input->objects[object_number].angle;
		Lidar_Detection_init[object_number * 32 + 3] = (double)input->objects[object_number].dimensions.x;
		Lidar_Detection_init[object_number * 32 + 4] = (double)input->objects[object_number].dimensions.y;
		Lidar_Detection_init[object_number * 32 + 5] = (double)input->objects[object_number].dimensions.z;
		Lidar_Detection_init[object_number * 32 + 6] = (double)input->objects[object_number].indicator_state;
		Lidar_Detection_init[object_number * 32 + 7] = (double)input->objects[object_number].score;
	}
}

void coder2vanilla_2U(coder::array<double, 2U> coder, double *vanilla, int size) {
	for (int index = 0; index < size; index++) {
		vanilla[index] = coder[index];
	}
}

void coder2vanilla_3U(coder::array<double, 3U> coder, double *vanilla, int size) {
	for (int index = 0; index < size; index++) {
		vanilla[index] = coder[index];
	}
}

void paramInit() {
	SAMPLE_TIME = 0.0500;
	LIDAR_TRACK_SWITCH = 1;
	FRONT_VISION_TRACK_SWITCH = 0;

	LIDAR_DETECTION_ptr = &LIDAR_DETECTION;
	UNIT_CONVERSION_ptr = &UNIT_CONVERSION;
	DEFINITION_ptr = &DEFINITION;
	FUSION_TRACK_ptr = &FUSION_TRACK;
	TRACKING_ptr = &TRACKING;
	ASSOCIATION_ptr = &ASSOCIATION;

	LIDAR_DETECTION_ptr->MEASURE.REL_POS_X = 1;
	LIDAR_DETECTION_ptr->MEASURE.REL_POS_Y = 2;
	LIDAR_DETECTION_ptr->MEASURE.YAW = 3;
	LIDAR_DETECTION_ptr->MEASURE.LENGTH = 4;
	LIDAR_DETECTION_ptr->MEASURE.WIDTH = 5;
	LIDAR_DETECTION_ptr->MEASURE.HEIGHT = 6;
	LIDAR_DETECTION_ptr->MEASURE.CLASS = 7;
	LIDAR_DETECTION_ptr->MEASURE.SCORE = 8;
	LIDAR_DETECTION_ptr->MEASURE.STATE_NUMBER = 10;
	LIDAR_DETECTION_ptr->MEASURE.ID_EXIST = 0;

	LIDAR_DETECTION_ptr->TRACKING.REL_POS_X = 9;
	LIDAR_DETECTION_ptr->TRACKING.REL_POS_Y = 10;
	LIDAR_DETECTION_ptr->TRACKING.STATE_NUMBER = 3;

	LIDAR_DETECTION_ptr->TRACK_NUMBER = 32;

	LIDAR_DETECTION_ptr->STATE_NUMBER = 13;

	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.YAW = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.LONG_VEL = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.LAT_VEL = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.YAW_RATE = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.LONG_ACC = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.LAT_ACC = 1;
	UNIT_CONVERSION_ptr->IN_VEHICLE_SENSOR.STEERING_ANGLE = 1;

	UNIT_CONVERSION_ptr->LIDAR.REL_POS_X = 1;
	UNIT_CONVERSION_ptr->LIDAR.REL_POS_Y = 1;
	UNIT_CONVERSION_ptr->LIDAR.HEADING_ANGLE = 1;
	UNIT_CONVERSION_ptr->LIDAR.LENGTH = 1;
	UNIT_CONVERSION_ptr->LIDAR.WIDTH = 1;
	UNIT_CONVERSION_ptr->LIDAR.HEIGHT = 1;
	UNIT_CONVERSION_ptr->LIDAR.ID = 1;
	UNIT_CONVERSION_ptr->LIDAR.LABEL = 1;
	UNIT_CONVERSION_ptr->LIDAR.BEHAVIOR_STATE = 1;
	UNIT_CONVERSION_ptr->LIDAR.REL_VEL_X = 1;
	UNIT_CONVERSION_ptr->LIDAR.REL_VEL_Y = 1;

	UNIT_CONVERSION_ptr->LIDAR_DETECTION.REL_POS_X = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.REL_POS_Y = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.HEADING_ANGLE = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.LENGTH = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.WIDTH = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.HEIGHT = 1;
	UNIT_CONVERSION_ptr->LIDAR_DETECTION.LABEL = 1;

	UNIT_CONVERSION_ptr->FRONT_VISION.REL_POS_X = 1;
	UNIT_CONVERSION_ptr->FRONT_VISION.REL_POS_Y = 1;

	UNIT_CONVERSION_ptr->GT_TRACK.REL_POS_X = 1;
	UNIT_CONVERSION_ptr->GT_TRACK.REL_POS_Y = 1;
	UNIT_CONVERSION_ptr->GT_TRACK.REL_VEL_X = 1;
	UNIT_CONVERSION_ptr->GT_TRACK.REL_VEL_Y = 1;
	UNIT_CONVERSION_ptr->GT_TRACK.HEADING_ANGLE = 1;

	UNIT_CONVERSION_ptr->SWITCH = 1;

	DEFINITION_ptr->ON = 1;
	DEFINITION_ptr->OFF = 0;

	FUSION_TRACK_ptr->MEASURE.REL_POS_Y = 1;
	FUSION_TRACK_ptr->MEASURE.REL_POS_X = 2;
	FUSION_TRACK_ptr->MEASURE.REL_VEL_Y = 3;
	FUSION_TRACK_ptr->MEASURE.REL_VEL_X = 4;
	FUSION_TRACK_ptr->MEASURE.WIDTH = 5;
	FUSION_TRACK_ptr->MEASURE.LENGTH = 6;
	FUSION_TRACK_ptr->MEASURE.HEIGHT = 7;
	FUSION_TRACK_ptr->MEASURE.HEADING_ANGLE = 8;
	FUSION_TRACK_ptr->MEASURE.RANGE = 10;
	FUSION_TRACK_ptr->MEASURE.ANGLE = 11;
	FUSION_TRACK_ptr->MEASURE.CLASS = 9;

	FUSION_TRACK_ptr->ASSOCIATION.FUSED_LDT_ID = 12;

	FUSION_TRACK_ptr->MEASURE_STATE_NUMBER = 12;

	FUSION_TRACK_ptr->TRACKING.REL_POS_Y = 13;
	FUSION_TRACK_ptr->TRACKING.REL_POS_X = 14;
	FUSION_TRACK_ptr->TRACKING.REL_VEL_Y = 15;
	FUSION_TRACK_ptr->TRACKING.REL_VEL_X = 16;
	FUSION_TRACK_ptr->TRACKING.UPDATED_AGE = 17;
	FUSION_TRACK_ptr->TRACKING.COASTING_AGE = 18;
	FUSION_TRACK_ptr->TRACKING.LIFE_TIME = 19;
	FUSION_TRACK_ptr->TRACKING.REL_POS_NORMAL = 20;
	FUSION_TRACK_ptr->TRACKING.REL_VEL_TANGENTIAL = 21;
	FUSION_TRACK_ptr->TRACKING.REL_VEL_NORMAL = 22;
	FUSION_TRACK_ptr->TRACKING.REL_ACC_Y = 23;
	FUSION_TRACK_ptr->TRACKING.REL_ACC_X = 24;

	FUSION_TRACK_ptr->THREAT.P_LONG = 29;
	FUSION_TRACK_ptr->THREAT.TTC = 30;
	FUSION_TRACK_ptr->THREAT.D_BR = 31;
	FUSION_TRACK_ptr->THREAT.D_W = 32;
	FUSION_TRACK_ptr->THREAT.X_P = 33;
	FUSION_TRACK_ptr->THREAT.I_LONG = 34;
	FUSION_TRACK_ptr->THREAT.DLC = 35;
	FUSION_TRACK_ptr->THREAT.TLC = 36;
	FUSION_TRACK_ptr->THREAT.I_LAT = 37;
	FUSION_TRACK_ptr->THREAT.RSS_X = 38;
	FUSION_TRACK_ptr->THREAT.RSS_Y = 39;
	FUSION_TRACK_ptr->THREAT.FCW_HONDA = 40;
	FUSION_TRACK_ptr->THREAT.FCW_MAZDA = 41;

	FUSION_TRACK_ptr->VEHICLE_RECOGNITION.ID = 42;
	FUSION_TRACK_ptr->VEHICLE_RECOGNITION.RECOGNITION = 43;

	FUSION_TRACK_ptr->VEHICLE_MANEUVER.PR_CTRV = 44;
	FUSION_TRACK_ptr->VEHICLE_MANEUVER.MANEUVER = 45;

	FUSION_TRACK_ptr->TRACKING_STATE_NUMBER = 24;

	FUSION_TRACK_ptr->THREAT_STATE_NUMBER = 37;

	FUSION_TRACK_ptr->RECOGNITION_STATE_NUMBER = 39;

	FUSION_TRACK_ptr->MANEUVER_STATE_NUMBER = 41;

	FUSION_TRACK_ptr->STATE_NUMBER = 41;

	FUSION_TRACK_ptr->TRACK_NUMBER = 32;

	TRACKING_ptr->FUSION_TRACK.STATE_NUMBER = 6;
	TRACKING_ptr->FUSION_TRACK.PDAF.PD = 0.9000;
	TRACKING_ptr->FUSION_TRACK.PDAF.PG = 0.9900;
	TRACKING_ptr->FUSION_TRACK.VEHICLE_GATING_THRESHOLD = 9.2000;
	TRACKING_ptr->FUSION_TRACK.MAHALANOBIS_DISTANCE.DEFAULT_VALUE = 300;
	TRACKING_ptr->FUSION_TRACK.X_ACC_VAR_CV = 1;
	TRACKING_ptr->FUSION_TRACK.Y_ACC_VAR_CV = 0.2500;
	TRACKING_ptr->FUSION_TRACK.X_ACC_VAR_CA = 30;
	TRACKING_ptr->FUSION_TRACK.Y_ACC_VAR_CA = 10;

	TRACKING_ptr->MEASURE.STATE_NUMBER = 2;

	std::fill_n(TRACKING_ptr->LIDAR_ASSOCIATION_MAP, 32, 0);

	TRACKING_ptr->LIDAR.X_ACC_VAR_CV = 1;
	TRACKING_ptr->LIDAR.Y_ACC_VAR_CV = 0.2500;
	std::fill_n(TRACKING_ptr->LIDAR.H_CV, 16, 0);
	TRACKING_ptr->LIDAR.H_CV[0] = 1;
	TRACKING_ptr->LIDAR.H_CV[5] = 1;
	std::fill_n(TRACKING_ptr->LIDAR.R_CV, 36, 0);
	TRACKING_ptr->LIDAR.R_CV[0] = 0.0100;
	TRACKING_ptr->LIDAR.R_CV[7] = 0.0100;
	TRACKING_ptr->LIDAR.R_CV[14] = 0.0100;
	TRACKING_ptr->LIDAR.R_CV[21] = 0.0100;
	TRACKING_ptr->LIDAR.R_CV[28] = 0.0100;
	TRACKING_ptr->LIDAR.R_CV[35] = 0.0100;
	TRACKING_ptr->LIDAR.DELETE_TRACK_COUNT = 0;
	std::fill_n(TRACKING_ptr->LIDAR.H_CA, 36, 0);
	TRACKING_ptr->LIDAR.H_CA[0] = 1;
	TRACKING_ptr->LIDAR.H_CA[7] = 1;
	std::fill_n(TRACKING_ptr->LIDAR.R_CA, 36, 0);
	TRACKING_ptr->LIDAR.R_CA[0] = 0.0100;
	TRACKING_ptr->LIDAR.R_CA[7] = 0.0100;
	TRACKING_ptr->LIDAR.R_CA[14] = 0.0100;
	TRACKING_ptr->LIDAR.R_CA[21] = 0.0100;
	TRACKING_ptr->LIDAR.R_CA[28] = 0.0100;
	TRACKING_ptr->LIDAR.R_CA[35] = 0.0100;

	TRACKING_ptr->LIDAR_GATING.INPUT_NUMBER = 2;
	TRACKING_ptr->LIDAR_GATING.Y_MIN = -1.5000;
	TRACKING_ptr->LIDAR_GATING.Y_MAX = 1.5000;
	TRACKING_ptr->LIDAR_GATING.X_MIN = -5;
	TRACKING_ptr->LIDAR_GATING.X_MAX = 5;

	std::fill_n(TRACKING_ptr->ASSOCIATION_MAP, 32, 0);

	TRACKING_ptr->GATING.INPUT_NUMBER = 2;

	TRACKING_ptr->RESIDUAL.DEFAULT_VALUE = 300;
	TRACKING_ptr->RESIDUAL.REL_POS_Y = 1;
	TRACKING_ptr->RESIDUAL.REL_POS_X = 2;
	TRACKING_ptr->RESIDUAL.REL_VEL_Y = 3;
	TRACKING_ptr->RESIDUAL.REL_VEL_X = 4;

	TRACKING_ptr->VEHICLE_GATING.Y_MIN = -2;
	TRACKING_ptr->VEHICLE_GATING.Y_MAX = 2;
	TRACKING_ptr->VEHICLE_GATING.X_MIN = -3;
	TRACKING_ptr->VEHICLE_GATING.X_MAX = 3;
	TRACKING_ptr->VEHICLE_GATING.VY_MIN = -1.5000;
	TRACKING_ptr->VEHICLE_GATING.VY_MAX = 1.5000;
	TRACKING_ptr->VEHICLE_GATING.VX_MIN = -1.5000;
	TRACKING_ptr->VEHICLE_GATING.VX_MAX = 1.5000;

	TRACKING_ptr->THRESHOLD_MAX_LIFE_TIME = 10;

	TRACKING_ptr->DELETE_TRACK_COUNT = 0;

	ASSOCIATION_ptr->SENSOR_NUMBER = 1;

	ASSOCIATION_ptr->MAP.LDT = 1;
	ASSOCIATION_ptr->MAP.FVT = 2;

	ASSOCIATION_ptr->GATING.INPUT_NUMBER = 3;

	ASSOCIATION_ptr->RESIDUAL.DEFAULT_VALUE = 300;
	ASSOCIATION_ptr->RESIDUAL.REL_POS_Y = 1;
	ASSOCIATION_ptr->RESIDUAL.REL_POS_X = 2;
	ASSOCIATION_ptr->RESIDUAL.REL_VEL_X = 3;
	ASSOCIATION_ptr->RESIDUAL.REL_VEL_Y = 4;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI_NUMBER = 3;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.ROI_X_MIN = 0;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.ROI_X_MAX = 40;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.THRESHOLD_Y = 2.7000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.THRESHOLD_X = 7;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.THRESHOLD_VY = 5;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI1.THRESHOLD_VX = 6;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.ROI_X_MIN = 40;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.ROI_X_MAX = 70;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.ROI_X_SLOPE = 0.3333;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.THRESHOLD_Y = 2.7000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.THRESHOLD_X = 7;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.THRESHOLD_VY = 5;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI2.THRESHOLD_VX = 6;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.ROI_X_MIN = 70;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.ROI_X_MAX = 90;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.THRESHOLD_Y = 2.7000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.THRESHOLD_X = 17;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.THRESHOLD_VY = 5;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI3.THRESHOLD_VX = 6;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.ROI_X_MIN = 90;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.ROI_X_MAX = 120;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.THRESHOLD_Y = 2.7000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.THRESHOLD_X = 17;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.THRESHOLD_VY = 5;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.FVT.ROI4.THRESHOLD_VX = 6;
	
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.ROI_NUMBER = 2;

	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.ROI_X_MIN = 0;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.ROI_X_MAX = 20;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_Y = 2;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_X = 6;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_VY = 1.5000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_VX = 1.5000;
	
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.ROI_X_MIN = 20;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.ROI_X_MAX = 120;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.ROI_X_SLOPE = 0.2500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_Y = 3;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_X = 6;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_VY = 2;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_VX = 2;
	
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.ROI_X_MIN = -20;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.ROI_X_MAX = 0;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_Y = 1.3000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_X = 0.5500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_VY = 1.9500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_VX = 0.5500;
	
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.ROI_X_MIN = -120;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.ROI_X_MAX = -20;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.ROI_X_SLOPE = -0.2500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_Y = 1.3000;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_X = 0.5500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_VY = 1.9500;
	ASSOCIATION_ptr->VEHICLE_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_VX = 0.5500;
	
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI_NUMBER = 3;

	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.ROI_X_MIN = 0;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.ROI_X_MAX = 25;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.THRESHOLD_Y = 1;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.THRESHOLD_X = 2.5000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.THRESHOLD_VY = 1;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI1.THRESHOLD_VX = 1.5000;

	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.ROI_X_MIN = 25;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.ROI_X_MAX = 35;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.ROI_Y_SLOPE = 0.1000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.ROI_X_SLOPE = 0.2500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.THRESHOLD_VY = 1.5000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI2.THRESHOLD_VX = 2;

	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.ROI_X_MIN = 35;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.ROI_X_MAX = 120;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.THRESHOLD_Y = 2;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.THRESHOLD_X = 5;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.THRESHOLD_VY = 2;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.FVT.ROI3.THRESHOLD_VX = 2.5000;

	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.ROI_NUMBER = 2;

	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.ROI_X_MIN = 0;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.ROI_X_MAX = 20;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_Y = 2;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_X = 6;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_VY = 1.5000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI1.THRESHOLD_VX = 1.5000;
	
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.ROI_X_MIN = 20;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.ROI_X_MAX = 120;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.ROI_X_SLOPE = 0.2500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_Y = 3;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_X = 6;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_VY = 2;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.FRONT.ROI2.THRESHOLD_VX = 2;
	
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.ROI_X_MIN = -20;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.ROI_X_MAX = 0;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_Y = 1.3000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_X = 0.5500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_VY = 1.9500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI1.THRESHOLD_VX = 0.5500;
	
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.ROI_X_MIN = -120;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.ROI_X_MAX = -20;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.ROI_X_SLOPE = -0.2500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_Y = 1.3000;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_X = 0.5500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_VY = 1.9500;
	ASSOCIATION_ptr->PEDESTRIAN_GATING.LDT.LDT.REAR.ROI2.THRESHOLD_VX = 0.5500;

	std::fill_n(Association_Map_k_1, 32, 0);
	std::fill_n(Fusion_Track_k_1, 768, 0);
	std::fill_n(P_Fusion_Track_k_1, 1152, 0);
}
