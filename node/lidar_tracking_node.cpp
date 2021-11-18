#include "ros/ros.h"
#include <iostream>
#include <time.h>

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

// msg
double Lidar_Detection[256];


// preprocessing header

// tracking header

void msgCopy();

void msgCallback() {
	const msg_tmp = msg;
	msgCopy(msg_tmp); // why indirect?

	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "");
	ros::NodeHandle nh;

	ros::Subscriber nh.subscribe("Lidar_detection", 1, msgCallback);
}

// time stmap must be needed?
void msgCopy(const ) {
	lidar_detection_[0] = msg->measure.relative_position_x;
	lidar_detection_[1] = msg->measure.relative_position_y;
	lidar_detection_[2] = msg->measure.yaw;
	lidar_detection_[3] = msg->measure.length;
	lidar_detection_[4] = msg->measure.width;
	lidar_detection_[5] = msg->measure.height;
	lidar_detection_[6] = msg->measure.class;
	lidar_detection_[7] = msg->measure.score;
}
