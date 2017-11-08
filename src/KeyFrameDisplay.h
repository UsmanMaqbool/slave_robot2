/** @file KeyFrameDisplay.h
 *	@brief KeyFrameDisplay is a file copied from LSD-SLAM and modified
 *	Need struct InputPointDense
 *
 *	@author Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
*/
#pragma once

#undef Success
#include <Eigen/Core>
//#include <Eigen/Core>

//#include "QGLViewer/qglviewer.h"
#include "slave_robot2/keyframeMsg.h"
#include "slave_robot2/keyframeGraphMsg.h"
#include "slave_robot2/keyframeMsgStamped.h"
#include "slave_robot2/keyframeGraphMsgStamped.h"
#include "sophus/sim3.hpp"

#include <sstream>
#include <fstream>

struct InputPointDense
{
	float idepth;
	float idepth_var;
	char color[4];
};

