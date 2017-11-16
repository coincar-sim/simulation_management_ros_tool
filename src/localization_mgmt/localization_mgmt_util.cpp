#include "localization_mgmt_util.hpp"

namespace localization_mgmt_util {

geometry_msgs::Point interpolatePosition(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1, const double scale) {
	checkScaleForInterpolation(scale);
	geometry_msgs::Point interpolPosition;

	interpolPosition.x = p0.x + scale * (p1.x - p0.x);
	interpolPosition.y = p0.y + scale * (p1.y - p0.y);
	interpolPosition.z = p0.z + scale * (p1.z - p0.z);

	return interpolPosition;
}

geometry_msgs::Quaternion interpolateOrientation(
		const geometry_msgs::Quaternion& o0,
		const geometry_msgs::Quaternion& o1, const double scale) {
	checkScaleForInterpolation(scale);
	geometry_msgs::Quaternion interpolOrientation;

	tf2::Quaternion tfQuat0, tfQuat1, tfQuatInterpol;
	tf2::fromMsg(o0, tfQuat0);
	tf2::fromMsg(o1, tfQuat1);
	tfQuatInterpol = tfQuat0.slerp(tfQuat1, scale).normalized();
	interpolOrientation = tf2::toMsg(tfQuatInterpol);

	return interpolOrientation;
}

geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& p0,
		const geometry_msgs::Pose& p1, const double scale) {
	checkScaleForInterpolation(scale);
	geometry_msgs::Pose interpolPose;

	interpolPose.position = interpolatePosition(p0.position, p1.position,
			scale);
	interpolPose.orientation = interpolateOrientation(p0.orientation,
			p1.orientation, scale);

	return interpolPose;
}

geometry_msgs::Pose calculatePose(const geometry_msgs::Pose& startPose,
		const geometry_msgs::Pose& deltaPose) {
	Eigen::Affine3d startPoseEigen, deltaPoseEigen, newPoseEigen;
	geometry_msgs::Pose newPoseGeom;

	tf::poseMsgToEigen(startPose, startPoseEigen);
	tf::poseMsgToEigen(deltaPose, deltaPoseEigen);

	// add deltaPose to startPose
	newPoseEigen = startPoseEigen * deltaPoseEigen;
	tf::poseEigenToMsg(newPoseEigen, newPoseGeom);

	// normalize orientation quaternion
	geometry_msgs::Quaternion newQuatGeom = newPoseGeom.orientation;
	tf2::Quaternion newQuatTf2;
	tf2::fromMsg(newQuatGeom, newQuatTf2);
	newQuatTf2.normalize();
	newPoseGeom.orientation = tf2::toMsg(newQuatTf2);

	return newPoseGeom;
}

std::tuple<size_t, double> getInterpolationIndexAndScale(
		const simulation_only_msgs::DeltaTrajectoryWithID& dtwid,
		const uint64_t startTimeDeltaTrajectory,
		const ros::Time& interpolationTimestamp) {
	double scale = -1;
	size_t index = 0;

	uint64_t dtCurrent = interpolationTimestamp.toNSec()
			- startTimeDeltaTrajectory;

	uint64_t dtFirst =
			dtwid.delta_poses_with_delta_time.front().delta_time.toNSec();

	uint64_t dtLast =
			dtwid.delta_poses_with_delta_time.back().delta_time.toNSec();

	if (dtCurrent < dtFirst) {
		throw std::out_of_range(
				"interpolationTimestamp out of range: smaller than startTimeDeltaTrajectory");
	}

	if (dtCurrent > dtLast) {
		throw std::out_of_range(
				"interpolationTimestamp out of range: larger than startTimeDeltaTrajectory+dtLast");
	}

	for (size_t i = 0; i < dtwid.delta_poses_with_delta_time.size() - 1; i++) {
		uint64_t dt0 = dtwid.delta_poses_with_delta_time[i].delta_time.toNSec();
		uint64_t dt1 =
				dtwid.delta_poses_with_delta_time[i + 1].delta_time.toNSec();

		if (dt0 <= dtCurrent && dtCurrent <= dt1) {
			scale = static_cast<double>(dtCurrent - dt0)
					/ static_cast<double>(dt1 - dt0);
			index = i;
			break;
		}
	}

	return std::tuple<size_t, double>(index, scale);
}

geometry_msgs::Transform transformFromPose(const geometry_msgs::Pose& pose) {
	geometry_msgs::Transform transform;

	transform.translation.x = pose.position.x;
	transform.translation.y = pose.position.y;
	transform.translation.z = pose.position.z;
	transform.rotation.x = pose.orientation.x;
	transform.rotation.y = pose.orientation.y;
	transform.rotation.z = pose.orientation.z;
	transform.rotation.w = pose.orientation.w;

	return transform;
}

automated_driving_msgs::MotionState newMotionStatePoseOnly() {
	automated_driving_msgs::MotionState ms;
	ms.pose.covariance = covGroundTruth();
	ms.twist.covariance = covUnknown();
	ms.accel.covariance = covUnknown();
	return ms;
}

boost::array<double, 36> covGroundTruth() {
	boost::array<double, 36> cov;
	cov.fill(0.0);
	return cov;
}

boost::array<double, 36> covUnknown() {
	boost::array<double, 36> cov;
	cov.fill(0.0);
	cov[0] = -1.;
	cov[7] = -1.;
	cov[14] = -1.;
	cov[21] = -1.;
	cov[28] = -1.;
	cov[35] = -1.;
	return cov;
}

ros::Time rosTimeFromNsec(const uint64_t nsec) {
	ros::Time timestamp;
	timestamp.fromNSec(nsec);
	return timestamp;
}

void checkScaleForInterpolation(const double scale) {
	if (scale < 0 || scale > 1) {
		throw std::invalid_argument(
				"scale not in [0,1] --> no interpolation possible");
	}
}

bool deltaTrajectoryContainsNANs(const simulation_only_msgs::DeltaTrajectoryWithID& dtwid){
    for (size_t i = 0; i < dtwid.delta_poses_with_delta_time.size() - 1; i++) {
        const automated_driving_msgs::DeltaPoseWithDeltaTime& dpwdt = dtwid.delta_poses_with_delta_time[i];
        if (std::isnan(dpwdt.delta_pose.position.x) || std::isnan(dpwdt.delta_pose.position.y) || std::isnan(dpwdt.delta_pose.position.z)){
            return true;
        }
        if (std::isnan(dpwdt.delta_pose.orientation.x) || std::isnan(dpwdt.delta_pose.orientation.y) || std::isnan(dpwdt.delta_pose.orientation.z) || std::isnan(dpwdt.delta_pose.orientation.w)){
            return true;
        }
        if (std::isnan(dpwdt.delta_time.toSec())){
            return true;
        }
    }
    return false;
}

} // namespace localization_mgmt_util
