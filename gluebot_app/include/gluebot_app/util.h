#ifndef _GLUEBOT_UTIL_H_
#define _GLUEBOT_UTIL_H_

std::vector<Eigen::Affine3d> createGlueTask(Eigen::Affine3d part_frame)
{
    std::vector<Eigen::Affine3d> waypoints;

    Eigen::Affine3d waypoint = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(M_PI + 0.3, Eigen::Vector3d::UnitY());

    // Eigen::Affine3d waypoint;
    // tf::poseEigenToMsg(pose, waypoint);

    waypoint.translation() << 0.0, 0.0, 0.02;
    waypoints.push_back(waypoint);
    int N = 5;
    double step = 0.15 / ((double) N - 1);
    for (int i = 0; i < N; ++i)
    {
        waypoint.translation()[0] += step;
        waypoints.push_back(waypoint);
    }
    for (int i = 0; i < waypoints.size(); ++i)
    {
      waypoints[i] = part_frame * waypoints[i];
    }
    return waypoints;
}

#endif