#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

int main()
{
    PROFILE_FUNCTION;
    // Adjust these variables
    float x = 34233.0f; // Position in x-axis
    float y = 32111.0f; // Position in y-axis
    float z = 0.0f;  // Position in z-axis

    float roll = 0.0;  // Roll (rotation around x-axis)
    float pitch = 0.0; // Pitch (rotation around y-axis)
    float yaw = 0.0;    // Yaw (rotation around z-axis)

    // Convert roll, pitch, yaw to a quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

    // Set the pose with the position and quaternion
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    // Get the transformation matrix from the pose
    Eigen::Affine3f T_w_c_est = roadmap_explorer::getTransformFromPose(pose);
    // std::cout << T_w_c_est.translation() << std::endl;
    // std::cout << "**" << std::endl;
    // std::cout << T_w_c_est.rotation() << std::endl;

    for (float dx = -5; dx <= 5; dx += 0.35)
    {
        for (float dy = -5; dy <= 5; dy += 0.35)
        {
            auto landmark_x = x + dx;
            auto landmark_y = y + dy;
            // Define the point in the world coordinates
            Eigen::Vector3f p3d_w_eig(landmark_x, landmark_y, 0.0f); // Adjust this as needed
            Eigen::Vector3f p3d_c_eig;

            // Define the covariance matrix Q (3x3)
            Eigen::Matrix3f Q = Eigen::Matrix3f::Identity(); // Adjust this as needed

            // Compute the information of the point
            float information = roadmap_explorer::computeInformationOfPointLocal(p3d_c_eig, p3d_w_eig, T_w_c_est, Q);

            // Print the result
            std::cout << information << ", ";
        }
        std::cout << std::endl;
    }

    return 0;
}