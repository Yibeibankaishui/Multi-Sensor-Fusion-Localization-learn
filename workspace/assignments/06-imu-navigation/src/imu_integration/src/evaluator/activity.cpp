// TODO
#include <fstream>
#include <memory>
#include <boost/filesystem.hpp>

#include "imu_intergration/subscriber/odom_subscriber.hpp"
#include "imu_integration/evaluator/activity.hpp"

namespace imu_integration {

namespace evaluator {

Activity::Activity(void) : private_nh_("~"), 
                           dir_path("~/Multi-Sensor-Fusion-Localization-learn/workspace/assignments/06-imu-navigation/src/imu_intergration/trajectory") {       // TODO : add path

    private_nh_.param("pose/topic_name/ground_truth", config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation", config_.topic_name.estimation, std::string("/pose/estimation"));

    sub_eva = private_nh_.subscribe(config_.topic_name.estimation, 1000000, &Activity::EvaluationCallback, this);
    sub_gt = private_nh_.subscribe(config_.topic_name.ground_truth, 1000000, &Activity::GTCallback, this);

    CreateFiles(dir_path);

}  

void Activity::EvaluationCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static double timestamp_eva_init = msg -> header.stamp.toSec();

    eva_writter_ << msg -> header.stamp.toSec() - timestamp_eva_init << ' '
                 << msg -> pose.pose.position.x << ' '
                 << msg -> pose.pose.position.y << ' '
                 << msg -> pose.pose.position.z << ' '
                 << msg -> pose.pose.orientation.x << ' '
                 << msg -> pose.pose.orientation.y << ' '
                 << msg -> pose.pose.orientation.z << ' '
                 << msg -> pose.pose.orientation.w << '\n';
}                        

void Activity::GTCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static double timestamp_gt_init = msg -> header.stamp.toSec();

    gt_writer_ << msg -> header.stamp.toSec() - timestamp_gt_init << ' '
                << msg -> pose.pose.position.x << ' '
                << msg -> pose.pose.position.y << ' '
                << msg -> pose.pose.position.z << ' '
                << msg -> pose.pose.orientation.x << ' '
                << msg -> pose.pose.orientation.y << ' '
                << msg -> pose.pose.orientation.z << ' '
                << msg -> pose.pose.orientation.w << '\n';
}

bool Activity::CreateFiles(const std::string &dir_path) {
    if (!boost::filesystem::is_directory(dir_path)) {
        boost::filesystem::create_directory(dir_path);
    }
    if (!boost::filesystem::is_directory(dir_path)) {
        std::cout << "Couldn't create directory " << dir_path << "\n";
        return false;
    }

    std::string gt_path = dir_path + "/ground_truth.txt";
    std::string eva_path = dir_path + "/evaluation.txt";

    gt_writter_.open(gt_path);
    eva_writter_.open(eva_path);

    return true;

}

}
}

