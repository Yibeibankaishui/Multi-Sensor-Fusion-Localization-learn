// TODO
#include <fstream>
#include <memory>
#include <boost/filesystem.hpp>

#include "imu_intergration/subscriber/odom_subscriber.hpp"
#include "imu_integration/evaluator/activity.hpp"

namespace imu_integration {

namespace evaluator {

Activity::Activity(void) : private_nh_("~"), 
                           dir_path("") {       // TODO : add path

    private_nh_.param("pose/topic_name/ground_truth", config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation", config_.topic_name.estimation, std::string("/pose/estimation"));

    sub_eva = private_nh_.subscribe(config_.topic_name.estimation, 1000000, &Activity::EvaluationCallback, this);
    sub_gt = private_nh_.subscribe(config_.topic_name.ground_truth, 1000000, &Activity::GTCallback, this);

    CreateFiles(dir_path);

}  

void Activity::EvaluationCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static double timestamp_eva_init = msg -> header.stamp.toSec();

    eva_writter_ << msg -> header.stamp.toSec() - timestamp_eva_init << ' '
                 << 
}                        

void Activity::GTCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static double timestamp_gt_init = msg -> header.stamp.toSec();

    gt_writer_ << msg -> header.stamp.toSec() - timestamp_gt_init << ' '
               << 
}

bool Activity::CreateFiles(const std::string &dir_path) {

}

}
}

