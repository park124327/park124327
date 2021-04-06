### Hi there 👋

<!--
**park124327/park124327** is a ✨ _special_ ✨ repository because its `README.md` (this file) appears on your GitHub profile.

import numpy as np
from matplotlib import pyplot as plt
import math

######################1번############################
###################### Text 파일 불러오기 ##############################################
f = open("D:\\대학\Lidar박소현.txt",'r')

text_str = f.read()
split_text = text_str.split() #unit.(mm)

#print(split_text)

####################### 좌표계 변환(Spherical to Cartesian) #############################
modified_angle = []
cartesian = np.zeros((1, 2))
empty = np.zeros((1,2))
count = 0

for i in range(0,270):
    if float(split_text[i])> 500:
        if count ==0:
            cartesian[0, 0] = (float(split_text[i]) * math.cos(math.radians(i - 45))) / 1000
            cartesian[0, 1] = (float(split_text[i]) * math.sin(math.radians(i - 45))) / 1000
            count+=1
        else:
            empty[0, 0] = (float(split_text[i]) * math.cos(math.radians(i - 45))) / 1000
            empty[0, 1] = (float(split_text[i]) * math.sin(math.radians(i - 45))) / 1000
            cartesian = np.concatenate((cartesian, empty))
#print(cartesian)

######################## 유클리드 거리 계산 #############################

def euclidean_dist(x1, y1, x2, y2):
    dist = math.sqrt(math.pow((x1-x2) ,2) + math.pow((y1-y2),2))
    return dist

def range_from_origin(x,y):
    dist = r = math.sqrt((x*x)+(y*y))
    return dist

######################## ROI 설정 ######################################
def ROI_theta(x, y):
    r = 0
    theta = 0

    r = math.sqrt((x*x)+(y*y))
    theta = math.atan2(y, x) * (180/math.pi)
    return theta

# for i in range(len(cartesian)):
#     if((ROI_theta(cartesian[i,0], cartesian[i,1]) < 40 or ROI_theta(cartesian[i,0], cartesian[i,1] > 110))):
#         cartesian[i,0] = 0
#         cartesian[i,1] = 0

######################## 유클리디언 클러스터링 ##########################
data_array = np.zeros_like(cartesian, dtype = float)

index_size = len(data_array)
index_array = np.zeros((index_size,1), dtype = int)

threshold_r = 6.6

for i in range(len(cartesian)):
    data_array[i,0] = cartesian[i,0]
    data_array[i,1] = cartesian[i,1]

#print(data_array)

for j in range(len(data_array)-1):
    if(euclidean_dist(data_array[j,0], data_array[j,1], data_array[j+1,0], data_array[j+1,1]) < threshold_r and index_array[j+1] == 0):
        index_array[j+1] = index_array[j]
    else:
        index_array[j+1] = index_array[j]+1

data_index_array = np.concatenate((data_array, index_array), axis=1)

#print(data_index_array)

####################### 가장 큰 클러스터 번호 추출 ##########################
max_cluster_number = np.ndarray.flatten(index_array)

result_number = np.bincount(max_cluster_number).argmax()

print("가장 큰 클러스터 : ", result_number)

###################### 가장 가까운 클러스터 ################################
data_sum = np.zeros_like(index_array, dtype=float)

for i in range(len(index_array)):
    data_sum[i] = range_from_origin(data_index_array[i,0], data_index_array[i,1])

data_index_array = np.concatenate((data_index_array, data_sum), axis=1)

print(data_index_array)

a = min(data_index_array[:,3])
b = 0

for i in range(len(data_index_array)):
    if(a == data_index_array[i,3]):
        b = data_index_array[i,2]
print("가장 가까운 클러스터 번호 :" ,int(b))

####################### 클러스터 크기 확인 ##################################
count_cluster = len(np.unique(index_array))

count_number = 0


#print(index_array)
for i in range(count_cluster):
    for j in range(len(index_array)):
        if(i == index_array[j]):
            count_number = count_number + 1
    print("%d번째 클러스터 사이즈 : %d"%(i,count_number))
    count_number = 0

######################### 데이터 Plot ######################################
plt.scatter(data_index_array[:,0], data_index_array[:,1], c = data_index_array[:,2], s = 20)
plt.title('threshold_r=%f' %threshold_r)
plt.show()


















########################2번##############################
#include "multi_lidar_calibrator.h"

#define MAX_CALIBRATE_COUNT 100 // *** 100번이후로 칼리 안함.

using namespace std;  // *** 편의상 (번호 매기려고) 추가함.
static int calibrate_count = 0; // *** 0번부터 셀거임, class ex) my option으로 하고 launch파일과 연동


void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = parent_frame_;
  in_publisher.publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
  pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);

  pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);

  pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
  pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);

  parent_frame_ = in_parent_cloud_msg->header.frame_id;
  child_frame_ = in_child_cloud_msg->header.frame_id;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointT, PointT> ndt; // *** ndt 객체 선언을 if문 밖에서 해야하므로 뺐다.
  pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>); // *** 같은이유로 if문에서 빼냄
  cout << calibrate_count << endl;  // *** 이건 몇번째 칼리인지 알기위해


  
  if (calibrate_count < MAX_CALIBRATE_COUNT){  // *** 칼리를 (MAX_CALIBRATE_COUNT)번만 하기위해 if문에 넣어놓는다.

	  DownsampleCloud(in_child_cloud, child_filtered_cloud, voxel_size_);



	  ndt.setTransformationEpsilon(ndt_epsilon_);
	  ndt.setStepSize(ndt_step_size_);
	  ndt.setResolution(ndt_resolution_);

	  ndt.setMaximumIterations(ndt_iterations_);

	  ndt.setInputSource(child_filtered_cloud);
	  ndt.setInputTarget(in_parent_cloud);



	  Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	  Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	  Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	  Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());

	  Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

          
	  if(current_guess_ == Eigen::Matrix4f::Identity())
	  {
	    current_guess_ = init_guess_;
	  }
          

	  ndt.align(*output_cloud, current_guess_);
	  
	  // 그냥 출력하는 부분임
	  std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
		    << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

	  std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

	  // Transforming unfiltered, input cloud using found transform.
	  // in_child_cloud에 ndt.getFinalTransformation을 곱해서 output_cloud로 Transform한다.
  	pcl::transformPointCloud (*in_child_cloud, *output_cloud, ndt.getFinalTransformation());

  	current_guess_ = ndt.getFinalTransformation();  // *** 이번 루프에서 곱한 변환값을 current_guess_에 저장한다.

	  //++calibrate_count;  // *** calibration 횟수 (+1) 이건 else안의 if문 때문에 밖에다 뺐음 (155번째줄근처)

          
    if(calibrate_count > 50)   // *** 불안정한 데이터인 50개의 초기데이터를 버리고, 그 다음부터의 Matrix들을 비어있는 averaged_matrix에 더해 넣는다.
    {
		averaged_matrix += current_guess_;
    } 

  }
  // *** 정해놓은 조건동안 calibration을 하고 나서는, if문을 빠져나와 더이상 calibration을 하지 않는다.

  // *** calibration을 끝내고 나서는, averaged_matrix를 더한 횟수만큼 나눈다. 그리고 averaged_matrix를 current_guess_에 다시 넣는다.
  
  else
  {
	  if (calibrate_count == 100) // *** 100번째 칼리때,
    {
 	    averaged_matrix /= 49; // *** 49개의 matrix값들을 더해놓은 averaged_matrix를 49로 나눠서 평균을 만든다.
      current_guess_ = averaged_matrix; // *** 다시 current_guess_에 평균이 된 averaged_matrix를 넣는다.
	  }
  
  // 그 다음 최종 결론값인 current_guess를 in_child_cloud에 곱해서 output_cloud를 만드는 곳임.
  pcl::transformPointCloud (*in_child_cloud, *output_cloud, current_guess_);
  }

  
  Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
  Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
  std::cout << "This transformation can be replicated using:" << std::endl;
  std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
            << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
            << " /" << child_frame_ << " 10" << std::endl;

  std::cout << "Corresponding transformation matrix:" << std::endl
            << std::endl << current_guess_ << std::endl << std::endl;

  *output_cloud += *in_parent_cloud;  // ***** 여기가 두 개의 pointcloud가 합쳐지는 부분!!

  PublishCloud(calibrated_cloud_publisher_, output_cloud);
  // timer end
  //auto end = std::chrono::system_clock::now();
  //auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  //std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

  ++calibrate_count;  // ******** calibration 횟수(+1), 이걸로 칼리 횟수를 Control함. 칼리 멈춰도 횟수는 올라감.
}

/*void ROSMultiLidarCalibratorApp::InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose)
{
  ROS_INFO("[%s] Initial Pose received.", __APP_NAME__);
  tf::Quaternion pose_quaternion(in_initialpose->pose.pose.orientation.x,
                   in_initialpose->pose.pose.orientation.y,
                   in_initialpose->pose.pose.orientation.z,
                   in_initialpose->pose.pose.orientation.w);

  //rotation
  initialpose_quaternion_ = pose_quaternion;

  //translation
  initialpose_position_.setX(in_initialpose->pose.pose.position.x);
  initialpose_position_.setY(in_initialpose->pose.pose.position.y);
  initialpose_position_.setZ(in_initialpose->pose.pose.position.z);


}*/

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
  pcl::VoxelGrid<PointT> voxelized;
  voxelized.setInputCloud(in_cloud_ptr);
  voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  voxelized.filter(*out_cloud_ptr);
}

void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_parent_topic_str, points_child_topic_str;
  std::string initial_pose_topic_str = "/initialpose";
  std::string calibrated_points_topic_str = "/points_calibrated";

  in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
  ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

  in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
  ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

  in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
  ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

  in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
  ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

  in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
  ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

  in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
  ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

  in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
  ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

  in_private_handle.param<double>("x", initial_x_, 0.0);
  in_private_handle.param<double>("y", initial_y_, 0.0);
  in_private_handle.param<double>("z", initial_z_, 0.0);
  in_private_handle.param<double>("roll", initial_roll_, 0.0);
  in_private_handle.param<double>("pitch", initial_pitch_, 0.0);
  in_private_handle.param<double>("yaw", initial_yaw_, 0.0);

  ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
           initial_x_, initial_y_, initial_z_,
           initial_roll_, initial_pitch_, initial_yaw_);

  //generate subscribers and synchronizer
  cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                       points_parent_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

  cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                          points_child_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

  /*initialpose_subscriber_ = node_handle_.subscribe(initial_pose_topic_str, 10,
                                                            &ROSMultiLidarCalibratorApp::InitialPoseCallback, this);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, initial_pose_topic_str.c_str());*/

  calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());

  cloud_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
                                                     *cloud_parent_subscriber_,
                                                     *cloud_child_subscriber_);
  cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

}


void ROSMultiLidarCalibratorApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END",__APP_NAME__);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
  //initialpose_quaternion_ = tf::Quaternion::getIdentity();
  current_guess_ = Eigen::Matrix4f::Identity();
}
