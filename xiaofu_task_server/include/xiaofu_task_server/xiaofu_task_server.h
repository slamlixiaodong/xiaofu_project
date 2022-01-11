
#ifndef __xiaofu_task_server_H_
#define __xiaofu_task_server_H_
// 头文件
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>  
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <cstdio>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>

#include "xiaofu_task_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <xiaofu_task_server/xiaofu_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <xiaofu_task_server/sensor_pub.h>
#include "std_msgs/Int64.h"
#include "std_srvs/Empty.h"
#include "tf/tf.h"
#include "sensor_msgs/Range.h"
#include "csignal"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/console.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace std;
#define FRAME_HEADER      0X55  //帧头，和下位机一致
#define FRAME_TAIL  0X0A //帧尾

#define RECEIVE_SENSOR_DATA_SIZE    21//接收传感器板
#define RECEIVE_BATTERY_DATA_SIZE    18//接收电源板
#define SET_SEND_SENSOR_LED_DATA_SIZE			16//向下位机发送
#define SENSOR_CALLBACK_DATA_SIZE  4  //传感器led控制反馈
#define SET_SEND_BATTERY_DATA_SIZE 5 //电源板发送字节

#define LED_ACK_HEADER        0xAA

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

typedef uint8_t u8;
typedef uint16_t u16;

  /*
 0x01 关闭灯光 0x02 打开灯光 0x03 快速快闪 0x04 慢速闪烁 0x05 正常闪烁 0x06 正常呼吸 0x07 快速呼吸 0x08 慢速呼吸 0x09 暂无
  */                         //帧头          左臂           右臂           左腿            右腿           校验位
uint8_t left_mode[] = {FRAME_HEADER,0x01,0x04,0x01,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x01,0x01,0x01,0x00,0x00};
uint8_t right_mode[] = {FRAME_HEADER,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x00,0x00};
uint8_t forward_mode[] = {FRAME_HEADER,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x00,0x00};
uint8_t charge_mode[] = {FRAME_HEADER,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x01,0x01,0x01,0x04,0x01,0x01,0x00,0x00};
#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

//向传感器发送的结构体
typedef struct _SEND_SENSOR_DATA_  
{
	  uint8_t tx[SET_SEND_SENSOR_LED_DATA_SIZE];       
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_SENSOR_DATA;

//向电源板发送的结构体
typedef struct _SEND_BATTERY_DATA_  
{
	  uint8_t tx[SET_SEND_BATTERY_DATA_SIZE];       
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_BATTERY_DATA;

//传感器板反馈的结构体
typedef struct _RECEIVE_SENSOR_CALLBACK_DATA_     
{
	  uint8_t rx[SENSOR_CALLBACK_DATA_SIZE];
		unsigned char Frame_Header; //1个字节 帧头
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_SENSOR_CALLBACK_DATA;

//传感器数据的结构体
typedef struct _SENSOR_DATA_     
{
		uint8_t rx[RECEIVE_SENSOR_DATA_SIZE];
	  uint8_t Flag_Stop;
		unsigned char Frame_Header; //1个字节 帧头
		float Wave[5];     //5个超声波  单位 cm
		u8 Switch;      //光电开关
		float Temp;        //温度传感器 C
		float Humidity;    //湿度传感器  %
		float PM_25;       //PM25传感器值 mg/m^3
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}SENSOR_DATA;

//电源的结构体
typedef struct _BATTERY_DATA_     
{
	  uint8_t rx[RECEIVE_BATTERY_DATA_SIZE];
    uint8_t Flag_Stop;
		unsigned char Frame_Header; //1个字节 帧头
		float Voltage;  //电池电压
		float Current;   //电池电流
		float Quantity ; //电池电量
		float Capacity ; //电池容量
		int RSOC ; //电池剩余电量百分比
		int Ctrl ; //FET控制状态
		int Protect ;//电池保护状态
		int Cycle ; //循环次数
		float CRC;
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}BATTERY_DATA;

typedef RECEIVE_SENSOR_CALLBACK_DATA (*p_cb)(bool,void*);
typedef string (*xf_cb)(string,string,void*);
class xiaofu_robot
{
    public:
        xiaofu_robot();
        ~xiaofu_robot();
        void thread_fun();
        void open_battery(string control);
        static RECEIVE_SENSOR_CALLBACK_DATA leg_callback(bool recieve_success,void* arg);
        void get_leg_callback(bool recieve_success,void* arg,p_cb p);
        static string xiaofu_task_exec(string cmd_control,string _context,void* arg);
        void xiaofu_task_exec_cb(string cmd_control,string _context_,xf_cb p);
    private:
        serial::Serial sensor_Serial,battery_Serial;
        SENSOR_DATA Get_Sensor_Data();//获取传感器数据
        BATTERY_DATA Get_Battery_Data();//获取电源板数据
        static void signalHandler(int signum);//ros shutdown函数
        // std::vector<move_base_msgs::MoveBaseGoal> read_target_from_file();//从文件导入目标点
        map<string,move_base_msgs::MoveBaseGoal> read_target_from_file();//从文件导入目标点
        // void target_publish(int single_target); //目标点发布函数
        void target_publish(string tmp_position_name);
        void save_target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg); //记录目标点到vector回调函数
        void save_target_to_file();//保存目标点到文件
        void turn_left_and_right(string cmd,float goal_angle,float angle_speed,double rate);//左转右转处理函数
        void turn_left_and_right(float goal_angle);  //六麦板角度处理函数
        void forward_and_back(string cmd,float goal_distance,float linear_speed,double rate);//前进后退处理函数
      	void Robot_Led_control(string light_control);  //腿灯控制函数
        void androidCallback(const std_msgs::String::ConstPtr& msg); //接收任务话题
        bool xiaofu_task_server(xiaofu_task_server::xiaofu_server::Request &req, xiaofu_task_server::xiaofu_server::Response &res);
        void mutil_target_publish();
        static void* xiaofu_vel_cmd_pub(void *arg);   //线程判断电池状态，发出控制命令
        static void* xiaofu_sensor(void *arg);
        float shorttofloat(uint8_t Data_High,uint8_t Data_Low);//short转float
        u16 MODBUS_CRC(u8 *buff,u8 len);//CRC校验调用
        u8 Check_CRC(u8 *data, int size);//CRC校验函数
        
        int serial_baud_rate;//波特率
        string sensor_usart_port_name,battery_usart_port_name,target_file_path,map_file_path;//串口名字映射
        ros::NodeHandle n;//创建句柄
        ros::Subscriber sub_task,sub_target_pose; //定义速度订阅者和灯光订阅者
        ros::Publisher battery_pub,cmd_vel_publisher,batter_cmd_vel_publisher,sensor_pub,wave0_pub,wave1_pub,task_pub;//定义发布者
        ros::ServiceClient lidar_start_client,lidar_stop_client;
        ros::ServiceServer xiaofu_task; //定义一个服务
        ros::Time _Now, _Last_Time;//时间相关
        float Sampling_Time; //采样时间
        std_srvs::Empty srv;
        RECEIVE_SENSOR_CALLBACK_DATA receive_sensor_led_data;//腿灯反馈数据
        SEND_SENSOR_DATA Send_Sensor_Led_Data;  //发送控制腿灯的结构体   
        SEND_BATTERY_DATA Send_battery_control; //发送控制电源的结构体
        SENSOR_DATA temp_sensor_data; //传感器临时数据定义
        BATTERY_DATA battery_data;//电源数据
        map<string,move_base_msgs::MoveBaseGoal> position_map;
        vector<geometry_msgs::PoseStamped> target_pose; //目标点设置
        int sensor_error_count=0;   //控制时传感器错误计数
        bool send_success,navi_flag,charge_flag; //回调函数标志位
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac{"move_base",true};
        map<string,move_base_msgs::MoveBaseGoal> move_goal;
        string tmp_cmd_control,context,android_res;
        // std::vector<move_base_msgs::MoveBaseGoal> goal;//要发布的目标点数据
};


class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);


      std::string mapdatafile =  mapname_ + ".pgm";
      std::string map_pgm_path = "/home/xiaofu04/" + mapdatafile;
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      // FILE* out = fopen(mapdatafile.c_str(), "w");
      FILE* out = fopen(map_pgm_path.c_str(), "w");
      ROS_INFO("%s",map_pgm_path.c_str());
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile =  mapname_ + ".yaml";
      std::string map_yaml_path = "/home/xiaofu04/" + mapmetadatafile;
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      // FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
      FILE* yaml = fopen(map_yaml_path.c_str(), "w");

      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};


class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated_ = (res != 0);
      if (!deprecated_) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            exit(-1);
          }
        } catch (YAML::Exception &) {
          ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }

          boost::filesystem::path mapfpath(mapfname);
          if (!mapfpath.is_absolute())
          {
            boost::filesystem::path dir(fname);
            dir = dir.parent_path();
            mapfpath = dir / mapfpath;
            mapfname = mapfpath.string();
          }
        } catch (YAML::InvalidScalar &) {
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      try
      {
          map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
          exit(-1);
      }
      // To make sure get a consistent time in simulation
      ROS_DEBUG("Waiting for valid time (make sure use_sime_time is false or a clock server (e.g., gazebo) is running)");
      ros::Time::waitForValid();
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_DEBUG("Got time");
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      get_map_service_ = nh_.advertiseService("static_map", &MapServer::mapCallback, this);
      //pub = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

      // Latched publisher for metadata
      metadata_pub_= nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      metadata_pub_.publish( meta_data_message_ );

      // Latched publisher for data
      map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      map_pub_.publish( map_resp_.map );
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher metadata_pub_;
    ros::ServiceServer get_map_service_;
    bool deprecated_;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

#endif
