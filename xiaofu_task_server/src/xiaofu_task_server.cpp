
#include<xiaofu_task_server/xiaofu_task_server.h>

void xiaofu_robot::signalHandler(int signum)
{
  ROS_INFO("%s is received, Terminating the node...",strsignal(signum));
  ros::shutdown();
  exit(signum);
}

void xiaofu_robot::save_target_to_file()
{
  // ofstream fout(target_file_path,ios::out|ios::binary);
  // if(!fout)
  //     ROS_ERROR("file open failed");
	// fout << std::setprecision(10);
	// for(int i=0; i < target_pose.size();i++)
  //   		fout << target_pose[i].pose.position.x<<" "<< target_pose[i].pose.position.y << " " << tf::getYaw(target_pose[i].pose.orientation) << '\n';
  ofstream fout(target_file_path,ios::out|ios::binary);
  if(!fout)
      ROS_ERROR("file open failed");
	fout << std::setprecision(10);
  // position_map.find("chufang");
	for(const auto& map_iterator : position_map)
    		fout << map_iterator.first<<" " << map_iterator.second.target_pose.pose.position.x<< " "<< map_iterator.second.target_pose.pose.position.y << " "<< tf::getYaw(map_iterator.second.target_pose.pose.orientation) << "\n";
  ROS_INFO("Save target pose successful!!!");
}

void xiaofu_robot::save_target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // geometry_msgs::PoseStamped tmp_target;
    // tmp_target.header = msg->header;
    // tmp_target.pose = msg->pose;
    // target_pose.push_back(tmp_target);
    move_base_msgs::MoveBaseGoal tmp_target;
    string position;
    tmp_target.target_pose.header = msg->header;
    tmp_target.target_pose.pose = msg->pose;
    cout<<"input position name:";
    cin>>position;
    position_map.insert(pair<string,move_base_msgs::MoveBaseGoal>(position,tmp_target));
}

// std::vector<move_base_msgs::MoveBaseGoal> xiaofu_robot::read_target_from_file()
// {
//   std::vector<move_base_msgs::MoveBaseGoal> move_base_goal;
//   move_base_msgs::MoveBaseGoal goal;
//   string s_pose;
//   goal.target_pose.header.frame_id = "map";
//   goal.target_pose.pose.position.x = 0;
//   goal.target_pose.pose.position.y = 0;
//   goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
//   move_base_goal.push_back(goal);
//   ifstream infile(target_file_path,ios::in | ios::binary); 
//   if(!infile)
//     ROS_ERROR("file open failed");
//   while(getline(infile,s_pose))
//   {
//     size_t s = s_pose.find_first_not_of(' ');
//     size_t e = s_pose.find_last_not_of(' ');
//     std::stringstream ss(s_pose.substr(s, e - s + 1));
//     float data_pose[3];
//     int i_pose = 0; 
//     while (ss.good()) 
//     {
//         ss >> data_pose[i_pose];
//         i_pose++;
//     }
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.pose.position.x = data_pose[0];
//     goal.target_pose.pose.position.y = data_pose[1];
//     goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, data_pose[2]);
//     move_base_goal.push_back(goal);
//   }
//   infile.close();  
//   return move_base_goal;
// }

map<string,move_base_msgs::MoveBaseGoal> xiaofu_robot::read_target_from_file()
{
  map<string,move_base_msgs::MoveBaseGoal> move_base_goal;
  move_base_msgs::MoveBaseGoal goal;
  string s_pose,position_name;
  bool flag = true;
  ifstream infile(target_file_path,ios::in | ios::binary); 
  if(!infile)
    ROS_ERROR("file open failed");
  while(getline(infile,s_pose))
  {
    size_t s = s_pose.find_first_not_of(' ');
    size_t e = s_pose.find_last_not_of(' ');
    std::stringstream ss(s_pose.substr(s, e - s + 1));
    float data_pose[3];
    int i_pose = 0; 
    while (ss.good()) 
    {
        if(flag)
        {
          ss >> position_name;
          flag = false;
        }
        ss >> data_pose[i_pose];
        i_pose++;
    }
    flag = true;
    
    // ROS_INFO("test:%s,%.2f,%.2f,%.2f",position_name.c_str(),data_pose[0],data_pose[1],data_pose[2]);
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = data_pose[0];
    goal.target_pose.pose.position.y = data_pose[1];
    goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, data_pose[2]);
    move_base_goal.insert(pair<string,move_base_msgs::MoveBaseGoal>(position_name,goal));
  }
  infile.close();  
  // for (auto const& test_iterator : move_base_goal)
  // {
  //   ROS_INFO("test:%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",test_iterator.first.c_str(),test_iterator.second.target_pose.pose.position.x,test_iterator.second.target_pose.pose.position.y,
  //   test_iterator.second.target_pose.pose.orientation.x,test_iterator.second.target_pose.pose.orientation.y,test_iterator.second.target_pose.pose.orientation.z,test_iterator.second.target_pose.pose.orientation.w);
  // }
  return move_base_goal;
}

// void xiaofu_robot::target_publish(int single_target)
bool xiaofu_robot::target_publish(string tmp_position_name)
{
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    // auto target_goal = read_target_from_file();
    // target_goal[single_target].target_pose.header.stamp = ros::Time::now();
    // ac.sendGoal(target_goal[single_target]);//给目标点打时间戳并发布
    // goal = read_target_from_file();
    const auto tmp_iterator = move_goal.find(tmp_position_name);
    if(tmp_iterator!= move_goal.end())
    {
      if(!ac.waitForServer(ros::Duration(60)))
      {
        ROS_INFO("Can't connected to move base server");
        return false;
      }
      tmp_iterator->second.target_pose.header.stamp = ros::Time::now();
      ac.sendGoal(tmp_iterator->second);
      ROS_INFO("start move to target %s",tmp_position_name.c_str());
      bool finished_within_time = ac.waitForResult(ros::Duration(180));//执行任务在180s之内,超出后取消任务
      if(!finished_within_time)
      {
          ac.cancelGoal();
          ROS_INFO("Timed out achieving goal");
      }
      else
      {
          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              ROS_INFO("target %s succeeded!",tmp_position_name.c_str());
          }
          else
          {
              ROS_INFO("The base failed for some reason");
          }
      }
      // std_msgs::String msg;
      // msg.data = "cancal_target";
      // task_pub.publish(msg);
    }
    else 
        return false;
}

void xiaofu_robot::mutil_target_publish()
{
    // auto move_goal = read_target_from_file();
    if(move_goal.empty())
    {
        ROS_INFO("do not set target, can not navigation");
        return;
    }
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    if(!ac.waitForServer(ros::Duration(60)))
    {
      ROS_INFO("Can't connected to move base server");
      return ;
    }
    ROS_INFO("start_target_move");
    auto iter = move_goal.begin();
    while(iter != move_goal.end() && navi_flag )
    //for(auto iter = move_goal.begin(); iter != move_goal.end(); iter++)
    {
      iter->second.target_pose.header.stamp = ros::Time::now();
      ac.sendGoal(iter->second);
      ROS_INFO("start move to target %s",iter->first.c_str());
      bool finished_within_time = ac.waitForResult(ros::Duration(180));//执行任务在180s之内,超出后取消任务
      if(!finished_within_time)
      {
          ac.cancelGoal();
          ROS_INFO("Timed out achieving goal");
      }
      else
      {
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("target %s succeeded!",iter->first.c_str());
        }
        else
        {
            ROS_INFO("The base failed for some reason");
        }
      }
      iter++;
      if(iter == move_goal.end())
      {
        ROS_INFO("Robot have finished navigation");
        iter = move_goal.begin();
        ROS_INFO("start navigating again");
      }

      ros::spinOnce();
    }
}

void xiaofu_robot::androidCallback(const std_msgs::String::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  tmp_cmd_control = msg->data.c_str();
  // xiaofu_task_exec_cb(tmp_cmd_control,xiaofu_task_exec);
}

bool xiaofu_robot::xiaofu_task_server( xiaofu_task_server::xiaofu_server::Request &req,  xiaofu_task_server::xiaofu_server::Response &res)
{
  // 将接收到的消息打印出来
  // ROS_INFO("I heard : [%s],[%s]", req.data_req.c_str(),req.context.c_str());
  tmp_cmd_control = req.data_req;
  cout<<tmp_cmd_control<<endl;
  context = req.context;
  xiaofu_task_exec_cb(tmp_cmd_control,context,xiaofu_task_exec);
  // cout<< android_res << endl;
  res.data_res = android_res;
  return true;
}

void xiaofu_robot::xiaofu_task_exec_cb(string cmd_control,string _context_,xf_cb p)
{
    p(cmd_control,_context_,this);
}
string xiaofu_robot::xiaofu_task_exec(string cmd_control,string _context,void* arg)
{
    xiaofu_robot *_this = (xiaofu_robot *)arg;
    _this->android_res = "";
    // ROS_INFO("[%s]",_this->android_res);
    if(cmd_control == "request_target_pose")
    {   
      if(_this->move_goal.empty())
      {
        _this->android_res = "no_target_pose";
        return _this->android_res;
      }
      for(const auto& map_iterator : _this->move_goal)
      {
        _this->android_res += map_iterator.first + " ";
      }
    }
    if(_this->charge_flag)
    {
        _this->android_res = "charge_mode";
        return _this->android_res;
    }

    if(cmd_control == "delete_one_target")
    {
      if(_this->move_goal.erase(_context))
      {
        fstream file(_this->target_file_path, ios::out);
        if(!file)
          ROS_ERROR("file open failed");
	      file << std::setprecision(10);

      	for(const auto& map_iterator : _this->move_goal)
    		  file << map_iterator.first<<" " << map_iterator.second.target_pose.pose.position.x<< " "<< map_iterator.second.target_pose.pose.position.y << " "<< tf::getYaw(map_iterator.second.target_pose.pose.orientation) << "\n";   
        _this->android_res = "delete " + _context + " target pose successful";
      }
      else
        _this->android_res = "delete " + _context + " target pose failed, target pose have been deleted or is not exist";
    }
    if(cmd_control == "single_navigation")
    {
      
      bool test_flag = _this->target_publish(_context);
      if(!test_flag &&_this->ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        _this->android_res = "finished single target pose navigation";
      }
      else
        _this->android_res = "single target pose navigation failed for some reasons";
    }
    if(cmd_control == "mutil_target")
    {
      _this->navi_flag = true;
      _this->android_res = "mutil_target";
      _this->mutil_target_publish();
    }
    if(cmd_control == "cancel_mutil") 
    {
      _this->navi_flag = false;
      // _this->ac.cancelGoal();
      ROS_INFO("Cancel task");
      _this->android_res = "cancel_mutil successful";
    }
    if(cmd_control == "forward")
    {
      //_this->Robot_Led_control(cmd_control);
      // get_leg_callback(send_success,this,&xiaofu_robot::leg_callback);
      _this->forward_and_back(cmd_control,1,0.3,50); 
      _this->android_res = "forward move successful";
    }
    if (cmd_control == "back")
    {
      //_this->Robot_Led_control(cmd_control);
      // get_leg_callback(send_success,this,&xiaofu_robot::leg_callback);
      _this->forward_and_back(cmd_control,1,0.3,50);
      _this->android_res = "back move successful";
    }

    if (cmd_control == "turn_left")
    {
      //_this->Robot_Led_control(cmd_control);
      // get_leg_callback(send_success,this,&xiaofu_robot::leg_callback);
      _this->turn_left_and_right(cmd_control,M_PI/2,1,50);
      _this->android_res = "turn left successful";
    }
    if(cmd_control == "turn_right")
    {
      //_this->Robot_Led_control(cmd_control);
      // get_leg_callback(send_success,this,&xiaofu_robot::leg_callback);
      _this->turn_left_and_right(cmd_control,M_PI/2,1,50);
      _this->android_res = "turn right successful";
    }
    if(cmd_control == "stop")
    {
      _this->cmd_vel_publisher.publish(geometry_msgs::Twist());
      _this->android_res = "stop successful";
    }
    if(cmd_control == "poweroff")
    {
      _this->android_res = "start shutdown";
      _this->open_battery(cmd_control);
    }
    if(cmd_control == "save_target_pose")
    {
      _this->save_target_to_file();
      _this->android_res = "save target pose successful";
    }

    if(cmd_control == "save_map")
    {
      std::string mapname = _this->map_file_path + "xiaofu_map";
      int threshold_occupied = 65;
      int threshold_free = 25;
      MapGenerator mg(mapname, threshold_occupied, threshold_free);
      while(!mg.saved_map_ && ros::ok())
          ros::spinOnce();
      _this->android_res = "save map successful";
    }
    if(cmd_control == "delete_map")
    {
      const char *mapPath = (_this->map_file_path + "xiaofu_map.pgm").c_str();
      const char *map_yaml_Path = (_this->map_file_path + "xiaofu_map.yaml").c_str();
      if(!remove(mapPath)&&!remove(map_yaml_Path))
          _this->android_res = "delete map successful";
          // ROS_INFO("delete successful!!!");//res.data_res = "delete map successful";
      else 
          ROS_INFO("delete failed!!!");//res.data_res = "did not find map file";
    }
    if (cmd_control == "load_map")
    {
      std::string fname = _this->map_file_path + "xiaofu_map.yaml";
      double res1 = 0.0;
      try
      {
        MapServer ms(fname, res1);
        _this->android_res = "load map successful";
        ros::spin();
      }
      catch(std::runtime_error& e)
      {
        ROS_ERROR("map_server exception: %s", e.what());
      }
    }
    if(cmd_control =="delete_target")
    {
        fstream file(_this->target_file_path, ios::out);
        _this->android_res = "delete map successful";
    }

    return _this->android_res;
}

void* xiaofu_robot::xiaofu_vel_cmd_pub(void *arg)
{ 
  xiaofu_robot *_this = (xiaofu_robot *)arg;
  ros::Rate sensor_hz_control(2); 
  std_msgs::Int64 msg;
  while(ros::ok())
  {
    _this->battery_data = _this->Get_Battery_Data();
    if (_this->battery_data.rx[0] == 0x55)
    {
      msg.data = _this->battery_data.RSOC;
      _this->battery_pub.publish(msg);
      if(_this->battery_data.RSOC < 10 && _this->battery_data.RSOC != 0)
      {
        _this->target_publish("p8");
      }
      if(_this->battery_data.Current > 0)
      {
          //_this->Robot_Led_control("charge");
          //_this->batter_cmd_vel_publisher.publish(geometry_msgs::Twist());
	  _this->charge_flag = true;
      }
      else
	  _this->charge_flag = false;
    }
    if(_this->tmp_cmd_control != "poweroff")
        _this->open_battery("heartbeat");
  }
}

void* xiaofu_robot::xiaofu_sensor(void *arg)
{ 
  xiaofu_robot *_this = (xiaofu_robot *)arg;
  ros::Rate sensor_hz_control(100); 
  while(ros::ok())
  {
    SENSOR_DATA sensor_data = _this->Get_Sensor_Data();	
    xiaofu_task_server::sensor_pub sensor;//赋值传感器数据并发布
    sensor_msgs::Range wave0 , wave1;
    sensor.Wave = {sensor_data.Wave[0],sensor_data.Wave[1],sensor_data.Wave[2],sensor_data.Wave[3],sensor_data.Wave[4]};
    sensor.Switch = sensor_data.Switch;
    sensor.Temp = sensor_data.Temp;
    sensor.Humidity = sensor_data.Humidity;
    sensor.PM_25 = sensor_data.PM_25;
    wave0.range = sensor_data.Wave[0]/100;
    wave1.range = sensor_data.Wave[1]/100;
    _this->sensor_pub.publish(sensor);
    wave0.header.stamp = ros::Time::now();
    wave1.header.stamp = ros::Time::now();
    _this->wave0_pub.publish(wave0);
    _this->wave1_pub.publish(wave1);
    sensor_hz_control.sleep();
  }
}



void xiaofu_robot::thread_fun()
{
    pthread_t cmd_pub_thread,xiaofu_sensor_thread; //创建线程
    // pthread_create(&cmd_pub_thread,NULL,xiaofu_vel_cmd_pub,(void *)this);
    // pthread_create(&xiaofu_sensor_thread,NULL,xiaofu_sensor,(void *)this);
}


xiaofu_robot::xiaofu_robot():Sampling_Time(0),send_success(false),navi_flag(false),charge_flag(false)//初始化构造函数
{
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("sensor_usart_port_name", sensor_usart_port_name, "/dev/ttyS1"); //定义底盘传感器板串口
  private_nh.param<std::string>("battery_usart_port_name",battery_usart_port_name,"/dev/ttyS0" ); // 定义电源板串口
  private_nh.param<std::string>("target_file_path",target_file_path,"/home/lxd/target_pose.csv");
  private_nh.param<std::string>("map_file_path",map_file_path,"/home/lxd/");
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //定义波特率
  //ros订阅安卓控制话题
  // sub_task = n.subscribe("/cmd", 1, &xiaofu_robot::androidCallback,this); 
  sub_target_pose = n.subscribe("/move_base_simple/goal",10,&xiaofu_robot::save_target_pose_callback,this);
  //ros发布话题
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/voice/cmd_vel", 10,this);  //前进后退左转右转命令发布
//  batter_cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/battery/cmd_vel", 10,this);
  battery_pub = n.advertise<std_msgs::Int64>("/battery", 1000,this);  //发布电池电量话题
  wave0_pub = n.advertise<sensor_msgs::Range>("Wave0",10,this);
  wave1_pub = n.advertise<sensor_msgs::Range>("Wave1",10,this);
  task_pub = n. advertise<std_msgs::String>("cmd",10,this);
  //雷达启动和关闭服务声明
  lidar_start_client = n.serviceClient<std_srvs::Empty>("start_motor",this);
  lidar_stop_client = n.serviceClient<std_srvs::Empty>("stop_motor",this);
 
  xiaofu_task = n.advertiseService("xiaofu_task_server",&xiaofu_robot::xiaofu_task_server,this);

  //sensor msg publish
  sensor_pub = n.advertise<xiaofu_task_server::sensor_pub>("/xiaofu_sensor",1000,this);
  signal(SIGINT,signalHandler);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //调试接口
  //先后打开传感器串口和电源板串口
  //  try{
  //    sensor_Serial.setPort(sensor_usart_port_name);
  //    sensor_Serial.setBaudrate(serial_baud_rate);
  //    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
  //    sensor_Serial.setTimeout(_time);
  //    sensor_Serial.open();//开启串口
  //  }
  //  catch (serial::IOException& e){
  //        ROS_ERROR_STREAM("snesor_usart_port_name can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  //  }
  //  if(sensor_Serial.isOpen()){
  //    ROS_INFO_STREAM("sensor serial port opened");//开启成功
  //  }else{
  //  }
  // try{
  //   battery_Serial.setPort(battery_usart_port_name);
  //   battery_Serial.setBaudrate(serial_baud_rate);
  //   serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
  //   battery_Serial.setTimeout(_time);
  //   battery_Serial.open();//开启串口
  // }
  // catch (serial::IOException& e){
  //       ROS_ERROR_STREAM("battery_usart_port_name can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  // }
  // if(battery_Serial.isOpen()){
  //   ROS_INFO_STREAM("battery serial port opened");//开启成功
  // }else{
  //  }

    fstream file;
    file.open(target_file_path,ios::in);
    file.get();
    string clean_target_pose;
    if(move_goal.empty() && !(file.eof()))
    {
        //ROS_INFO("target pose have been exist, clean the pose? yes/no");
        //cin>>clean_target_pose;
        //if(clean_target_pose == "yes")
        //{
        //  fstream file(target_file_path, ios::out);
        //  ROS_INFO("delete target pose success, set new target pose by rviz");
        //}
        //else 
        //{
          move_goal = read_target_from_file();
          ROS_INFO("load target successful");
        //}
    }
    else
    {
        ROS_INFO("publish /move_base_simple/goal topic by rviz to save target pose and save target pose to file!!!");
    }

}

/**************************************
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
xiaofu_robot::~xiaofu_robot()
{
   delete this;
}


void xiaofu_robot::open_battery(string control)
{
  short  transition;  //中间变量
  if(control=="open_battery")
  {
      Send_battery_control.tx[1] = 0xff;
      Send_battery_control.tx[2] = 0x00;
  }
  else if(control == "poweroff")
  {
      Send_battery_control.tx[1] = 0x80;
      Send_battery_control.tx[2] = 0x80;
  }
  else 
  {
      Send_battery_control.tx[1] = 0x00;
      Send_battery_control.tx[2] = 0x00;
  }
  Send_battery_control.tx[0] = FRAME_HEADER;
  
  u16 tmp = MODBUS_CRC(Send_battery_control.tx,SET_SEND_BATTERY_DATA_SIZE-2);
  Send_battery_control.tx[3] = tmp >> 8;
  Send_battery_control.tx[4] = tmp;
  try
  {
    battery_Serial.write(Send_battery_control.tx,sizeof(Send_battery_control.tx)); //向串口发数据
  }
  catch (serial::IOException& e){
    ROS_ERROR_STREAM("serial error!!! ");
  }
}

void xiaofu_robot::get_leg_callback(bool recieve_success, void *arg, p_cb p)
{
  if(recieve_success)
    RECEIVE_SENSOR_CALLBACK_DATA  led_callback = p(recieve_success,arg);
}

RECEIVE_SENSOR_CALLBACK_DATA xiaofu_robot::leg_callback(bool recieve_success,void* arg)
{
    xiaofu_robot *_this = (xiaofu_robot *) arg;
    short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
    uint8_t Receive_Data_Pr[SENSOR_CALLBACK_DATA_SIZE]={0};
    RECEIVE_SENSOR_CALLBACK_DATA led;
    _this->sensor_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
    for(j=0;j<4;j++)
    {
      if(Receive_Data_Pr[j]==LED_ACK_HEADER)
        {Header_Pos=j;}  
    }
    
    led.Frame_Header = led.rx[0]; //数据的第一位是帧头（固定值）
    if (led.Frame_Header == LED_ACK_HEADER )//判断帧头
    {
        //补充CRC校验
        if(_this->Check_CRC(led.rx, 4))
        {
           for(int i=0;i<4;i++)
           {
             ROS_INFO("%x",led.rx[i]);
           }
           recieve_success = false;
           return led;
        }
    }
}
void xiaofu_robot::Robot_Led_control(string light_control)
{
  if(light_control == "forward" || light_control == "back")
    memcpy(Send_Sensor_Led_Data.tx,forward_mode,sizeof(forward_mode));
  if(light_control == "turn_left")
    memcpy(Send_Sensor_Led_Data.tx,left_mode,sizeof(left_mode));
  if(light_control == "turn_right")
    memcpy(Send_Sensor_Led_Data.tx,right_mode,sizeof(right_mode));
  if(light_control == "charge")
    memcpy(Send_Sensor_Led_Data.tx,charge_mode,sizeof(charge_mode));
  u16 tmp = MODBUS_CRC(Send_Sensor_Led_Data.tx,SET_SEND_SENSOR_LED_DATA_SIZE-2);
  Send_Sensor_Led_Data.tx[14] = tmp >> 8;
  Send_Sensor_Led_Data.tx[15] = tmp;
  try
  {
      sensor_Serial.write(Send_Sensor_Led_Data.tx,SET_SEND_SENSOR_LED_DATA_SIZE); //向串口发数据
  }
  catch (serial::IOException& e){
    ROS_ERROR_STREAM("serial error!!! ");
  }
  send_success = true;
}

u8 xiaofu_robot::Check_CRC(u8 *data, int size) //CRC 校验
{
    if(MODBUS_CRC(data,size-2)==(((u16)data[size-2]<<8)|data[size-1]))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
u16 xiaofu_robot::MODBUS_CRC(u8 *buff,u8 len) //modbus 协议的 crc 校验
{
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;
    u8 n = 0;
    u8 i = 0;
    for (n = 0; n < len; n++) 
    { //要校验的位数为 len 个
        tmp = buff[n] ^ tmp;
        for (i = 0; i < 8; i++) 
        { //此处的 8 -- 指每一个 char 类型又 8bit，每 bit 都要处理
            if (tmp & 0x01) 
            {
              tmp = tmp >> 1;
              tmp = tmp ^ 0xa001;
            }
            else 
            {
              tmp = tmp >> 1;
              }
        }
      } /*CRC 校验后的值*/
      ret1 = tmp >> 8;
      ret1 = ret1 | (tmp << 8);
      return ret1;
}
float xiaofu_robot::shorttofloat(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;  //获取数据的高8位
      transition_16 |=  Data_Low;     //获取数据的低8位
      data_return   =  (transition_16 / 1000)+(transition_16 % 1000); //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
  return data_return;
}
BATTERY_DATA xiaofu_robot::Get_Battery_Data()
{
  BATTERY_DATA battery_data;
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_BATTERY_DATA_SIZE]={0};
  battery_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
  for(j=0;j<18;j++)
    {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
    }
    //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
    if(Tail_Pos==(Header_Pos+17))
    {
         //ROS_INFO("1----");
      memcpy(battery_data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))
    {
        //ROS_INFO("2----");
        for(j=0;j<18;j++)
        battery_data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%18];
    }
    else 
    {
     //ROS_INFO("3----");
     //return false;
    }    
    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    //battery_data.rx[0],battery_data.rx[1],battery_data.rx[2],battery_data.rx[3],battery_data.rx[4],battery_data.rx[5],battery_data.rx[6],battery_data.rx[7],
    //battery_data.rx[8],battery_data.rx[9],battery_data.rx[10],battery_data.rx[11],battery_data.rx[12],battery_data.rx[13],battery_data.rx[14],battery_data.rx[15],
    //battery_data.rx[16],battery_data.rx[17],battery_data.rx[18]); 
  
  battery_data.Frame_Header= battery_data.rx[0]; //数据的第一位是帧头（固定值）
  battery_data.Frame_Tail= battery_data.rx[17];  //数据的最后一位是帧尾（数据校验位）

 if (battery_data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (battery_data.Frame_Tail == FRAME_TAIL) //判断帧尾
    { 
      //补充CRC校验
      if(Check_CRC(battery_data.rx, 17))
      { 
        //ROS_INFO("TEST");
        battery_data.Voltage = battery_data.rx[1]*256+battery_data.rx[2];
        battery_data.Current = shorttofloat(battery_data.rx[3],battery_data.rx[4]);
	      battery_data.Quantity = battery_data.rx[5]*256+battery_data.rx[6];
        battery_data.Capacity = battery_data.rx[7]*256+battery_data.rx[8];
        battery_data.RSOC = battery_data.rx[9];
        battery_data.Ctrl = battery_data.rx[10];
        battery_data.Protect = battery_data.rx[11]*256+battery_data.rx[12];
        battery_data.Cycle = battery_data.rx[13]*256+battery_data.rx[14];
//        ROS_INFO("%.2f,%.2f,%.2f,%d\%,%d,%d",battery_data.Voltage/100,battery_data.Current/100,battery_data.Quantity/100,battery_data.RSOC,battery_data.Ctrl,battery_data.Protect,battery_data.Cycle);
        //return true;
        return battery_data;
     }
    }
  }    
}
SENSOR_DATA xiaofu_robot::Get_Sensor_Data()
{
  SENSOR_DATA  sensor_data;
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_SENSOR_DATA_SIZE]={0};
  sensor_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
  for(j=0;j<21;j++)
  {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
  }
  //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
  if(Tail_Pos==(Header_Pos+20))
  {
        //ROS_INFO("1----");
    memcpy(sensor_data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
  }
  else if(Header_Pos==(1+Tail_Pos))
  {
      //ROS_INFO("2----");
      for(j=0;j<21;j++)
      sensor_data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%21];
  }
  else 
  {
    //ROS_INFO("3----");
    //return false;
  }    
  sensor_data.Frame_Header= sensor_data.rx[0]; //数据的第一位是帧头（固定值）
  sensor_data.Frame_Tail= sensor_data.rx[20];  //数据的最后一位是帧尾（固定值）

  if (sensor_data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (sensor_data.Frame_Tail == FRAME_TAIL) //判断帧尾
    {
      //补充CRC校验
      if(Check_CRC(sensor_data.rx, 20))
      { 
        sensor_data.Wave[0] = shorttofloat(sensor_data.rx[1],sensor_data.rx[2]);
        sensor_data.Wave[1] = shorttofloat(sensor_data.rx[3],sensor_data.rx[4]);
        sensor_data.Wave[2] = shorttofloat(sensor_data.rx[5],sensor_data.rx[6]);
        sensor_data.Wave[3] = shorttofloat(sensor_data.rx[7],sensor_data.rx[8]);
        sensor_data.Wave[4] = shorttofloat(sensor_data.rx[9],sensor_data.rx[10]);
        sensor_data.Switch =  sensor_data.rx[11];//shorttofloat(0,sensor_data.rx[11]);
        sensor_data.Temp =  shorttofloat(sensor_data.rx[12],sensor_data.rx[13]);
        sensor_data.Humidity = shorttofloat(sensor_data.rx[14],sensor_data.rx[15]);
        sensor_data.PM_25 =  shorttofloat(sensor_data.rx[16],sensor_data.rx[17]);
//        ROS_INFO("%x-%x-%x-%x",sensor_data.rx[1],sensor_data.rx[2],sensor_data.rx[3],sensor_data.rx[4]);
//        ROS_INFO("Switch:%x,Wave1:%.0f,Wave2:%.0f,Wave3:%.0f,Wave4:%.0f,Wave5:%.0f,Temp:%.0f,Humi:%.0f,PM_25:%.0f",sensor_data.Switch,sensor_data.Wave[0],sensor_data.Wave[1],sensor_data.Wave[2],sensor_data.Wave[3],sensor_data.Wave[4],sensor_data.Temp,sensor_data.Humidity,sensor_data.PM_25);
        return sensor_data;
     }
    }
  } 
}



void xiaofu_robot::forward_and_back(string cmd,float goal_distance,float linear_speed,double rate)
{
    geometry_msgs::Twist speed;
    float time_duration = goal_distance / linear_speed;
    //Robot_Led_control(cmd);
    ros::Rate loopRate(rate);
    if(cmd == "forward")
        speed.linear.x = linear_speed;
    if(cmd == "back")
        speed.linear.x = -linear_speed;
    int ticks = int(time_duration * rate);
    for(int i = 0; i < ticks; i++)
    {
      //temp_sensor_data = Get_Sensor_Data();
     // if(speed.linear.x > 0 && temp_sensor_data.Wave[0]>20 && temp_sensor_data.Wave[1]>20 )
     // {
          cmd_vel_publisher.publish(speed);
          //Robot_Led_control(cmd);
    //  } 
  //    if(speed.linear.x < 0 && temp_sensor_data.Wave[2]>20 && temp_sensor_data.Wave[3]>20 && temp_sensor_data.Wave[4]>20)
//      {
//        cmd_vel_publisher.publish(speed);
        //Robot_Led_control(cmd);
//      }
      loopRate.sleep();
    }
    speed.linear.x = 0;
    cmd_vel_publisher.publish(speed);
}

void xiaofu_robot::turn_left_and_right(string cmd,float goal_angle,float angle_speed,double rate)
{
  geometry_msgs::Twist speed;
  float time_duration = goal_angle / angle_speed;
  ros::Rate loopRate(rate);
  //Robot_Led_control(cmd);
  if(cmd == "turn_left")
  {
    speed.angular.z = angle_speed;
    //Robot_Led_control(cmd);
  }    
  if(cmd == "turn_right")
  {
    speed.angular.z = -angle_speed;
    //Robot_Led_control(cmd);
  }
  int ticks = int(time_duration * rate);
  for(int i = 0; i < ticks; i++)
  {
    cmd_vel_publisher.publish(speed); // 将刚才设置的指令发送给机器人
    loopRate.sleep();
  }
  speed.angular.z = 0;
  cmd_vel_publisher.publish(speed);
}

void xiaofu_robot::turn_left_and_right(float goal_angle)
{
    if(goal_angle > M_PI)
    {
      goal_angle = 3 * M_PI / 2 - goal_angle ;
      turn_left_and_right("turn_right",goal_angle,1,50);
    }
    else
    {
      goal_angle = M_PI - goal_angle;
      turn_left_and_right("turn_left",goal_angle,1,50);
    }
}
