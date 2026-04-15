

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <actionlib/server/simple_action_server.h>
#include <matrix_msgs/ARTrackOperationAction.h>
#include <dynamic_reconfigure/server.h>
#include <matrix_docking_track/MatrixARTrackOperationConfig.h>
using namespace matrix_msgs;
using namespace matrix_docking_track;

class MatrixARTrackOperation
{    
    protected:
        ros::NodeHandle n_;

        // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        actionlib::SimpleActionServer<ARTrackOperationAction> as_;
        std::string action_name_;
        ARTrackOperationFeedback feedback_;
        ARTrackOperationResult result_;

        geometry_msgs::Twist cmd_msg;
        ar_track_alvar_msgs::AlvarMarkers ar_tack_;

        ros::Publisher cmd_vel_publisher_;

        ros::Subscriber ARTrackDetect_subscriber_;

        std::string serial_no = "20210001B";
        std::string model= "smr200";
        int track_control_timeout = 30;
        int track_loss_threshold = 2;
        float lin_p = 0.17;
        float lin_i = 0.05;
        float lin_d = 0.03;
        float ang_p = 0.17;
        float ang_i = 0.05;
        float ang_d = 0.03;

        float def_lin_p;
        float def_lin_i;
        float def_lin_d;
        float def_ang_p;
        float def_ang_i;
        float def_ang_d;

        float lastError_dy, lastError_dx;

    
    public:
        MatrixARTrackOperation(std::string name) :
            as_(n_, name, boost::bind(&MatrixARTrackOperation::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            as_.registerPreemptCallback(boost::bind(&MatrixARTrackOperation::preemptCB, this));
            // init_srv_client();
            init_pub();
            init_sub();
            init_param();
            init_dynamic_reconfigure();
        }

        void init_dynamic_reconfigure()
        {
            // dynamic_reconfigure::Server<MatrixARTrackOperationConfig> dyn_srv_;
            // dyn_srv_.setCallback(boost::bind(&MatrixARTrackOperation::callback, this, _1, _2));

            // dynamics reconfigure node 
            dynamic_reconfigure::Server<matrix_docking_track::MatrixARTrackOperationConfig> server;
            dynamic_reconfigure::Server<matrix_docking_track::MatrixARTrackOperationConfig>::CallbackType f;

            f = boost::bind(&MatrixARTrackOperation::callback, this, _1, _2);
            server.setCallback(f);
            ROS_INFO("Dynamics reconfigure start");

        }

        void callback(matrix_docking_track::MatrixARTrackOperationConfig &config, uint32_t level)
        {
            if(config.restore_defult)
            {
                config.lin_p = def_lin_p;
                config.lin_i = def_lin_i;
                config.lin_d = def_lin_d;

                config.ang_p = def_ang_p;
                config.ang_i = def_ang_i;
                config.ang_d = def_ang_d;

                config.restore_defult = false;
            }

            track_control_timeout = config.track_control_timeout;
            track_loss_threshold = config.track_loss_threshold;
            lin_p = config.lin_p;
            lin_i = config.lin_i;
            lin_d = config.lin_d;
            ang_p = config.ang_p;
            ang_i = config.ang_i;
            ang_d = config.ang_d;
        }

        void init_param()
        {
            ros::param::param<std::string>("~serial_no", serial_no, "20210001B");
            ros::param::param<std::string>("~model", model, "smr200");
            ros::param::param<int>("~track_control_timeout", track_control_timeout, 30);
            ros::param::param<int>("~track_loss_threshold", track_loss_threshold, 2);
            ros::param::param<float>("~lin_p", lin_p, 0.17);
            ros::param::param<float>("~lin_i", lin_i, 0.05);
            ros::param::param<float>("~lin_d", lin_d, 0.03);
            ros::param::param<float>("~ang_p", ang_p, 0.17);
            ros::param::param<float>("~ang_i", ang_i, 0.05);
            ros::param::param<float>("~ang_d", ang_d, 0.03);
            
            def_lin_p = lin_p;
            def_lin_i = lin_i;
            def_lin_d = lin_d;

            def_ang_p = ang_p;
            def_ang_i = ang_i;
            def_ang_d = ang_d;

        }

        void init_pub()
        {
            cmd_vel_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

        void init_sub()
        {
            ARTrackDetect_subscriber_ = n_.subscribe("/ar_pose_marker", 1, &MatrixARTrackOperation::cbARTrack, this);
        }

        void cbARTrack(const ar_track_alvar_msgs::AlvarMarkers &msg)
        {
            ar_tack_ = msg;  
        }

        ~MatrixARTrackOperation(void)
        {
        }

        void preemptCB()
        {
            ROS_WARN("%s got preempted!", action_name_.c_str());
            as_.setPreempted();
        }

        void executeCB(const ARTrackOperationGoalConstPtr &goal)
        {
            ros::Rate rate(20);

            bool success = false;
            bool task_complete = false;

            bool _isTrackMatch = false;
            bool _isControlFinish = false;

            double  track_control_timeout_,
                    track_control_begin_;

            feedback_.sequence_state = ARTrackOperationFeedback::INITPARAM;

            if(!as_.isActive() || as_.isPreemptRequested()) return;
            
            while(!task_complete)
            {
                //Check for ros
                if(!ros::ok())
                {
                    // result.final_count = progress;
                    // as_.setAborted(result,"I failed !");
                    ROS_INFO("%s Shutting down",action_name_.c_str());
                    break;
                }

                if(!as_.isActive() || as_.isPreemptRequested())
                {
                    return;
                }

                // control sequence 
                switch(feedback_.sequence_state)
                {
                    case ARTrackOperationFeedback::INITPARAM:
                        ROS_INFO("[MatrixARTrackOperation]:Current step %d : Initial parameter before start", feedback_.sequence_state);
                        feedback_.sequence_state = ARTrackOperationFeedback::SEARCH_AR;
                        break;
                    
                    case ARTrackOperationFeedback::SEARCH_AR:
                        ROS_INFO("[MatrixARTrackOperation]:Current step %d : Search AR Track %d", feedback_.sequence_state, goal->ar.track_no);
                        _isTrackMatch = SearchARTrack(goal->ar.track_no); 
                        if(_isTrackMatch)
                        {
                            feedback_.sequence_state = ARTrackOperationFeedback::TRACK_PARAM;
                        }
                        else
                        {
                            feedback_.sequence_state = ARTrackOperationFeedback::ERROR;
                            result_.result = ARTrackOperationResult::NOT_FOUND_TRACK;

                        }
                        break;
                    
                    case ARTrackOperationFeedback::TRACK_PARAM:
                        ROS_INFO("[MatrixARTrackOperation]:Current step %d : Setup parameter before Tracking", feedback_.sequence_state);
                        track_control_timeout_ = ros::Time::now().toSec() + track_control_timeout;
                        // track_control_begin_   =  ros::Time::now().toSec();
                        ResetPram();
                        feedback_.sequence_state = ARTrackOperationFeedback::TRACK_AR;
                        break;

                     case ARTrackOperationFeedback::TRACK_AR:
                        ROS_INFO("[MatrixARTrackOperation]:Current step %d : Track AR code no. %d", feedback_.sequence_state);
                        _isControlFinish = ControlTrack(goal->ar.track_no, goal->ar.dx, goal->ar.dy, goal->ar.max_vel.linear.x, goal->ar.max_vel.angular.z);
                        if(_isControlFinish)
                        {
                            ROS_INFO("[MatrixARTrackOperation]: Track control FINISH!!!");
                            feedback_.sequence_state = ARTrackOperationFeedback::FINISH;
                            result_.result = ARTrackOperationResult::TRACK_SUCCESS;
                            success = true;
                        }
                        else
                        {
                            if(abs(ros::Time::now().toSec() > track_control_timeout_))
                            {
                                ROS_ERROR("[MatrixARTrackOperation]: Track control timeout!!!");
                                result_.result = ARTrackOperationResult::TRCK_CONTROL_TIME_OUT;
                                feedback_.sequence_state = ARTrackOperationFeedback::ERROR;
                            }
                            else
                            {
                                //stay in ARTrackOperationGoal::TRACK_AR until _isControlFinish = true
                                feedback_.sequence_state = ARTrackOperationFeedback::TRACK_AR;
                            }
                        }
                         break;
                    
                    case ARTrackOperationFeedback::ERROR:
                        feedback_.sequence_state = ARTrackOperationFeedback::FINISH;
                        break;
                    
                    case ARTrackOperationFeedback::FINISH:
                        ROS_INFO("[MatrixARTrackOperation]:Current step %d : Control state finish!!", feedback_.sequence_state);
                        task_complete = true;
                        break;
                    
                        
                }

                as_.publishFeedback(feedback_);
                rate.sleep();

            }
            as_.setSucceeded(result_);
            result_.sequence_state = feedback_.sequence_state;
            result_.result = result_.result;
            ROS_INFO("%s: %s", action_name_.c_str() ,(success)? "Succeeded":"Fail" );
            ROS_INFO(" ");
            ROS_INFO("*************************");
        }

        bool SearchARTrack(int ar_no_)
        {   
            int track_found = ar_tack_.markers.size();
            if(track_found != 0)
            {
                ROS_INFO("[MatrixARTrackOperation]:Search AR Track no.%d in %d found", ar_no_, track_found);
                for(int i = 0; i < track_found; i++){
                    if(ar_tack_.markers[i].id == ar_no_){
                        ROS_INFO("[MatrixARTrackOperation]:Found AR no.%d!!", ar_tack_.markers[i].id);
                        return true;
                    }
                    else{
                        if((track_found - i) <= 1){
                            ROS_ERROR("[MatrixARTrackOperation]:Track no.%d not found", ar_no_);
                            return false;
                        }else;
                    }

                }
            }
            else{
                return false;
            }
            
        }

        // bool getDataTrack(int track_no, float distance, float tolerance)
        // {
        //     static float x_, y_;
        //     static bool found_ar =false;

        //     for(int i = 0; i < ar_tack_.markers.size(); i++)
        //     {
        //         if(ar_tack_.markers[i].id == track_no)
        //         {
        //             x_ = ar_tack_.markers[i].pose.pose.position.x;
        //             y_ = ar_tack_.markers[i].pose.pose.position.y;
        //             found_ar = true;
        //             break;
        //         }else;
        //     }

        //     if(found_ar)
        //     {
        //         if(base_control(x_, y_, tolerance) < tolerance)
        //         {
        //             return true;
        //         }
        //         else
        //         {
        //             return false;
        //         }
        //         found_ar = false;
        //     }
            
        // }

        void ResetPram()
        {
            lastError_dx = 0.0;
            lastError_dy = 0.0;
        }

        bool ControlTrack(int track_no, float dx, float dy, float max_vel_x, float max_vel_z)
        {
            // float err_d[2]       = {0, 0};
            // static float current_d[2]   = {};
            // static float d[2] = {tolerance, 0};

            static float x_, y_;
            bool found_ar = false;

            float err_dx;
            float err_dy;

            for(int i = 0; i < ar_tack_.markers.size(); i++)
            {
                if(ar_tack_.markers[i].id == track_no)
                {
                    x_ = ar_tack_.markers[i].pose.pose.position.x;
                    y_ = ar_tack_.markers[i].pose.pose.position.y;

                    found_ar = true;
                    break;
                }else;

            }

            if(found_ar)
            {
                geometry_msgs::Twist twist;
                err_dx = x_ - dx;
                err_dy = y_ - dy;

                

                if((abs(err_dx) <= 0.001) && (abs(err_dy) <= 0.003))
                {
                    twist.linear.x = 0.0;
                    twist.linear.y = 0.0;
                    twist.linear.z = 0.0;
                    twist.angular.x = 0.0;
                    twist.angular.y = 0.0;
                    twist.angular.z = 0.0;
                    cmd_vel_publisher_.publish(twist);
                    return true;
                }else
                {
                    float Kp_x = lin_p;
                    float Ki_x = lin_i;
                    float Kd_x = lin_d;

                    float linear_x = (Kp_x * err_dx) + (Ki_x*(err_dx + lastError_dx)) + (Kd_x * (err_dx - lastError_dx));
                    lastError_dx = err_dx;



                    // err_dy = y_ - dy;
                    float angular_z;

                    // if(abs(err_dy) > 0.00)
                    // {
                        float Kp_y = ang_p;
                        float Ki_y = ang_i;
                        float Kd_y = ang_d;

                        angular_z = (Kp_y * err_dy) + (Ki_y*(err_dy + lastError_dy)) + (Kd_y * (err_dy - lastError_dy));
                        
                    // }
                    // else{
                    //     angular_z = 0;
                    // }
                    
                    lastError_dy = err_dy;

                    
                    twist.linear.x = (linear_x < 0) ? -std::max(linear_x, -max_vel_x) : -std::min(linear_x, max_vel_x);
                    twist.linear.x = -twist.linear.x;
                    twist.linear.y = 0.0;
                    twist.linear.z = 0.0;
                    twist.angular.x = 0.0;
                    twist.angular.y = 0.0;
                    twist.angular.z = (angular_z < 0) ? -std::max(angular_z, -max_vel_z) : -std::min(angular_z, max_vel_z);
                    // if(abs(twist.angular.z) >= 0.03)
                    // {
                        // twist.linear.x = abs(std::min(copysign((float)max_vel_x, (float)twist.linear.x), std::max(copysign((float)0.000, (float)twist.linear.x), (float) twist.linear.x)));
                        twist.angular.z = -(std::min(copysign((float)max_vel_z, (float)twist.angular.z), std::max(copysign((float)0.000, (float)twist.angular.z), (float) twist.angular.z)));
                    // }
                    // else
                    // {
                    //     twist.angular.z = 0.000;
                    // }

                    

                    cmd_vel_publisher_.publish(twist);

                    ROS_INFO("error_dx:%f, error_dy:%f, linear_x:%f, angular_z:%f", err_dx, err_dy, twist.linear.x, twist.angular.z);
                        return false;
                }
                
                

                

            }else
            {
                return false;
            }

        }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MatrixARTrackOperation");
  ROS_INFO("Start matrix_artrack_operation server");

  MatrixARTrackOperation missions("matrix_artrack_operation");
  ros::spin();

  return 0;
}