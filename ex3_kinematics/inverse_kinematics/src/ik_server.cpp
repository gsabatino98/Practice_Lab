#include <ros/ros.h>
#include <inverse_kinematics_msgs/ik_actionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

class IK_Solver
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<inverse_kinematics_msgs::ik_actionAction> as;
    std::string action_name;
    inverse_kinematics_msgs::ik_actionFeedback feedback_;
    inverse_kinematics_msgs::ik_actionResult result_;

public:
    IK_Solver(std::string name) : as(nh_, name, boost::bind(&IK_Solver::executeCB, this, _1), false),
                                      action_name(name)
    {

        as.start();
    }

    ~IK_Solver(void) {}


    bool check_solution(std::vector<std::vector<double>> all_ik_solution,std::vector< double > ik_solution){
        int dim_sol = ik_solution.size();
        int all_sol = all_ik_solution.size();
        int i = 0;
        int j = 0;

        for (i = 0; i < all_sol; i++)
        {
            for (j = 0; j < dim_sol; j++)
            {
                if (all_ik_solution[i][j]!=ik_solution[j])
                {
                    return true;
                }
            }            
        }
        return false;
        
    }

    void sendFeedback(std::vector<double> ik_solution, moveit_msgs::RobotState r_state){

        r_state.joint_state.position=ik_solution;
        feedback_.robot_state=r_state;
        result_.robot_state.push_back(r_state);
        ROS_INFO("Feedback della soluzione inviato !!!!!");
        for (int i = 0; i < ik_solution.size(); i++)
        {
            ROS_INFO_STREAM("\t"<<feedback_.robot_state.joint_state.position[i]);
        }
        
        as.publishFeedback(feedback_);

    }

    void sendResult(std::string msg_goal){
        ROS_INFO("Invio risultato");
        as.setSucceeded(result_,msg_goal);
        ros::shutdown();
    }

    void executeCB(const inverse_kinematics_msgs::ik_actionGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("INIZIO");
        const geometry_msgs::Pose& pose = goal->pose_goal;
        const std::vector< double >& ik_seed = goal->robot_state.joint_state.position;
        double timeout = 50;
        std::vector< double > solution;
        std::vector<std::vector<double>> all_solutions;
        moveit_msgs::MoveItErrorCodes code;
        moveit_msgs::RobotState robot_solution = goal->robot_state;
        kinematics::KinematicsQueryOptions opt;

        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        robot_state::RobotState r_state(kinematic_model);
    
        moveit::core::robotStateMsgToRobotState(robot_solution,r_state);

        const kinematics::KinematicsBaseConstPtr solver = r_state.getJointModelGroup("fanuc_m20ia")->getSolverInstance();

        for (int i = 0; i < ik_seed.size(); i++)
        {
            ROS_INFO_STREAM(ik_seed[i]);
        }
        
        bool end = false;

        int same_solution = 0;

        while (true)
        {
            // solver->getPositionIK(pose,ik_seed,solution,code);
            solver->searchPositionIK(pose,ik_seed,timeout,solution,code,opt);
            // && (same_solution == 0 || check_solution(all_solutions,solution)==true)
            if (code.val==code.SUCCESS && (all_solutions.size()==0 || check_solution(all_solutions,solution)==true))
            {
                ROS_INFO("NUOVA SOLUZIONE");

                all_solutions.push_back(solution);
                sendFeedback(solution,robot_solution);
                same_solution = 1;                
            }
            else
            {
                same_solution += 1;
            }
            
            if (same_solution == 1000 || all_solutions.size()==7)
            {
                break;
            }
            
        }        

        if (all_solutions.size()==7)
        {
            sendResult("Trovate tutte le possibili configurazioni");
        }
        else if (all_solutions.size()==0)
        {
            sendResult("Nessuna soluzione trovata");
        }
        
        {
            sendResult("Trovate solo alcune soluzioni");
        }
    }

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ik_fanuc_m20ia_server");
    ROS_INFO("Avvio server IK solver");
    IK_Solver action("ik_fanuc_m20ia");
    ros::spin();
    ros::shutdown();
    return 0;
}