

#include "ros/ros.h"
#include <cmath>
#include <time.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
 
#define NAME_OF_THIS_NODE "joystick_handler_node"

//-----------------------------------------------------------------

class Joystick_Handler
/* this class template includes all the elements needed to define
 * a basic ROS node */
{
  private: 
    ros::NodeHandle nh;
    
    // ros::Subscriber Subscriber;
    /*********************************************************
     * Uncomment if your node subscribes to a topic.
     *
     * Add one similar variable for each additional topic that
     * your node subscribes to.
     */
    
    // ros::Publisher Publisher;
    /*********************************************************
     * Uncomment if your node publishes messages to a topic.
     *
     * Add one similar variable declaration for each additional
     * topic that your node publishes on.  
     */
    
    // ros::ServiceClient Client;
    // name_of_the_server_package::name_of_the_srv_file Server;
    /*********************************************************
     * Uncomment if your node needs to act as a client of the ROS
     * server defined by the file name_of_the_srv_file.srv, defined
     * by the ROS package called name_of_the_server_package.
     *
     * Add one similar variable declaration for each additional
     * ROS server that your node needs to access as a client.
     */  
     
    // ros::ServiceServer Service;
    /*********************************************************
     * Uncomment if your node acts as a ROS server, providing
     * a service to clients.
     *
     * Add one similar variable declaration for each additional
     * ROS service that your node provides to clients.
     */  
    
     // ros::Timer TimeoutTimer;
    /*********************************************************
     * Uncomment if your node uses a timeout. A timeout ensures
     * that a specified method of the node is called if the node
     * does not receive messages on a given topic for a specified
     * interval of time. Such method, defined below, is called
     * TimeoutCallback.
     *
     * Add one similar variable declaration for each additional
     * timeout used by your node.
     */

    // param_type ParamVar;
    /*********************************************************
     * Uncomment if your node uses parameters that are stored
     * by the ROS parameter server. param_type is the data
     * type of the parameter, and ParamVar is a variable where
     * the value of the parameter will be stored when it is
     * retrieved from the parameter server.
     * Add one similar statement for each additional parameter
     * stored on the server.
     *
     * Allowed types for ROS parameters are those supported
     * by the YAML language, as described in
     * http://www.ros.org/wiki/rosparam
     * Note that, to define a parameter called my_param on the
     * ROS parameter server and assign value my_val to it, you
     * can use any of the following methods:
     * 1) using rosparam from the command line;
     * 2) putting a <param> statement into a launchfile,
     *    according to http://www.ros.org/wiki/roslaunch/XML;
     * 3) put the parameter definition (expressed using YAML
     *    syntax) into a text file, then loading it by putting
     *    into a launchfile a statement like
     *      <rosparam command="load" file="$(find my_pkg)/
     *       /path_within_pkg_directory/my_file.yaml" />
     *    where my_file.yaml is the YAML file and my_pkg is the
     *    package it belongs to.
     *
     * Please note that if the statement that defines a
     * parameter (either directly or by loading a .yaml file)
     * in a launchfile is included into a <node> block, the
     * parameter is defined in the namespace of the node: i.e.,
     * it is a private parameter of such node. This is the
     * preferred way to define parameters in ROS, because it
     * minimizes conflicts and "name hijacking".
     *
     * NOTE: variable ParamVar must be moved to the public section
     * of class ROSnode if you need to use its value from outside 
     * a class instance (e.g., from the main block).
     */
     
    // void MessageCallback(const msg_pkg::msg_type::ConstPtr& msg);
    /* ********************************************************
     * Uncomment if your node receives messages on a topic, and
     * you want an activity performed each time the node gets a
     * one such message. The activity is represented by method
     * MessageCallback.
     * msg_type must be substituted with the name of the type of 
     * message that your node receives, published on the topic
     * that your node subscribed to; msg_pkg must be substituted
     * with the name of the ROS package which contains the .msg
     * file defining such message type.
     * A package which defines many common ROS message types is
     * std_msgs: a list of the message types it provides is here:
     * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
     *
     * If your node receives messages from more than one 
     * topic, usually one separate MessageCallback method should
     * be defined for each topic. At the very minimum, one method
     * is required for each separate message type (in principle,
     * messages having the same type can use the same callback
     * method even if they are on different topics).
     */ 

    // void TimeoutCallback(const ros::TimerEvent& event);
    /*********************************************************
     * Uncomment if your node requires a timeout, i.e. if you
     * want method TimeoutCallback to get called whenever
     * the node is not active for a given period of time.
     *
     * If your node uses more than one timeout, usually one
     * separate TimeoutCallback method should be defined.
     */ 

    // void PeriodicTask(void);
    /*********************************************************
     * Uncomment if your node is run periodically and needs to
     * execute a task each time it is run. Such task is
     * represented by method PeriodicTask.
     */
     
    // void UseService(void);
    /*********************************************************
     * Uncomment if your node uses a service provided by a ROS 
     * server. Requests to the server are performed by method
     * UseService.
     *
     * You need a similar method for each service used by your
     * node.
     */
    
    // bool ProvideService(name_of_the_server_package::name_of_the_srv_file::Request  &Req,  name_of_the_server_package::name_of_the_srv_file::Response &Res);
    /*********************************************************
     * Uncomment if your node acts as the ROS server for the
     * service defined by the file name_of_the_srv_file.srv, 
     * which is part of the ROS package called 
     * name_of_the_server_package. The service is implemented by
     * method ProvideService.
     *
     * You need a similar method for each service provided by
     * your node.
     */
     
    // void ROSnode::StopTimeout(void);
    // void ROSnode::StartTimeout(void);
    /*********************************************************
     * Uncomment if your node requires a timeout. The first method
     * is used to stop the timeout (e.g., each time that execution
     * of a given task starts); the second method is used to
     * restart the timeout (e.g., when task execution ends).
     *
     * If your node uses more than one timeout, you can choose to
     * use a single couple of methods StopTimeout/StartTimeout
     * that affects all timeouts (in that case, remember to alter
     * the code of such methods accordingly), or to define
     * separate methods for each timeout.
	 
	  
     */
	Joystick_Handler();
  public:
    double RunPeriod;
    /* if the node is run periodically, this is the period -in 
     * seconds- between the start of one execution of the node 
     * and the start of the next one */
    
    void Prepare(void);
    /* prepares the node for running: all the preparatory tasks
     * that the node has to perform (both due to the interaction
     * with the ROS system and to your particular application)
     * should be performed by this method */
    
    void RunContinuously(void);
    /* used to run the node if it has to run continuously.
     * WARNING: this method and RunPeriodically should never be
     * both called for the same node. */
    
    void RunPeriodically(float Period);
    /* used to run the node if it has to run periodically; Period
     * is the distance in time between runs, in seconds.
     * WARNING: this method and RunContinuously should never be
     * both called for the same node. */
    
    void Shutdown(void);
    /* performs all the activities that the node is required to 
     * perform immediately before it shuts down
     * NOTE: all the elements of the node that are provided by ROS
     * do not require you to do anything to shut down a node; this 
     * method is provided as a container for code that your own
     * node may be required to run in order to shut down its
     * activities in a clean way. */
	 
	
};
Joystick_Handler::Joystick_Handler(){
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;

  std::string FullParamName;
  /* full (i.e. fully resolved starting from namespace /) name of
   * a parameter to be retrieved from the ROS parameter server. 
   * Actually used only if your node accesses the ROS parameter
   * server to retrieve parameter values (how to do this is
   * explained below). */
  
  /***********************************************************
   * RETRIEVING THE VALUE OF RunPeriod FROM THE PARAMETER SERVER
   * [a generic version of the following code, suitable for
   *  retrieving generic parameters, will be provided later]
   *
   * If your node runs periodically, you have to assign a 
   * value to variable RunPeriod. If you do nothing, RunPeriod
   * takes value RUN_PERIOD_DEFAULT, as defined at the beginning
   * of this file.
   * If you want to modify the value of RunPeriod, you can either
   * change RUN_PERIOD_DEFAULT (but you will have to recompile 
   * this file after the change), or you can adopt a more
   * sophisticated approach that does not require recompiling.
   * In fact, provided that you define parameter
   * 
   * /name_of_the_namespace/NAME_OF_THIS_NODE/run_period
   *
   * (of type double) in the ROS parameter server, and that you
   * assign the desired value to it, you can have the node read the
   * value of RunPeriod from the parameter server when it starts
   * running. You can enable such reading by uncommenting the
   * following piece of code:
   *
     FullParamName = ros::this_node::getName()+"/run_period";
      
     if (true == Handle.getParam(FullParamName, RunPeriod))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
   */


  // Subscriber = Handle.subscribe("name_of_the_topic", length_of_the_queue, &ROSnode::MessageCallback, this);
  /***********************************************************
   * Uncomment if your node subscribes to the topic called
   * name_of_the_topic. Substitute length_of_the_queue with an
   * integer defining how many messages the input queue should
   * be able to contain.
   * NOTE: if your node is run periodically, make sure that the
   * queue is long enough to avoid that useful, but older, 
   * messages are discarded by ROS because the queue is full.
   * 
   * One such instruction is required for each topic that the node
   * subscribes to.
   */
  
  // Publisher = Handle.advertise<msg_pkg::msg_type>("name_of_the_topic", length_of_the_queue);
  /***********************************************************
   * Uncomment if your node publishes to the topic called
   * name_of_the_topic. Substitute length_of_the_queue with an
   * integer defining how many messages the output queue should
   * be able to contain.
   * msg_type must be substituted with the name of the type of 
   * message that your node will publish on the topic; msg_pkg 
   * must be substituted with the name of the ROS package which
   * contains the .msg file defining such message type.
   * A package which defines many common ROS message types is
   * std_msgs: a list of the message types it provides is here:
   * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
   *
   * One such instruction is required for each topic that the node
   * publishes to.
   */
   
  // Client = Handle.serviceClient<name_of_server_package::name_of_the_srv_file>("name_of_the_service");
  /***********************************************************
   * Uncomment if your node needs to act as a client of a service
   * called name_of_the_service, provided by a ROS server defined
   * by file name_of_the_srv_file.srv which is part of the ROS
   * package called name_of_the_server_package.
   *
   * Add one similar statement for each additional ROS server
   * that your node needs to access as a client.
   */
   
  // Service = Handle.advertiseService("name_of_the_service", &ROSnode::ProvideService, this);
  //ROS_INFO("ROS service %s available (provided by node %s).", "name_of_the_service", ros::this_node::getName().c_str());
  /***********************************************************
   * Uncomment the first statement if your node acts as a ROS
   * server, providing a service called name_of_the_service
   * to clients. The service is implemented by method
   * ProvideService.
   * Also uncomment the second statement if you want to highlight
   * the availability of the service, for instance for debugging
   * purposes.
   *
   * Add similar statements for each additional ROS service that
   * your node provides.
   */
   
  // TimeoutTimer = Handle.createTimer(ros::Duration(duration_of_the_timeout), &ROSnode::TimeoutCallback, this, put_here_true_or_false);
  /***********************************************************
   * Uncomment if your node requires a timeout. Substitute 
   * duration_of_the_timeout with the required value, in seconds,
   * expressed as a float (e.g., 14.0).
   * put_here_true_or_false should be substituted with true or
   * false. In the first case, after the timeout expires method
   * TimeoutCallback will be called only once ("once-only"
   * timeout). In the second case, once the timeout expires,
   * TimeoutCallback is called periodically, with period equal to
   * duration_of_the_timeout.
   * This statement also starts the timeout.
   *
   * You need one such instruction for each TimeoutTimer
   * attribute that you defined above.
   */
  
  /***********************************************************
   * RETRIEVING PARAMETER VALUES FROM THE ROS PARAMETER SERVER
   *
   * Uncomment the following piece of code if you want to
   * retrieve a parameter named my_param from the ROS parameter
   * server, and store its value in variable ParamVar (which
   * you need to have declared as a member variable of class
   * ROSNode; you should choose whether to make ParamVar a public
   * or private variable depending on who needs to access it).
   *
     // FullParamName = ros::this_node::getName()+"/my_param";
     //uncomment this if my_param is a private parameter of the
     //node, i.e. if its full name is 
     // /name_of_the_namespace/NAME_OF_THIS_NODE/my_param
     
     // FullParamName = Handle.getNamespace()+"my_param";
     //uncomment this if, instead, my_param is a global parameter
     //of the namespace that the node belongs to, i.e. if its
     //full name is /name_of_the_namespace/my_param
 
     if (true == Handle.getParam(FullParamName, ParamVar))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
   *
   * You need one piece of code like this for each parameter
   * value that your node needs to retrieve from the ROS parameter
   * server.
   */
   
   ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunContinuously(void)
{
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
  
  /* From ROS documentation:
   * "ros::spin() will not return until the node has been 
   * shutdown, either through a call to ros::shutdown() or a 
   * Ctrl-C." */
}


void ROSnode::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  while (ros::ok())
  {
    //PeriodicTask();
    /*********************************************************
     * Uncomment if you have defined a periodic task that the node
     * has to execute every time it is run. If you leave this
     * commented out, every time the node is run it only processes
     * callbacks.
     */
     
    ros::spinOnce();
    /* From ROS documentation:
     * "ros::spinOnce() will call all the callbacks waiting to be
     * called at that point in time. ." */
    
    LoopRate.sleep();
  }
}


void ROSnode::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
  /* 
   * ROS does not require you to do anything to shut down cleanly
   * your node. However, if your node needs to do do something
   * before shutting down, put the relevant code here.
   */
}


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::MessageCallback(const msg_pkg::msg_type::ConstPtr& msg)
{
  StopTimeout();
  The preceding statement blocks the timeout every time the 
  message callback is executed. Remove it if you don't want that
  to happen, or if your node does not use a timeout. 
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StopTimeout method that
  needs to be called.
  
  ...put here your code for the management of the message
  
  StartTimeout();
  The preceding statement restarts the timeout when the execution 
  of the message callback ends. Delete it if you don't want that
  to happen, or if your node does not use a timeout.
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StartTimeout method that
  needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::TimeoutCallback(const ros::TimerEvent& event)
{
  StopTimeout();
  Delete the preceding statement if your node does not use a
  timeout, or add a similar statement for each additional
  StopTimeout method that needs to be called.
  
  ...put here your code for the management of the timeout event
  
  StartTimeout();
  Delete the preceding statement if your node does not use a
  timeout, or add a similar statement for each additional
  StartTimeout method that needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::PeriodicTask(void)
{
  StopTimeout();
  The preceding statement blocks the timeout every time the 
  periodic task is executed. Delete it if you don't want that to
  happen, or if your node does not use a timeout. 
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StopTimeout method that
  needs to be called.
  
  ...put here your code for the management of the periodic task
  
  StartTimeout();
  The preceding statement restarts the timeout when the execution 
  of the periodic task ends. Delete it if you don't want that to
  happen, or if your node does not use a timeout.
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StartTimeout method that
  needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::UseService(void)
{
  Server.request.name_of_input = expression of appropriate type;
  
  where name_of_input must be substituted with the correct name of
  the input field, as defined by the .srv file of the server.
  
  if (Client.call(Server))
  {
    variable of appropriate type = Server.response.name_of_output;
    
    where name_of_output must be substituted with the correct name
    of the output field, as defined by the .srv file of the server.
  }
  else
  {
    Put here the code to manage the "server not responding" 
    condition. 
    Please note that if the server node is active but is not
    running at the time of the request by the client because it
    runs periodically and is currently sleeping, this is NOT a
    "server not responding" condition. The request of the client
    is queued by ROS, and the client will get the response at
    the next run of the server.
  }
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
bool ROSnode::ProvideService(name_of_the_server_package::name_of_the_srv_file::Request  &Req,  name_of_the_server_package::name_of_the_srv_file::Response &Res)
{
  suppose that the .srv file defines two input fields called in1
  and in2, and two output fields called out1 and out2. Then you
  have to write something like:
    
  Res.out1 = expression using Req.in1 and Req.in2;
  Res.out2 = expression using Req.in1 and Req.in2;
}
 *
 */


/*************************************************************
 * Uncomment the following blocks if you defined the corresponding
 * methods of ROSnode.
 *
void ROSnode::StopTimeout(void)
{
  TimeoutTimer.stop();
  ...add here similar statements to stop other timers, if needed
}

void ROSnode::StartTimeout(void)
{
  TimeoutTimer.setPeriod(ros::Duration(duration_of_the_timeout));
  ...substitute duration_of_the_timeout with the required value, in
  seconds, expressed as a float (e.g., 14.0)  
  TimeoutTimer.start();
  
  ...add here similar statements to start other timers, if needed
}
 *
 */


//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  /* NOTE: the call to ros::init should be the FIRST statement of
   * the 'main' block. You get compilation errors if you use any
   * part of the ROS system before that statement. */

  /* NOTE. If this node is launched with rosrun, a new node
   * gets added to the running ROS system, the name of which is
   * the string assigned to NAME_OF_THIS_NODE. If the node is
   * launched with roslaunch, it is possible that this choice of
   * name has been overridden, and that the newly added node takes
   * different name than NAME_OF_THIS_NODE. This happens when the
   * <node> statement in the launchfile used to launch the node
   * specifies a name for the node (which is not mandatory). */
  
  ROSnode MyNode;
   
  MyNode.Prepare();
  
  // MyNode.RunContinuously();
  // MyNode.RunPeriodically(MyNode.RunPeriod);
  /*
   * Uncomment ONE AND ONLY ONE of the above statements.
   */ 
   
  MyNode.Shutdown();
  
  return (0);
}


/*************************************************************
 * NOTE. If you want this file to be compiled and the
 * corresponding executable to be generated, remember to add a
 * suitable rosbuild_add_executable line to the CMakeLists.txt
 * file of your package. [Warning: the contents of this note is
 * valid for versions of ROS that use the (older) rosbuild
 * build system. They may be obsolete if your version of ROS is,
 * instead, based on the newer catkin build system.]
 *************************************************************/
