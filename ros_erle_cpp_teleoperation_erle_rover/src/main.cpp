#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>

class ErleRoverManager
{
public:
 
  ErleRoverManager();
  ~ErleRoverManager();
  bool init();

  int getLinearVelocity();
  int getAngularVelocity();

private:
  int vx, wz;

  int linear_vel_step, linear_vel_max, linear_vel_min;
  int angular_vel_step, angular_vel_max, angular_vel_min;
  std::string name;

  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
};

int ErleRoverManager::getLinearVelocity()
{
  return vx;
}

int ErleRoverManager::getAngularVelocity()
{
  return wz;
}

/**
 * @brief Default constructor, needs initialisation.
 */
ErleRoverManager::ErleRoverManager() :
                         linear_vel_step(2),
                         linear_vel_max(1560),
                         linear_vel_min(1440),
                         angular_vel_step(100),
                         angular_vel_max(1900),
                         angular_vel_min(1100),
                         quit_requested(false),
                         key_file_descriptor(0),
                         vx(0.0), wz(0.0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

ErleRoverManager::~ErleRoverManager()
{
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool ErleRoverManager::init()
{
  std::cout << "ErleRoverManager : using linear  vel step [" << linear_vel_step << "]." << std::endl;
  std::cout << "ErleRoverManager : using linear  vel max  [" << linear_vel_max << ", " << linear_vel_min << "]." << std::endl;
  std::cout << "ErleRoverManager : using angular vel step [" << angular_vel_step << "]." << std::endl;
  std::cout << "ErleRoverManager : using angular vel max  [" << angular_vel_max << ", " << angular_vel_min << "]." << std::endl;

  vx = 1500;
  wz = 1500;

  boost::thread t(boost::bind(&ErleRoverManager::keyboardInputLoop, this));
  return true;
}

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void ErleRoverManager::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }

  puts("Exit");
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void ErleRoverManager::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case 67:
    {
      incrementAngularVelocity();
      break;
    }
    case 68:
    {
      decrementAngularVelocity();
      break;
    }
    case 65:
    {
      incrementLinearVelocity();
      break;
    }
    case 66:
    {
      decrementLinearVelocity();
      break;
    }
    case 32:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    default:
    {
      break;
    }
  }
}


/**
 * @brief If not already maxxed, increment the command velocities..
 */
void ErleRoverManager::incrementLinearVelocity()
{
  if (vx <= linear_vel_max){
    vx += linear_vel_step;
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void ErleRoverManager::decrementLinearVelocity()
{
  if (vx >= linear_vel_min){
    vx -= linear_vel_step;
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void ErleRoverManager::incrementAngularVelocity()
{
  if (wz <= angular_vel_max){
    wz += angular_vel_step;
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void ErleRoverManager::decrementAngularVelocity()
{
  if (wz >= angular_vel_min){
    wz -= angular_vel_step;
  }
}

void ErleRoverManager::resetVelocity()
{
  vx = 1500;
  wz = 1500;
}

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  ros::init(argc, argv, "mavros_msgs_rc_override");
  ros::NodeHandle n;

  ErleRoverManager erlerover_manager;
  erlerover_manager.init();

  int rate = 20;
  ros::Rate r(rate);

  ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  mavros_msgs::OverrideRCIn msg_override;

  while (n.ok()){

        msg_override.channels[0] = erlerover_manager.getAngularVelocity();
        msg_override.channels[1] = 0;
        msg_override.channels[2] = erlerover_manager.getLinearVelocity();
        msg_override.channels[3] = 0;
        msg_override.channels[4] = 0;
        msg_override.channels[5] = 0;
        msg_override.channels[6] = 0;
        msg_override.channels[7] = 0;

        rc_override_pub.publish(msg_override);
//        ros::spinOnce();
        r.sleep();
  }

  return 0;
}
