// File:          MyAssignmentController.java
// Date:
// Description:
// Author:
// Modifications:Lizhenghe_Chen_201521681

// ==============================================================
// COMP329 2021 Programming Assignment
// ==============================================================
// 
// The aim of the assignment is to move the robot around the arena in such a way
// as to generate an occupancy grid map of the arena itself.  Full details can be
// found on CANVAS for COMP329
//
// Only add code to the controller file - do not modify the other java files in this project.
// You can add code (such as constants, instance variables, initialisation etc) anywhere in 
// the file, but the navigation itself that occurs in the main loop shoudl be done after checking
// the current pose, and having updated the two displays.
//
// Note that the size of the occup[ancy grid can be changed (see below) as well as the update
// frequency of the map, adn whether or not a map is generated.  Changing these values may be
// useful during the debugging phase, but ensure that the solution you submit generates an
// occupancy grid map of size 100x100 cells (with a recommended update frquency of 2).
//
// ==============================================================
import java.util.*;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;

public class MyAssignmentController {

  // ---------------------------------------------------------------------------
  // Dimensions of the Robot
  // ---------------------------------------------------------------------------
  // Note that the dimensions of the robot are not strictly circular, as
  // according to the data sheet the length is 485mm, and width is 381mm
  // so we assume for now the aprox average of the two (i.e. 430mm)
  private final static double ROBOT_RADIUS = 0.215; // in meters
  public static double wheelRadius = 0.0975; // in meters
  public static double axelLength = 0.31; // Distance (in m) between the two wheels
  public static int MAX_NUM_SENSORS = 16; // Number of sensors on the robot

  // ---------------------------------------------------------------------------
  // Assignment Parameters
  // ---------------------------------------------------------------------------
  // Note that ideally the number of cells in the occupancy grid should be a
  // multiple of the
  // display size (which is 500x500). So smaller values such as 50x50 or 25x25
  // could be used
  // to initialise a map with fewer, but larger grid cells
  private final static int NUMBER_OF_ROWCELLS = 100; // How many cells across the occupancy grid
  private final static int NUMBER_OF_COLCELLS = 100; // How many cells down the occupancy grid

  // This is the frequency that the map is updated (i.e. the map is updated every
  // GRID_UPDATE_FREQUENCY
  // times the loop is iterated. Increasing it may make the simulator run faster,
  // but fewer samples
  // will be taken
  private final static int GRID_UPDATE_FREQUENCY = 2; // How frequently do we sample the world

  // This boolean switches on (or off) the generation of the occupancy grid. It
  // may be useful to
  // make this false whilst working on the navigation code to speed things up, but
  // any final solution
  // should verify that a valid occupancy grid map is generated.
  private final static boolean GENERATE_OCCUPANCY_GRID = true;

  // ---------------------------------------------------------------------------
  // Robot instance
  // ---------------------------------------------------------------------------
  public static Supervisor robot;
  public static Node robotNode;

  // ==================================================================================
  // Static Methods
  // ==================================================================================
  // getLocalisedPos()
  // returns the real position of the robot without the need for localisation
  // through
  // particle filters etc. The supervisor mode is used to facilitate this.
  public static Pose getLocalisedPos() {
    double[] realPos = robotNode.getPosition();
    double[] rot = robotNode.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(rot[2], rot[8]);
    double halfPi = Math.PI / 2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
      theta2 = -(3 * halfPi) + theta1;

    return new Pose(realPos[0], -realPos[2], theta2);
  }

  // ==================================================================================
  // Main Methods
  // ==================================================================================
  /* Notice that I canged some parameters to static: timeStep ps */
  // --- need to input initialise manually by user -------
  static Orientation orientation = Orientation.EAST;// it depends on the orientation of robot at beginning
  static MoveState state = MoveState.FORWARD;
  static int max_speed = 12;// make sure no more than 12.3m/s
  static Pose endpoint = new Pose(2.1, 2.1, 0);// robot's end point

  // -----------!!!I changed these two to static!!!-------
  static int timeStep;
  static DistanceSensor[] ps;

  // ---------Similar as Lab2, but static instead--------------
  // values will be set in command();
  static boolean inMotion = false;
  static int timeElapsed;
  static double linearVelocity;
  static double av = (linearVelocity / wheelRadius);;
  static double targetDist;
  static double targetTime;
  static Motor leftMotor;
  static Motor rightMotor;
  static int turnCounter = 0;// for debug and use to judge weather robot at beginning or not
  static double initial_distance = 0;// for FOWARD PID speed use

  static int judger = 0;// for extra function judgeScan() use

  public enum MoveState { // all possible robot's states
    STOP, ROTATE_AROUND_STOP, FORWARD, TURN_RIGHT_90, TURN_LEFT_90, ROTATE_AROUND, TURN_LITTLE_RIGHT, TURN_LITTLE_LEFT,
    AWAY_FROM_LEFT, AWAY_FROM_RIGHT
  };

  public enum Orientation {// all robot's possible orientations
    NORTH, SOUTH, EAST, WEST
  };

  public static void main(String[] args) {
    // Define Robot Parameters
    long loopCounter = 0; // Used to count the number of main loop iterations

    // ---------------------------------------------------------------------------
    // create the Supervised Robot instance.
    // ---------------------------------------------------------------------------
    robot = new Supervisor();
    robotNode = robot.getSelf(); // Get a handle to the Robot Node in supervisor mode

    // get the time step of the current world.
    timeStep = (int) Math.round(robot.getBasicTimeStep());

    // ---------------------------------------------------------------------------
    // Set up motor devices
    // ---------------------------------------------------------------------------
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");

    // Set the target positions of the motors
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Initialise motor velocity
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);

    // ---------------------------------------------------------------------------
    // set up proximity detectors
    // ---------------------------------------------------------------------------
    ps = new DistanceSensor[MAX_NUM_SENSORS];
    String[] psNames = {
        "so0", "so1", "so2", "so3", "so4", "so5", "so6", "so7",
        "so8", "so9", "so10", "so11", "so12", "so13", "so14", "so15",
    };

    // The following array determines the orientation of each sensor, based on the
    // details of the Pioneer Robot Stat sheet. Note that the positions may be
    // slightly
    // inaccurate as the pioneer is not perfectly round. Also these values are in
    // degrees
    // and so may require converting to radians. Finally, we assume that the front
    // of the
    // robot is between so3 and so4. As the angle between these is 20 deg, we assume
    // that
    // they are 10 deg each from the robot heading
    double[] psAngleDeg = { 90, 50, 30, 10, -10, -30, -50, -90,
        -90, -130, -150, -170, 170, 150, 130, 90 };

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }

    // ---------------------------------------------------------------------------
    // Set up occupancy grid
    // ---------------------------------------------------------------------------
    OccupancyGrid grid; // Instantiate to generate an occupancy grid
    if (GENERATE_OCCUPANCY_GRID == true) {
      grid = new OccupancyGrid(5.0, 5.0, // Size of the arena
          NUMBER_OF_ROWCELLS, // Number of cells along the x-axis
          NUMBER_OF_COLCELLS, // Number of cells along the y-axis
          ps, // Array of distance sensors
          psAngleDeg, // Orientation of each sensor (array)
          ROBOT_RADIUS); // Radius of the robot body (assumes cylindrical)
    } else {
      grid = null; // No occupancy grid will be generated
    }

    // ---------------------------------------------------------------------------
    // Set up display devices
    // ---------------------------------------------------------------------------
    // The sensor view from the Labs is included to assist in debugging
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
    SensorView sensorView = new SensorView(sensorDisplay, ps, psAngleDeg, ROBOT_RADIUS);

    // A variant of the Arena view is used to show the robot position in a map.
    // The current display is configured as a 500x500 display attached to the robot.
    Display occupancyGridDisplay = robot.getDisplay("occupancyGridDisplay");
    ArenaView gridView = new ArenaView(occupancyGridDisplay, getLocalisedPos(), grid, 5.0, 5.0, ROBOT_RADIUS);

    // ---------------------------------------------------------------------------
    // Main loop:
    // ---------------------------------------------------------------------------
    // perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {

      // ---------------------------------------------------------------------------
      // Get current pose of the robot
      // ---------------------------------------------------------------------------
      Pose p = getLocalisedPos();

      // ---------------------------------------------------------------------------
      // Update the grid map and arena display
      // ---------------------------------------------------------------------------
      if (loopCounter++ % GRID_UPDATE_FREQUENCY == 0) {
        if (GENERATE_OCCUPANCY_GRID == true) {
          grid.occupancy_grid_mapping(p);
        }
        gridView.setPose(p);
        gridView.paintView();
      }

      // ---------------------------------------------------------------------------
      // Update the sensor display
      // ---------------------------------------------------------------------------
      sensorView.setPose(p);
      sensorView.paintView();

      // ==============================================================================
      // Move robot - Assignemnt
      // ==============================================================================
      // Assignment Solution Here

      if (turnCounter == 0) {// at the beginning
        System.out.println("Start toward: " + orientation);
        state = judgeLeftRight(state, p);// judge robot where to go
      }

      if (state == MoveState.FORWARD) { // when the robot is at FOWARD state,
        state = keepOrientation(state, p); // make sure it is straight
      }

      if (forward_distance(ps) < 0.2 && state == MoveState.FORWARD) {// when robot's front almost hit the wall
        state = MoveState.STOP;
      } else if (state == MoveState.STOP) {// decide where to go
        if (judgeEnd(state, p)) {
          // judge the location is at the end, if ture, then stop robot and exit while
          // loop
          break;
        } else
          state = judgeLeftRight(state, p);
      }

      if ((leftward_distance(ps) + rightward_distance(ps)) / 2 < 0.25 && judger < 1) {
        // judger can make sure only scan n times until next turn.
        state = MoveState.ROTATE_AROUND_STOP;
      } else if (state == MoveState.ROTATE_AROUND_STOP) {
        state = judgeScan(state);
      }

      switch (state) {
        case FORWARD:
          // System.out.println("FORWARD");
          state = commands(MoveState.FORWARD);
          break;
        case TURN_RIGHT_90:// call turn right 90 degree method
          state = commands(MoveState.TURN_RIGHT_90);
          break;
        case TURN_LEFT_90:// call turn left 90 degree method
          state = commands(MoveState.TURN_LEFT_90);
          break;
        case ROTATE_AROUND:
          // System.out.println("ROTATE_AROUND");
          state = commands(MoveState.ROTATE_AROUND);
          break;
        case TURN_LITTLE_RIGHT:// call turn right a bit method
          // System.out.println("TURN_LITTLE_RIGHT");
          state = commands(MoveState.TURN_LITTLE_RIGHT);
          break;
        case TURN_LITTLE_LEFT:// call turn left a bit method
          // System.out.println("TURN_LITTLE_LEFT");
          state = commands(MoveState.TURN_LITTLE_LEFT);
          break;
        case AWAY_FROM_LEFT:// call turn left a bit to aovid hit the all
          // System.out.println("TURN_LITTLE_LEFT");
          state = commands(MoveState.AWAY_FROM_LEFT);
          break;
        case AWAY_FROM_RIGHT:
          // System.out.println("AWAY_FROM_RIGHT");
          state = commands(MoveState.AWAY_FROM_RIGHT);
          break;
        case ROTATE_AROUND_STOP:
          // System.out.println("STOP");
          targetDist = 0.0;
          targetTime = 0.0;
          Left_Right_Controller(0, 0, leftMotor, rightMotor);
          timeElapsed = 0; // reset time elapsed
          inMotion = false; // not in motion now
          break;
        case STOP:
          // reset the robot's parameters used when moving and stop the wheels
          System.out.println("STOP");
          targetDist = 0.0;
          targetTime = 0.0;
          Left_Right_Controller(0, 0, leftMotor, rightMotor);
          timeElapsed = 0;
          inMotion = false;
          break;
      }
      // System.out.println(state);
    }
    ;
    // Enter here exit cleanup code.
  }

  /*
   * Below are my functions that can help me control the robots,
   * also it could make codes clean, easy to fix and reusable
   */

  private static MoveState judgeScan(MoveState state) {
    /*
     * this function is extra, can help reduce noise when robot in a narrow place,
     */
    state = MoveState.ROTATE_AROUND;
    judger++;
    System.out.println("Scan");
    return state;
  }

  public static void Left_Right_Controller(double leftSpeed, double rightSpeed, Motor leftMotor, Motor rightMotor) {
    // this function simplify the Motor command
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
  }

  static ArrayList<Pose> Turn_List = new ArrayList<>();// to store the turned points
  static ArrayList<Pose> judgeList = new ArrayList<>();// the list to store points that were be consider to turn
  static double max_Manhattan_Distance = 0;
  static Pose max_dublicate_Pose;// the final point taht will be consider to turn another side

  public static MoveState judgeLeftRight(MoveState state, Pose p) {
    judger = 0;
    // format the data:
    double x = Double.parseDouble(String.format("%.1f", p.getX()));
    double y = Double.parseDouble(String.format("%.1f", p.getY()));
    double t = Double.parseDouble(String.format("%.1f", p.getTheta()));

    // get currenct left and right distances
    double leftd = leftward_distance(ps);
    double rightd = rightward_distance(ps);

    if (leftd > rightd) {// if leftside is bigger than rightside, turn left
      state = MoveState.TURN_LEFT_90;
      orientation = judgeOrientation(orientation, -1);
      initial_distance = leftd;

    } else { // else turn right
      state = MoveState.TURN_RIGHT_90;
      orientation = judgeOrientation(orientation, 1);
      initial_distance = rightd;
    }
    // below is the method to avoid dead loop:
    Pose testp = new Pose(x, y, t);
    if (isDuplicate(testp, Turn_List)) {// if duplicate
      if (leftd > 0.5 && rightd > 0.5) {// if duplicate and also have chance to turn both left and right
        // caculate it's Manhattan_Distance between end point
        double Manhattan_Distance = Math.abs(testp.getX() - endpoint.getX()) + Math.abs(testp.getY() - endpoint.getY());
        if (Manhattan_Distance >= max_Manhattan_Distance) {// find biggest distance
          max_Manhattan_Distance = Manhattan_Distance;
          max_dublicate_Pose = testp;
        }
        if (testp == max_dublicate_Pose && isDuplicate(testp, judgeList)) {
          System.out.println("\n**Reah the candidate location, Turn another way!**\n");
          if (state == MoveState.TURN_LEFT_90) {// turn to another side, also adjust the orientation state
            orientation = judgeOrientation(orientation, 1);
            orientation = judgeOrientation(orientation, 1);
            state = MoveState.TURN_RIGHT_90;
          } else {
            orientation = judgeOrientation(orientation, -1);
            orientation = judgeOrientation(orientation, -1);
            state = MoveState.TURN_LEFT_90;
          }
          // after turn to another side, reset data below
          judgeList.clear();
          max_Manhattan_Distance = 0;
          max_dublicate_Pose = new Pose(endpoint);
        }
        // if duplicate and also have chance to turn both left and right, store to
        // judgeList
        judgeList.add(testp);
        System.out.println("Manhattan_Distance: " + Manhattan_Distance + ", Candidate_Pose: " + max_dublicate_Pose);
      }
    } else {// if duplicate, store it to Turn_List
      Turn_List.add(testp);
    }
    turnCounter++;
    System.out.println("Toward " + orientation + ", initial distance: " + String.format("%.1f", initial_distance)
        + ", turnCounter:" + turnCounter + ", turn data:" + Turn_List);
    return state;
  }

  public static boolean isDuplicate(Pose newp, ArrayList<Pose> List) {
    for (Pose temp : List) {
      // if current location is very close to previous location
      boolean inX = (temp.getX() >= newp.getX() - 0.2 && temp.getX() <= newp.getX() + 0.2);
      boolean inY = (temp.getY() >= newp.getY() - 0.2 && temp.getY() <= newp.getY() + 0.2);
      if (inX && inY) {
        System.out.println("found a duplicate point: " + temp);
        return true;
      }
    }
    return false;
  }

  public static boolean judgeEnd(MoveState state, Pose p) {
    // if current location is very close to End location
    boolean inX = (p.getX() >= endpoint.getX() - 0.2 && p.getX() <= endpoint.getX() + 0.2);
    boolean inY = (p.getY() >= endpoint.getY() - 0.2 && p.getY() <= endpoint.getY() + 0.2);
    if (inX && inY) {
      targetDist = 0.0;
      targetTime = 0.0;
      Left_Right_Controller(0, 0, leftMotor, rightMotor);
      timeElapsed = 0;
      inMotion = false;

      System.out.println("Mapping END!");
      return true;
    }
    return false;
  }

  public static Orientation judgeOrientation(Orientation orientation, int turnValue) {
    // 1 means turn right vertically,-1 means turn left vertically
    if (orientation == Orientation.EAST && turnValue == 1) {
      orientation = Orientation.SOUTH;
    } else if (orientation == Orientation.EAST && turnValue == -1) {
      orientation = Orientation.NORTH;
    } else if (orientation == Orientation.NORTH && turnValue == 1) {
      orientation = Orientation.EAST;
    } else if (orientation == Orientation.NORTH && turnValue == -1) {
      orientation = Orientation.WEST;
    } else if (orientation == Orientation.WEST && turnValue == 1) {
      orientation = Orientation.NORTH;
    } else if (orientation == Orientation.WEST && turnValue == -1) {
      orientation = Orientation.SOUTH;
    } else if (orientation == Orientation.SOUTH && turnValue == 1) {
      orientation = Orientation.WEST;
    } else if (orientation == Orientation.SOUTH && turnValue == -1) {
      orientation = Orientation.EAST;
    }
    return orientation;
  }

  public static MoveState keepOrientation(MoveState state, Pose p) {
    // this method will help robots not to hit the wall or yaw;
    double domainl = 0;// left and right sensor's value
    double domainr = 0;
    double distance = 0.15;// the distance shoud keep between wall, usually smaller than face wall stop
                           // distance
    if (leftward_distance(ps) <= distance || frountLeft_distance(ps) <= distance) {
      // System.out.println("AWAY_FROM_LEFT");
      return MoveState.AWAY_FROM_LEFT;
    }
    if (rightward_distance(ps) <= distance || frontRight_distance(ps) <= distance) {
      // System.out.println("AWAY_FROM_RIGHT");
      return MoveState.AWAY_FROM_RIGHT;
    }

    switch (orientation) {
      case EAST:// Esat means between 0π +or- 0.03 dgree, 0.03 is a Threshold range
        domainl = 0.03;
        domainr = -0.03;
        break;
      case NORTH:// North means if between π/2 +- 0.03 dgree
        domainl = Math.PI / 2 + 0.03;
        domainr = Math.PI / 2 - 0.03;
        break;
      case SOUTH:
        domainl = -Math.PI / 2 + 0.03;
        domainr = -Math.PI / 2 - 0.03;
        break;
      case WEST:
        domainl = -Math.PI + 0.03;
        domainr = Math.PI - 0.03;
        break;
    }
    // below keep robot not exceed the domain
    if (orientation == Orientation.WEST) {
      // Notice that for West, it can be represent as both π and -π! need to judge
      // individually
      if (p.getTheta() > domainl && p.getTheta() < 0) {
        // System.out.println("turn right a bit" + domainl + ", " + domainr);
        state = MoveState.TURN_LITTLE_RIGHT;
      }
      if (p.getTheta() < domainr && p.getTheta() >= 0) {
        // System.out.println("turn left a bit" + domainl + ", " + domainr);
        state = MoveState.TURN_LITTLE_LEFT;
      }

    } else { // the orentations except WEST.
      if (p.getTheta() > domainl) {
        // System.out.println("turn right a bit" + domainl + ", " + domainr);
        state = MoveState.TURN_LITTLE_RIGHT;
      }
      if (p.getTheta() < domainr) {
        // System.out.println("turn left a bit" + domainl + ", " + domainr);
        state = MoveState.TURN_LITTLE_LEFT;
      }
    }
    return state;
  }

  public static MoveState commands(MoveState state) {
    double left = 0;// set some positive or negative value to control the wheels direction
    double right = 0;
    switch (state) {
      case FORWARD:// y=√x and x=current distance between front wall/initial distance
        av = Math
            .sqrt(Math.pow(max_speed, 2) * Double.parseDouble(String.format("%.2f", forward_distance(ps)))
                / initial_distance);
        if (av >= max_speed) {// make sure speed between from 5~12 m/s
          av = max_speed;
        } else if (av < 5) {
          av = 5;
        }
        Left_Right_Controller(av, av, leftMotor, rightMotor);
        inMotion = true; // note that we are now in motion
        return state = MoveState.FORWARD;// jump out command();
      case TURN_LITTLE_LEFT:
        // Start rotating by some small degrees (i.e. 1/180 of a circle)
        targetDist = axelLength * Math.PI / 180;
        left = -1;
        right = 1;
        linearVelocity = 0.1;
        break;
      case TURN_LITTLE_RIGHT:
        // Start rotating by 3 degrees (i.e. 1/180 of a circle)
        targetDist = axelLength * Math.PI / 180;
        left = 1;
        right = -1;
        linearVelocity = 0.1;
        break;
      case AWAY_FROM_LEFT:
        // yaw by some distance to right, the distance could not caculate precisely
        targetDist = axelLength * Math.PI / 20;
        left = 1;
        right = 0.3;
        linearVelocity = 0.5;
        break;
      case AWAY_FROM_RIGHT:
        // yaw by some distance to left, the distance could not caculate precisely
        targetDist = axelLength * Math.PI / 20;
        left = 0.3;
        right = 1;
        linearVelocity = 0.5;
        break;
      case ROTATE_AROUND:
        // Start rotating by 360 degrees (i.e. 1/1 of a circle)
        // System.out.println("need ROTATE_AROUND");
        targetDist = axelLength * Math.PI;
        left = 1;
        right = -1;
        linearVelocity = 0.5;
        break;
      case TURN_RIGHT_90:
        // Start rotating by 360+90 degrees (i.e. 1 + 1/4 of a circle)
        targetDist = axelLength * Math.PI * 5 / 4;
        left = 1;
        right = -1;
        linearVelocity = 0.5;
        break;
      case TURN_LEFT_90:
        // Start rotating by 360+90 degrees (i.e. 1 + 1/4 of a circle)
        targetDist = axelLength * Math.PI * 5 / 4;
        left = -1;
        right = 1;
        linearVelocity = 0.5;
        break;
      case STOP:
        break;
    }

    av = (linearVelocity / wheelRadius);
    if (inMotion == true) {
      // We have already started moving
      if (timeElapsed > targetTime) {
        // need to stop
        // System.out.println(timeElapsed+","+targetTime);
        // System.out.println("need to STOP");
        Left_Right_Controller(0, 0, leftMotor, rightMotor);
        inMotion = false; // we are no longer in motion
        state = MoveState.FORWARD; // change state
      } else {
        timeElapsed += timeStep; // Increment by the time state
      }
    } else {
      timeElapsed = 0; // reset time elapsed
      inMotion = true; // now in motion
      targetTime = 1000.0 * (targetDist / linearVelocity);
      Left_Right_Controller(av * left, av * right, leftMotor, rightMotor);
    }
    return state;
  }

  public static double getDistance(DistanceSensor psA, DistanceSensor psB) {
    // we can know the elements 5.0 and 1024 from: ps[0].getLookupTable();
    // this method will return and caculate the averange distance from two sensors
    double A = 5.0 - (5.0 / 1024.0 * psA.getValue());
    double B = 5.0 - (5.0 / 1024.0 * psB.getValue());
    return (A + B) / 2;
  }

  /* return distance below */
  public static double forward_distance(DistanceSensor[] ps) {
    return getDistance(ps[3], ps[4]);
  }

  public static double leftward_distance(DistanceSensor[] ps) {
    return getDistance(ps[0], ps[15]);
  }

  public static double rightward_distance(DistanceSensor[] ps) {
    return getDistance(ps[7], ps[8]);
  }

  public static double frontRight_distance(DistanceSensor[] ps) {
    return getDistance(ps[5], ps[6]);
  }

  public static double frountLeft_distance(DistanceSensor[] ps) {
    return getDistance(ps[1], ps[2]);
  }

}
