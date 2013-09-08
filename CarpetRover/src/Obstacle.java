import lejos.robotics.RangeReadings;
import lejos.robotics.RotatingRangeScanner;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.*;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.nxt.*;

public class Obstacle implements FeatureListener {

  public static final int X_TARGET=200;
  public static final int Y_TARGET=100;
  public static final double TRACK_WIDTH = 15.5;
  public static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_NXT2;
  
  public static final int MAX_DISTRANCE = 15;
  
  public static final int DIRECTION_NONE  = 0;
  public static final int DIRECTION_RIGHT = 1;
  public static final int DIRECTION_LEFT  = 2;
  
  
  private static final float[] HEAD_ANGLES = {-70, -60, -50, -40, -30, 30, 40, 50, 60, 70};
  
  private static final float OBSTACLE_WHEEL_ANGLE_LEFT  = 60;
  private static final float OBSTACLE_WHEEL_ANGLE_RIGHT = -60;
  private static final int   OBSTACLE_WHEEL_FORWARD     = 30;
  private static final int   OBSTACLE_WHEEL_BACKWARD    = -10;
  
  public static void main(String[] args) throws Exception {
    
    
    Obstacle robot = new Obstacle();
    robot.go();

  }
  
  DifferentialPilot pilot;
  Navigator nav;
  OdometryPoseProvider pp;
  
  public Obstacle() {
    pilot = new DifferentialPilot(WHEEL_SIZE, TRACK_WIDTH, Motor.B, Motor.C);
    pilot.setRotateSpeed(50);
    nav = new Navigator(pilot);
    pp = new OdometryPoseProvider(pilot);
   
    UltrasonicSensor sensor = new  UltrasonicSensor(SensorPort.S4);
    FeatureDetector detector = new RangeFeatureDetector(sensor, MAX_DISTRANCE, 90);
    detector.addListener(this);
  }
  
  public void go() {
    nav.goTo(new Waypoint(X_TARGET, Y_TARGET));
    Button.ESCAPE.waitForPressAndRelease();    
  }
  
  @Override
  public void featureDetected(Feature feature, FeatureDetector detector) {
    Sound.beep();
    
    // Avoid the obstacle
    boolean canBeAvoided = avoidObstacle(detector);
    if (!canBeAvoided) {
      for(int i = 0; i < 3; i++) {
        Sound.beep();
      }
      return;
    }
    
    // Compute new Waypoint
    Pose pose = pp.getPose();
    pilot.rotate(-pose.getHeading());

    int xTarget = X_TARGET-(int)(pose.getX());
    int yTarget = Y_TARGET-(int)(pose.getY());
    Waypoint target = new Waypoint(xTarget, yTarget);
    
    System.out.println("x: " + pose.getX());
    System.out.println("y: " + pose.getY());
    System.out.println("xt: " + xTarget);
    System.out.println("yt: " + yTarget);
    //Button.ENTER.waitForPressAndRelease();    

    nav = new Navigator(pilot); // retry with nav.clearPath();
    nav.goTo(target);
  }

  private boolean avoidObstacle(FeatureDetector detector) {
    pilot.stop();
    
    detector.enableDetection(false);
    
    pilot.travel(OBSTACLE_WHEEL_BACKWARD);
    
    int direction = findDirection();
    float angle = 0;
    if (direction == DIRECTION_NONE) {
      return false;
    }

    
    if (direction == DIRECTION_RIGHT) {
      angle = OBSTACLE_WHEEL_ANGLE_RIGHT;
    }
    if (direction == DIRECTION_LEFT) {
      angle = OBSTACLE_WHEEL_ANGLE_LEFT;     
    }

    pilot.rotate(angle);
    pilot.travel(OBSTACLE_WHEEL_FORWARD);
    pilot.rotate(-angle);

    detector.enableDetection(true);
    
    return true;
  }
  
  
  private int findDirection() {
    UltrasonicSensor sensor = new  UltrasonicSensor(SensorPort.S4);


    
    RotatingRangeScanner rrs = new RotatingRangeScanner(Motor.A, sensor, 1);
    rrs.setAngles(HEAD_ANGLES);
    RangeReadings rr = rrs.getRangeValues();
    float leftRange = 0;
    float rightRange = 0;
    for(int i=0; i < HEAD_ANGLES.length; i++) {
      float angle = HEAD_ANGLES[i];
      float range = rr.getRange(angle);
      if (angle < 0) {
        if (range > leftRange) {
          leftRange = range;
        }
      } else {
        if (range > rightRange) {
          rightRange = range;
        }
      }
      
    }

    
    LCD.clear();
    LCD.drawString("left  : " + leftRange, 0, 5);
    LCD.drawString("right : " + rightRange, 0, 6);
    
    int direction =  DIRECTION_NONE;
    
    if (leftRange < 0 || leftRange > rightRange) {
      direction = DIRECTION_LEFT;
    }
    if (rightRange < 0 || rightRange >= leftRange) {
      direction =  DIRECTION_RIGHT;
    }
    
    LCD.drawString("direction : " + printDirection(direction), 0, 7);

    //Button.ENTER.waitForPressAndRelease();    

    return direction;
    
  }
  
  private String printDirection(int direction) {
    
    switch(direction) {
    case DIRECTION_LEFT:
        return "Left";
        
    case DIRECTION_RIGHT:
      return "Right";
    default:
      return "NONE";
    }
    
  }
  
}
