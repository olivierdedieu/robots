import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.*;
import lejos.nxt.*;

public class Odometry {

  public static final int AREA_WIDTH=100;
  public static final int AREA_LENGTH=100;
  public static final double TRACK_WIDTH = 15.5;
  public static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_NXT2;
  
  
  public static void main(String[] args) throws Exception {
    
    
    DifferentialPilot robot = new DifferentialPilot(WHEEL_SIZE, TRACK_WIDTH, Motor.B, Motor.C);
    
    OdometryPoseProvider pp = new OdometryPoseProvider(robot);
    
    robot.rotate(90);
    robot.travel(100);
    robot.arc(30, 90);
    robot.travel(50);
    
    System.out.println("End: " + pp.getPose());
    
    Button.ENTER.waitForPressAndRelease();    
  }
  
}
