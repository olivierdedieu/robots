import lejos.robotics.navigation.*;
import lejos.nxt.*;

public class Rambler {

  public static final int AREA_WIDTH=100;
  public static final int AREA_LENGTH=100;
  public static final double TRACK_WIDTH = 15.5;
  public static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_NXT2;
  
  
  public static void main(String[] args) throws Exception {
    
    
    DifferentialPilot pilot = new DifferentialPilot(WHEEL_SIZE, TRACK_WIDTH, Motor.B, Motor.C);
    
    Navigator nav = new Navigator(pilot);
    
    while(!Button.ESCAPE.isDown()) {
      System.out.println("target: ");
      double x = (int)(Math.random() * AREA_WIDTH);
      double y = (int)(Math.random() * AREA_LENGTH);
      
      System.out.println("X: " + x);
      System.out.println("Y: " + y);
      System.out.println("Press ENTER key");
      
      Button.ENTER.waitForPressAndRelease();
      
      nav.goTo(new Waypoint(x, y));
      Thread.sleep(1000);
      nav.goTo(new Waypoint(0, 0, 0));
    }
    
  }
  
}
