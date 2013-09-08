import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;


public class MazeSolver implements ButtonListener {
  
  public static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_NXT1;
  
  public static final double TRACK_WIDTH = 9.2; // cm
  
  
  public static final int TRAVEL_SPEED = 8;
  public static final int ROTATE_SPEED = 50;

  public static final int FRONT_DISTANCE = 10; // cm
  public static final int WALL_MAX_DISTANCE = 30; // cm
  public static final int WALL_MIN_DISTANCE = 8; // cm

  public static final double ROTATE_RADIUS = 15; // degree
  public static final int SHIFT_DISTANCE = 5; // cm
  public static final int TURNAROUND_DISTANCE = 8; // cm
  
  public static final int TURN_LEFT = 0;
  public static final int TURN_RIGHT = 1;
  
  private DifferentialPilot pilot;
  private UltrasonicSensor frontSensor;
  private UltrasonicSensor wallSensor;

  
  public static void main(String[] args ) {
    MazeSolver robot = new MazeSolver();
    robot.go();
  }
  
  
  public MazeSolver() {
    initButtons();
    initPilot();
    initSensors();
  }
  
  private void initButtons() {
    Button.ESCAPE.addButtonListener(this);
  }
  
  private void initPilot() {
    pilot = new DifferentialPilot(WHEEL_SIZE, TRACK_WIDTH, Motor.B, Motor.C, true);
    pilot.setRotateSpeed(ROTATE_SPEED); 
    pilot.setTravelSpeed(TRAVEL_SPEED);
  }
  
  private void initSensors() {
    wallSensor = new  UltrasonicSensor(SensorPort.S1);
    frontSensor = new  UltrasonicSensor(SensorPort.S4);
  }
  
  public void go() {
    Sound.beep(); //we are starting

    while(!Button.ESCAPE.isDown())  {
      int turnDir = followWall();
      turn(turnDir);
    }

  }
  
  private int followWall() {
    // Run along the wall until no wall or a front wall 
    while(true) {

      
      /*
       * FIXME it does not work
      if (isWallToNear()) {
        pilot.stop();        
        Sound.beep();Sound.beep();Sound.beep();
        restoreWallDistance();
      }
      */

      
       if (!isWallOnLeft()) {
        pilot.stop();
        return TURN_LEFT;
      }
      

     if (isWallOnFront()) {
        pilot.stop();
        return TURN_RIGHT;
      }
      
      pilot.forward();
    }
   
  }
  
  private boolean isWallToNear() {
    if (wallSensor.getDistance() < WALL_MIN_DISTANCE) {
      return true;
    }
    pause(100);
    return wallSensor.getDistance() < WALL_MIN_DISTANCE;
  }
  
  private boolean isWallOnLeft() {
    if (wallSensor.getDistance() < WALL_MAX_DISTANCE) {
      return true;
    }
    pause(100);
    return wallSensor.getDistance() < WALL_MAX_DISTANCE;
  }
  
  private boolean isWallOnFront() {
    return frontSensor.getDistance() < FRONT_DISTANCE;
  }
  
  private void restoreWallDistance() {
    while(isWallToNear()) {
      pilot.rotateRight();
    }
  }
  
  private void turn(int turnDir) {
    if (turnDir == TURN_LEFT) {
      turnLeft();
    } else {
      turnRight();
    }
  }
  
  private void turnRight() {
    pilot.travel(2);
    pilot.rotate(-90);

  }

  
  private void turnLeft() {
    Sound.beep();
    Sound.beep();
    
    pause(200);
    
    pilot.travel(SHIFT_DISTANCE);
    pilot.arc(ROTATE_RADIUS, 90);

    //pilot.rotate(90);
    //pilot.travel(TURNAROUND_DISTANCE);
    
    // Check if there is a wall
    boolean wallOnLeft = wallSensor.getDistance() < WALL_MAX_DISTANCE;
    if (wallOnLeft) {
      pilot.travel(SHIFT_DISTANCE);
      return;
    }
    
    Sound.beep();

    // It is a final wall, turnaround
    pilot.arc(ROTATE_RADIUS, 90);
    pilot.travel(SHIFT_DISTANCE);

  }
  

  private void pause(int time)  {
    try { Thread.sleep(time); } catch(InterruptedException e){}
  }
  
  @Override
  public void buttonPressed(Button b) {
    // Empty
  }

  @Override
  public void buttonReleased(Button b) {
    System.exit(0);
  }
  
}
