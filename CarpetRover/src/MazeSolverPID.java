import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.PIDController;


public class MazeSolverPID implements ButtonListener {
  
  private static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_NXT1;
  
  private static final double TRACK_WIDTH = 9.2; // cm
  
  
  private static final int TRAVEL_SPEED = 8;
  private static final int ROTATE_SPEED = 50;

  private static final int NO_WALL_DISTANCE = 40; // cm
    
  private static final double ROTATE_RADIUS = 15; // degree
  private static final int   SHIFT_DISTANCE = 8; // cm
  
  private static final int TURN_LEFT = 0;
  private static final int TURN_RIGHT = 1;

  private static float KP = 3.0f;
  private static float KI = 0.0001f; // 0.001f; 
  private static float KD = 3.0f; 
  
  private static int wallDistance = 15;    // cm - target distance 
  private static int baseSpeed    = 200;   // Target Power

  
  private DifferentialPilot pilot;
  private UltrasonicSensor frontSensor;
  private UltrasonicSensor wallSensor;
  private PIDController pid;
  private int lastWallDistance = -1;
  
  public static void main(String[] args ) {
    MazeSolverPID robot = new MazeSolverPID();
    robot.go();
  }
  
  
  public MazeSolverPID() {
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
    wallSensor = new  UltrasonicSensor(SensorPort.S4);
    frontSensor = new  UltrasonicSensor(SensorPort.S1);
  }
  
  private void initPID() {
    
    calibrate();
    resetPID();
  }
  
  private void resetPID() {
    
    pid = new PIDController(wallDistance, 10);
    pid.setPIDParam(PIDController.PID_KP, KP);
    pid.setPIDParam(PIDController.PID_KI, KI);
    pid.setPIDParam(PIDController.PID_KD, KD);
    //pid.setPIDParam(PIDController.PID_LIMITHIGH, Motor.B.getMaxSpeed()-baseSpeed);
    pid.setPIDParam(PIDController.PID_LIMITHIGH, 1 * baseSpeed);
    pid.setPIDParam(PIDController.PID_LIMITLOW, -baseSpeed);
  }
  
  
  public void go() {
    initPID();

    while(!Button.ESCAPE.isDown())  {
      int turnDir = followWall();
      turn(turnDir);
      resetPID();
    }

  }
  
  private int followWall() {
    
    
    while(true) {
      //pilot.forward();
      Motor.B.backward();
      Motor.C.backward();
      
      if (isWallOnFront()) {
        pilot.stop();
        return TURN_RIGHT;
      }
      

      int value = getWallDistance();
      if (value < 0) {
        continue;
      }

      if (value > NO_WALL_DISTANCE) {
        pilot.stop();
        return TURN_LEFT;
      }

      int turn = pid.doPID(value);
      updateMotorSpeed(turn);
      printInfo(value, turn);
    }
   
  }
  
  private void calibrate() {
    LCD.drawString("Calibrate distance", 0, 0);
    LCD.drawString("Press ENTER...",     0, 2);
    Button.waitForAnyPress();
    wallDistance = getWallDistance();
  }


  private int getWallDistance() {
    
    int value = wallSensor.getDistance();
    int distance = value;
    if (value == 255) {
      distance = lastWallDistance;
    }
    lastWallDistance = value;
    return distance;
  }
  

  private void updateMotorSpeed(int turn) {

    // Compute Speed
    int speedB = baseSpeed + turn; //the power level for the B motor
    int speedC = baseSpeed - turn; // the power level for the C motor
    Motor.B.setSpeed(speedB);
    Motor.C.setSpeed(speedC);
  }

  
  private void printInfo(int value, int turn) {

    LCD.clear();
    
    int line = 0;
    LCD.drawString("expected : " + wallDistance, 0, line++);
    LCD.drawString("value    : " + value, 0, line++);
    LCD.drawString("last val : " + lastWallDistance, 0, line++);
    LCD.drawString("turn     : " + turn,  0, line++);
  }

  


  private boolean isWallOnFront() {
    return frontSensor.getDistance() < wallDistance;
  }
  
  
  private void turn(int turnDir) {
    if (turnDir == TURN_LEFT) {
      turnLeft();
    } else {
      turnRight();
    }
  }
  
  private void turnRight() {
    pilot.travel(2);   // FIXME constants
    pilot.rotate(-90); // FIXME constants

  }

  
  private void turnLeft() {
    Sound.beep();
    Sound.beep();
    
    pause(200);
    
    pilot.travel(SHIFT_DISTANCE);
    pilot.arc(ROTATE_RADIUS, 70); // FIXME constants

    
    // Check if there is a wall
    boolean wallOnLeft = wallSensor.getDistance() < NO_WALL_DISTANCE;
    if (wallOnLeft) {
      pilot.travel(SHIFT_DISTANCE);
      return;
    }
    

    // It is a final wall, turn around
    pilot.arc(ROTATE_RADIUS, 70); // FIXME constants
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
