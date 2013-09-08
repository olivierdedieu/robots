import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.PIDController;

// http://www.users.humboldt.edu/aschmidt/linefollowv3.html
public class WallFollower implements ButtonListener {

  
  public static float KP = 3.0f;
  public static float KI = 0.0001f; // 0.001f; 
  public static float KD = 3.0f; 
  
  public static int offset     = 15;     // Distance to the wall
  public static int baseSpeed  = 100;   // Target Power
  
  private UltrasonicSensor sensor;
  private PIDController pid;
  
  public static void main(String[] args) throws Exception {
    WallFollower robot = new WallFollower();
    robot.go();
  }
  
  public WallFollower() {
    initButtons();
    initSensors();
    initMotors();
  }

  private void initMotors() {
    Motor.B.setAcceleration(500);
    Motor.C.setAcceleration(500);
    
    Motor.B.setSpeed(baseSpeed);
    Motor.C.setSpeed(baseSpeed);
  }

  private void initSensors() {
    sensor = new  UltrasonicSensor(SensorPort.S4);
  }

  private void initButtons() {
    Button.ESCAPE.addButtonListener(this);
  }
  
  private void initPID() {
    calibrate();

    pid = new PIDController(offset, 10);
    pid.setPIDParam(PIDController.PID_KP, KP);
    pid.setPIDParam(PIDController.PID_KI, KI);
    pid.setPIDParam(PIDController.PID_KD, KD);
    //pid.setPIDParam(PIDController.PID_LIMITHIGH, Motor.B.getMaxSpeed()-baseSpeed);
    pid.setPIDParam(PIDController.PID_LIMITHIGH, 1 * baseSpeed);
    pid.setPIDParam(PIDController.PID_LIMITLOW, -baseSpeed);
  }
  
  
  public void go() throws InterruptedException {
    
    initPID();

    while(true) {
      Motor.B.backward();
      Motor.C.backward();
      int value = getValue();
      int turn = pid.doPID(value);
      updateMotorSpeed(turn);
      printInfo(value, turn);
    }
  }
  
  private void calibrate() {
    LCD.drawString("Calibrate High", 0, 0);
    Button.waitForAnyPress();
    offset = getValue();
  }


  private int getValue() {
    return sensor.getDistance();
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
    LCD.drawString("value  : " + value,   0, line++);
    LCD.drawString("turn   : " + turn, 0, line++);
  }


  @Override
  public void buttonPressed(Button b) {

  }

  @Override
  public void buttonReleased(Button b) {
    System.exit(0);
  }

}