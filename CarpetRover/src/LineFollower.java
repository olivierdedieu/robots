import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;

// http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
public class LineFollower implements ButtonListener {

  
  public static double Kp   = 5;    // 10 // REMEMBER we are using Kp*100 so this is really 10 !
  public static double Ki   = 0;   // 1  // REMEMBER we are using Ki*100 so this is really 1 !
  public static double Kd   = 0 ;  // 100 // REMEMBER we are using Kd*100 so this is really 100!
  
  public static int offset  = 38;     // Initialize the variables
  public static int Tp      = 200;    // Target Power
  
  public static int MAX_SPEED = Tp + 2000;
  
  
  public static double dt = 0.1; 
  public static boolean resetIntegral = false;
  
  public static void main(String[] args) throws Exception {
    LineFollower robot = new LineFollower();
    robot.go();
  }
  
  ColorSensor sensor;
  int valueSum = 0;
  int errorSum = 0;
  int turnSum = 0;
  int count = 0;
  boolean run = true;
  
  public LineFollower() {
    Button.ENTER.addButtonListener(this);
    Button.ESCAPE.addButtonListener(this);

    sensor = new ColorSensor(SensorPort.S3);
    sensor.setFloodlight(true);
    
    Motor.B.setAcceleration(500);
    Motor.C.setAcceleration(500);

  }
  
  public void go() throws InterruptedException {
    double integral = 0;   // the place where we will store our integral
    int lastError = 0;  // the place where we will store the last error value
    double derivative = 0; // the place where we will store the derivative

    while(true) {
      while(run) {
        int value = sensor.getLightValue();

        // calculate the error by subtracting the offset
        int error = value - offset;

        // calculate the integral 
        integral = integral + error*dt;   

        // calculate the derivative
        derivative = (error - lastError)/dt;   

        // the "P term" the "I term" and the "D term"
        double turn = Kp*error + Ki*integral + Kd*derivative ;

        updateMotorSpeed(turn);

        // save the current error so it can be the lastError next time around
        lastError = error; 

        // Compute average
        count++;
        valueSum = value + valueSum;
        errorSum = error + errorSum;
        turnSum = (int)turn + turnSum;

        printInfo(value, error, turn, integral, derivative);


        if (resetIntegral && error < 2) {
          integral = 0;
        }


        Thread.sleep((int)(1000 * dt));
      }

      Motor.B.stop();
      Motor.C.stop();
    }
  }
  
  private void printInfo(int value, int error, double turn, double integral, double derivative) {
    int valueAverage = valueSum / count;
    int errorAverage = errorSum / count;
    int turnAverage = turnSum / count;
    
    LCD.clear();
    
    int line = 0;
    LCD.drawString("Light     : " + value,             0, line++);
    LCD.drawString("Light M   : " + valueAverage,      0, line++);
    LCD.drawString("Error     : " + error,             0, line++);
    LCD.drawString("Error M   : " + errorAverage,      0, line++);
    LCD.drawString("Turn      : " + ((int)turn),       0, line++);
    LCD.drawString("Turn  M   : " + turnAverage,       0, line++);
    LCD.drawString("Integral  : " + ((int)integral),   0, line++);
    LCD.drawString("Derivative: " + ((int)derivative), 0, line++);

  }

  private void updateMotorSpeed(double turn) {
  
  
    // Compute Speed
    int speedB = (int)(Tp + turn); //the power level for the B motor
    int speedC = (int)(Tp - turn); // the power level for the C motor

    updateMotorSpeed(Motor.B, speedB);
    updateMotorSpeed(Motor.C, speedC);
  }

  private void updateMotorSpeed(NXTRegulatedMotor motor, int speed) {
    
    
    int realSpeed = Math.abs(speed);
    if (realSpeed > MAX_SPEED) {
      realSpeed = MAX_SPEED;
    }
    motor.setSpeed(realSpeed);  

    if (speed > 0) {
      motor.forward();
    } else {
      motor.backward();
    }
  }
  
  
  

  @Override
  public void buttonPressed(Button b) {

  }

  @Override
  public void buttonReleased(Button b) {
    if (b == Button.ENTER) {
      run = false;
      Motor.B.stop();
      Motor.C.stop();
    } else {
      System.exit(0);
    }
  }

}
