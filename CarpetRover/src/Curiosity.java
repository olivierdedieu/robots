import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;


public class Curiosity  implements ButtonListener {
  
  private static final double WHEEL_SIZE = DifferentialPilot.WHEEL_SIZE_RCX;
  
  private static final double TRACK_WIDTH = 15; // cm

  
  private static final int TRAVEL_SPEED = 50;
  private static final int ROTATE_SPEED = 100;
  
  private static final float OBSTACLE_DISTANCE     = 20;  // cm
  private static final float OBSTACLE_DISTANCE_OK  = 20;  // cm
  private static final int   OBSTACLE_BACKWARD     = -15; // cm
  private static final int   OBSTACLE_ANGLE        = 360;  // degree
  private static final int   INCLINAISON_BACKWARD  = -60;
  private DifferentialPilot pilot;
  private UltrasonicSensor frontSensor;
  private ColorSensor colorSensor;
  private int prevBrightness = 0;
  private long lastCheckTime = 0;
  
  public static void main(String[] args ) {
    Curiosity robot = new Curiosity();
    robot.go();
  }
  
  
  
  public Curiosity() {
    initButtons();
    initPilot();
    initSensors();
  }
  
  private void initButtons() {
    Button.ESCAPE.addButtonListener(this);
  }
  
  private void initPilot() {
    pilot = new DifferentialPilot(WHEEL_SIZE, TRACK_WIDTH, Motor.B, Motor.C, true);
    pilot.setTravelSpeed(TRAVEL_SPEED);
    pilot.setRotateSpeed(ROTATE_SPEED); 
  }
  
  private void initSensors() {
    frontSensor = new  UltrasonicSensor(SensorPort.S4);
    colorSensor = new  ColorSensor(SensorPort.S3);
  }
  
  public void go() {
    while(!Button.ESCAPE.isDown()) {
      if (tilt()) {
        backward();
      }
      /*
      if (obstacle()) {
        avoidObstacle();
      }
      */
      pilot.forward();
    }
  }
  
  private boolean tilt() {

    long dt = System.currentTimeMillis() - lastCheckTime;

    int curr = colorSensor.getLightValue();
    
    LCD.clear();   
    LCD.drawString("prev : " + prevBrightness, 0, 0);
    LCD.drawString("curr : " + curr, 0, 1);
    LCD.drawString("dt   : " + dt, 0, 2);

    boolean tilt = curr > 50 && prevBrightness > 50;
    if (dt > 80) {
      prevBrightness = curr;
      lastCheckTime = System.currentTimeMillis();
    }
    return tilt;
  }
  
  private void backward() {
    pilot.stop();
    Sound.beep();
    if (!tilt()) {
      return;
    }
    Sound.beep();
    pilot.travel(INCLINAISON_BACKWARD); 
    pilot.rotate(90);
  }

  private void avoidObstacle() {
    pilot.stop();
    Sound.beep();
    pilot.travel(OBSTACLE_BACKWARD); 
    pilot.stop();
    int distance;
    do {
      distance = frontSensor.getDistance();
      pilot.rotate(OBSTACLE_ANGLE);
    }
    while(distance < OBSTACLE_DISTANCE_OK);
    
    pilot.stop();
  }

  private boolean obstacle() {
    return frontSensor.getDistance() < OBSTACLE_DISTANCE;
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
