import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RangeReadings;
import lejos.robotics.RotatingRangeScanner;

public class TestRotatingHead {

  
  public static void main(String[] args) throws Exception {
    
    
    UltrasonicSensor sensor = new  UltrasonicSensor(SensorPort.S4);

    float leftAngle = -60;
    float rightAngle = 60;
    
    RotatingRangeScanner rrs = new RotatingRangeScanner(Motor.A, sensor, 1);
    rrs.setAngles(new float[] {leftAngle, rightAngle});
    RangeReadings rr = rrs.getRangeValues();
    float leftRange = rr.getRange(0);
    float rightRange = rr.getRange(1);
    
    LCD.drawString("left  : " + leftRange, 0, 6);
    LCD.drawString("right : " + rightRange, 0, 7);
    Button.ESCAPE.waitForPressAndRelease();    
  }
  
}
