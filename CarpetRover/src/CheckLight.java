import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;

public class CheckLight implements ButtonListener {

  public static void main(String[] args) throws Exception {
    CheckLight listener = new CheckLight();
    Button.ENTER.addButtonListener(listener);
    
    ColorSensor sensor = new ColorSensor(SensorPort.S3);
    sensor.setFloodlight(true);
    printLightValue(sensor);
  }
  
  
  public static void printLightValue(ColorSensor sensor) throws InterruptedException {
    int i = 0;
    while(true) {
      int value = sensor.getLightValue();
      System.out.println(i++ + ". Light: " + value);
      Thread.sleep(500);
    }
  }

  @Override
  public void buttonPressed(Button b) {

  }

  @Override
  public void buttonReleased(Button b) {
    System.exit(0);
  }

}
