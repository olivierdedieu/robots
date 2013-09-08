import lejos.nxt.*;

public class Twirler implements ButtonListener {

  public static void main(String[] args) throws Exception {
   
    while(true)  {
      float speed1 = (float)  (Math.random()  *800);
      float speed2 = (float)  (Math.random()  *800);
      Motor.B.setSpeed(speed1);
      Motor.C.setSpeed(speed2);
      if (Math.random() < 0.5)  {
        Motor.C.forward();

      }else {
        Motor.C.backward();
      }
      Thread.sleep(5000) ;
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