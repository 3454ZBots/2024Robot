package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RabidRobot.Rabid;

public final class Main 
{
  private Main() {}

  public static void main(String... args) 
  {
   RobotBase.startRobot(Robot::new);
    // RobotBase.startRobot(Rabid::new);
  }
}
