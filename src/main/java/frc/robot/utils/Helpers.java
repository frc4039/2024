package frc.robot.utils;
import edu.wpi.first.wpilibj.RobotController;

public class Helpers
{
    public static boolean IsBlackout()
    {
        return RobotController.getComments() == "blackout";
    }

    public static String GetRobotName()
    {
        return RobotController.getComments();
    }
}