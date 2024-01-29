package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;

public class Helpers {
    public static boolean IsBlackout() {
        return RobotController.getComments().equals("blackout");
    }

    public static String GetRobotName() {
        return RobotController.getComments();
    }
}