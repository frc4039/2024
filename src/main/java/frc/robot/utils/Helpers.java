package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;

public class Helpers {
    public static boolean isBlackout() {
        return RobotController.getComments().equals("blackout");
    }

    public static String getRobotName() {
        return RobotController.getComments();
    }
}