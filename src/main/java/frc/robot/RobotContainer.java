// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final Joystick driver = new Joystick(0);
  private final Trigger driverLeftTrigger = new Trigger (() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) >0.1);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final JoystickButton driverYButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driverAButton = new JoystickButton(driver, XboxController.Button.kA.value);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverLeftTrigger.whileTrue(new ShootCommand(() -> driver.getRawAxis (XboxController.Axis.kLeftTrigger.value), shooter));
    //driverYButton.onTrue()
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
