package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
    private ShooterSubsystem shooter;

    public AmpShoot(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {
    }

    public void execute() {
        shooter.shooterSpeedControl(ShooterConstants.kAmpLowerMotorSpeed, ShooterConstants.kAmpUpperMotorSpeed,
                ShooterConstants.kShooterAmpSpeedLimit);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shooterSpeedControl(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
