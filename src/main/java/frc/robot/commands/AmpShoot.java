package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;

    public AmpShoot(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter);
        addRequirements(feeder);
    }

    public void initialize() {
    }

    public void execute() {
        shooter.shooterSpeedControl(0, ShooterConstants.kAmpUpperMotorSpeed, ShooterConstants.kShooterAmpSpeedLimit);
        feeder.feederSpeedControl(FeederConstants.kFeederSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shooterSpeedControl(0, 0, 0);
        feeder.feederSpeedControl(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
