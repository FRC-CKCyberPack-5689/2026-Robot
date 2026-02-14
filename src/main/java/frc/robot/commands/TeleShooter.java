package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.MotorConstants;
import frc.robot.RobotContainer;

// This command handles the shooter during the tele-op phase
// of a game.
public class TeleShooter extends Command {
    // Main methods
    public TeleShooter() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.shooter.setIntakeSpeed(MotorConstants.kSHOOTER_INTAKE_SPEED);
        RobotContainer.shooter.setLauncherSpeed(MotorConstants.kSHOOTER_LAUNCHER_SPEED);
    }
}
