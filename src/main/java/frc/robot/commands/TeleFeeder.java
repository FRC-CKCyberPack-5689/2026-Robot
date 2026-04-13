package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.RobotContainer;

public class TeleFeeder extends Command {
    // Move the umjammer stuff into here eventually

    public TeleFeeder() {
        addRequirements(RobotContainer.shooter, RobotContainer.intake);
    }

    @Override
    public void execute() {
        RobotContainer.shooter.setAggravatorSpeed(-RMap.ShooterConstants.kAGGRAVATOR_SPEED);
        RobotContainer.shooter.setIntakeSpeed(-RMap.ShooterConstants.kINTAKE_SPEED);
        RobotContainer.intake.setIntakeSpeed(RMap.IntakeConstants.kINTAKE_SPEED);
        RobotContainer.intake.setArmPosition(false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.setAggravatorSpeed(0);
        RobotContainer.shooter.setIntakeSpeed(0);
        RobotContainer.intake.setIntakeSpeed(0);
    }

}
