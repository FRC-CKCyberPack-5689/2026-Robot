package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.RobotContainer;

// This command handles the intake during the tele-op phase
// of a game.
public class TeleIntake extends Command {
    // Main methods
    public TeleIntake() {
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Runs constantly while the button is pressed on the controller
        //RobotContainer.intake.setIntakeSpeed(RobotContainer.intake.intakeSpeed.getDouble(0.8));
        RobotContainer.intake.setIntakeSpeed(-RMap.IntakeConstants.kINTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.setIntakeSpeed(0);
    }
}