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
        RobotContainer.intake.setIntakeSpeed(RMap.MotorConstants.kINTAKE_SPEED);
        RobotContainer.intake.setPivotCurrent(RMap.MotorConstants.kINTAKE_PIVOT_CURRENT);
    }
}
