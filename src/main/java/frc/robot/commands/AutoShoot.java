package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoShoot extends Command {
    private double forwardSpeed;
    private double strafeSpeed;
    public AutoShoot() {
        addRequirements(RobotContainer.shooter, RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {
        forwardSpeed = 0;
        strafeSpeed = 0;
    }

    @Override
    public void execute() {
        //forwardSpeed = TeleDrive.calculateDriveAxis(-RobotContainer.controller.getLeftY(), forwardSpeed);
        //strafeSpeed = TeleDrive.calculateDriveAxis(RobotContainer.controller.getLeftX(), strafeSpeed);

        // Turn towards the target
        double kP = 0.01;
        double error = kP * RobotContainer.shooter.getTargetYaw();
        RobotContainer.driveTrain.drive(forwardSpeed, strafeSpeed, error);
    }
}