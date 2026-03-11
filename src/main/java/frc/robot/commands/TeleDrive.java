package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.RobotContainer;

// Note:
// If the robot has a "head" it will drive from its own perspective
// If the robot is "headless" it will drive from the driver's perspective

// This command handles movement during the tele-op phase
// of a game.
public class TeleDrive extends Command {
    private double forwardSpeed;
    private double strafeSpeed;
    private double rotationSpeed;
    private double desiredHeading;

    // Main methods
    public TeleDrive() {
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {
        forwardSpeed = 0;
        strafeSpeed = 0;
        rotationSpeed = 0;

        // Stop the robot from "jumping" if it is
        // rotated before it starts up.
        desiredHeading = RobotContainer.driveTrain.getHeading();
    }

    @Override
    public void execute() {
        // Update the speeds based on the controller
        forwardSpeed = calculateDriveAxis(-RobotContainer.controller.getLeftY(), forwardSpeed);
        strafeSpeed = calculateDriveAxis(RobotContainer.controller.getLeftX(), strafeSpeed);
        rotationSpeed = calculateDriveTheta(RobotContainer.controller.getRightX());

        // Drive!
        RobotContainer.driveTrain.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    /**
     * Calculates a ramped and clipped motor speed from
     * the controller's stick input.
     * 
     * @param input Controller stick input
     * @param currentSpeed Current axis speed of robot
     * @return Ramped and clipped motor speed
     */
    public static double calculateDriveAxis(double input, double currentSpeed) {
        double i = Math.abs(input);
        double c = Math.abs(currentSpeed);

        // Stop joystick drift
        i = MathUtil.applyDeadband(i, RMap.DriveConstants.kDEADBAND);

        // If our input is greater than our current speed, we should accelerate.
        // Otherwise, we should decelerate
        if (i != 0) {
            if (i > c) {
                // Accelerate
                i = Math.min(i, c + RMap.DriveConstants.kAXIS_ACCELERATION);
            } else {
                // Decelerate
                i = Math.max(i, c + RMap.DriveConstants.kAXIS_DECELERATION);
            }
        }

        // Custom speed control
        i *= RMap.DriveConstants.kAXIS_SPEED;

        // Make i take the sign(+/-) of input
        // This allows the code to work the same
        // for both positive and negative inputs
        i = Math.copySign(i, input);

        return i;
    }

    /**
     * Calculates a ramped and clipped motor speed from
     * the controller's stick input. This is literally
     * a mirror of the function above, but using
     * rotation constants instead.
     * 
     * @param input Controller stick input
     * @param currentSpeed Current rotation speed of robot
     * @return Ramped and clipped motor speed
     */
    public static double calculateRotationAxis(double input, double currentSpeed) {
        double i = Math.abs(input);
        double c = Math.abs(currentSpeed);

        // Stop joystick drift
        i = MathUtil.applyDeadband(i, RMap.DriveConstants.kDEADBAND);

        // If our input is greater than our current speed, we should accelerate.
        // Otherwise, we should decelerate
        if (i != 0) {
            if (i > c) {
                // Accelerate
                i = Math.min(i, c + RMap.DriveConstants.kROTATION_ACCELERATION);
            } else {
                // Decelerate
                i = Math.max(i, c + RMap.DriveConstants.kROTATION_DECELERATION);
            }
        }

        // Custom speed control
        i *= RMap.DriveConstants.kROTATION_SPEED;

        // Make i take the sign(+/-) of input
        // This allows the code to work the same
        // for both positive and negative inputs
        i = Math.copySign(i, input);

        return i;
    }

    /**
     * Calculates a ramped and clipped rotation speed from
     * the controller's stick input.
     * 
     * @param input Controller stick input
     * @return A ramped and clipped rotation speed
     */
    public double calculateDriveTheta(double input) {
        double rampedMotorDrive = calculateRotationAxis(input, rotationSpeed);
        double newHeading = RobotContainer.driveTrain.getHeading();

        // Check if the user is actively using stick
        if (Math.abs(rampedMotorDrive) > 0.0) {
            // User IS turning, update the heading
            desiredHeading = newHeading;
            return rampedMotorDrive;
        } else {
            // User IS NOT actively turning, maintain the desired rotation
            // Apply a correction based on how far we are from it
            // kP is how "quickly" we apply the correction
            double kP = 0.01;
            double error = kP * (desiredHeading - newHeading);
            error = MathUtil.applyDeadband(error, RMap.DriveConstants.kDEADBAND);
            return error;
        }
    }
}
