package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RMap.DriveConstants;

/*
 * This command handles tele-operated movement
 * during the match.
 * 
 * Note: This works in "head" mode. That means it will drive
 * from the robot's perspective, not the driver's.
 */
public class TeleDrive extends Command {
    private double currentForwardSpeed;
    private double currentStrafeSpeed;
    private double currentRotationSpeed;

    public TeleDrive() {
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {
        // Reset speeds when this command is
        // scheduled to prevent issues.
        currentForwardSpeed = 0;
        currentStrafeSpeed = 0;
        currentRotationSpeed = 0;
    }

    @Override
    public void execute() {
        // Read inputs from the controller
        // The forward input has to e inverted since -Y is forwards
        double forwardInput = -RobotContainer.controller.getLeftY();
        double strafeInput = RobotContainer.controller.getLeftX();
        double rotationInput = RobotContainer.controller.getRightX();

        // Drive!
        drive(forwardInput, strafeInput, rotationInput);
    }

    /**
     * Drives the robot using inputs from a controller.
     * 
     * @param forwardInput The forward input of the controller.
     * @param strafeInput The strafe input of the controller.
     * @param rotationInput The rotation input of the controller.
     */
    public void drive(double forwardInput, double strafeInput, double rotationInput) {
        // Apply deadbanding
        forwardInput = MathUtil.applyDeadband(forwardInput, DriveConstants.kDEADBAND);
        strafeInput = MathUtil.applyDeadband(strafeInput, DriveConstants.kDEADBAND);
        rotationInput = MathUtil.applyDeadband(rotationInput, DriveConstants.kDEADBAND);

        // Smooth the inputs
        double newForwardSpeed = rampInput(forwardInput, currentForwardSpeed);
        double newStrafeSpeed = rampInput(strafeInput, currentStrafeSpeed);
        double newRotationSpeed = rampInput(rotationInput, currentRotationSpeed);

        // Update our current speeds
        currentForwardSpeed = newForwardSpeed;
        currentStrafeSpeed = newStrafeSpeed;
        currentRotationSpeed = newRotationSpeed;

        // Pass the speeds into the drive train.
        RobotContainer.driveTrain.drive(
            currentForwardSpeed, 
            currentStrafeSpeed, 
            currentRotationSpeed
        );
    }

    /**
     * Smoothly transition between a new speed
     * and a current speed, limiting the rate we 
     * accelerate or decelerate towards it.
     * 
     * @param target The target speed.
     * @param current The current speed.
     * @return A ramped speed between the two.
     */
    public static double rampInput(double target, double current) {
        // The maximum amount we can accelerate or decelerate
        double limit;

        // Determine if we are accelerating or decelerating
        boolean shouldAccelerate = Math.abs(target) > Math.abs(current);
        if (shouldAccelerate) {
            limit = DriveConstants.kMAX_ACCELERATION;
        } else {
            limit = DriveConstants.kMAX_DECELERATION;
        }

        // Get the difference between our current
        // speed and our target speed.
        double delta = target - current;

        // Limit the rate we can accelerate or decelerate
        boolean acceleratingTooFast = delta > limit;
        boolean deceleratingTooFast = delta < -limit;

        if (acceleratingTooFast) {
            // Limit our acceleration
            return current + limit;
        } else if (deceleratingTooFast) {
            // Limit our deceleration
            return current - limit;
        } else {
            // Everything is fine
            // Just output the target speed
            return target;
        }
    }
}
