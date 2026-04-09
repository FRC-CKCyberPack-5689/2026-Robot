// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.ShooterConstants;
import frc.robot.RMap.VisionConstants;
import frc.robot.RobotContainer;

public class AssistedShoot extends Command {
	private double currentForwardSpeed;
    private double currentStrafeSpeed;
	private double startTime;

	public AssistedShoot() {
		addRequirements(
			RobotContainer.driveTrain,
			RobotContainer.shooter,
			RobotContainer.vision
		);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		// Initialize some defaults
		double launcherSpeed = 0;
		double rotationSpeed = 0;

		// Get a target from the camera
		PhotonTrackedTarget target = RobotContainer.vision.getTarget();

		// Check if a valid target exists
		if (target != null) {
			// If it does exist, start aiming towards the target
			// Delay the aggravator and intake to allow the launcher to spin up
			double currentTime = Timer.getFPGATimestamp();
			double deltaTime = currentTime - startTime;

			if (deltaTime > ShooterConstants.kASSISTED_DELAY) {
				RobotContainer.shooter.setAggravatorSpeed(ShooterConstants.kAGGRAVATOR_SPEED);
				RobotContainer.shooter.setIntakeSpeed(ShooterConstants.kINTAKE_SPEED);
			}

			// Get the distance to the target.
			double distance = RobotContainer.vision.getDistanceToTarget(target);

			// Ensure its within our shooting range.
			if (distance <= VisionConstants.kMAX_SHOOT_DISTANCE) {
				// Update our speeds to match the target
				double adjustedYaw = RobotContainer.vision.getAdjustedYaw(target);
				rotationSpeed = RobotContainer.pidController.calculate(adjustedYaw, 0);
				launcherSpeed = VisionConstants.kSHOOTER_MAP.get(distance);
			}
		}

		// Handle the launcher motor
		RobotContainer.shooter.setLauncherSpeed(launcherSpeed);

		// Handle driving
		// Get inputs from the controller
		double forwardInput = -RobotContainer.controller.getLeftY();
        double strafeInput = RobotContainer.controller.getLeftX();

		// Smooth the inputs
        double newForwardSpeed = TeleDrive.rampInput(forwardInput, currentForwardSpeed);
        double newStrafeSpeed = TeleDrive.rampInput(strafeInput, currentStrafeSpeed);

        // Update our current speeds
        currentForwardSpeed = newForwardSpeed;
        currentStrafeSpeed = newStrafeSpeed;

        // Pass the speeds into the drive train
		// Use the rotation speed that matches the target
        RobotContainer.driveTrain.drive(
            currentForwardSpeed, 
            currentStrafeSpeed, 
            rotationSpeed
        );
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.shooter.setIntakeSpeed(0);
        RobotContainer.shooter.setLauncherSpeed(0);
        RobotContainer.shooter.setAggravatorSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
