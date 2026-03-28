// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.VisionConstants;
import frc.robot.RobotContainer;

public class AssistedShoot extends Command {
	public AssistedShoot() {
		addRequirements(
			RobotContainer.driveTrain,
			RobotContainer.shooter,
			RobotContainer.vision
		);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Initialize some defaults
		double launcherSpeed = 0;
		double rotationSpeed = 0;

		// Get a target from the camera
		var target = RobotContainer.vision.getTarget();

		// Check if a valid target exists
		if (target != null) {
			// Get the distance to the target.
			double distance = RobotContainer.vision.getDistanceToTarget(target);

			// Ensure its within our shooting range.
			if (distance <= VisionConstants.kMaxShootDistance) {
				// Update our speeds to match the target 
				double adjustedYaw = RobotContainer.vision.getAdjustedYaw(target);
				rotationSpeed = RobotContainer.pidController.calculate(adjustedYaw, 0);
				launcherSpeed = VisionConstants.kSHOOTER_MAP.get(distance);
			}
		}

		RobotContainer.shooter.setLauncherSpeed(launcherSpeed);
		RobotContainer.driveTrain.drive(0, 0, rotationSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
