// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.ShooterConstants;
import frc.robot.RMap.VisionConstants;
import frc.robot.RMap;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AssistedShoot extends Command {
	// Same thing as with TeleShooter
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

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Initialize some defaults
		double launcherSpeed = 0;
		double rotationSpeed = 0;

		// Get a target from the camera
		PhotonTrackedTarget target = RobotContainer.vision.getTarget();

		// Check if a valid target exists
		if (target != null) {
			// Get the distance to the target.
			double distance = RobotContainer.vision.getDistanceToTarget(target);

			System.out.println("Distance to target: " + distance);

			// Ensure its within our shooting range.
			if (distance <= VisionConstants.kMaxShootDistance) {
				// Update our speeds to match the target 8
				double adjustedYaw = RobotContainer.vision.getAdjustedYaw(target);
				rotationSpeed = RobotContainer.pidController.calculate(adjustedYaw, 0);
				launcherSpeed = VisionConstants.kSHOOTER_MAP.get(distance);

				System.out.println("Rotation speed: " + rotationSpeed);
				System.out.println("Launcher speed: " + launcherSpeed);
			}
		} else {
			System.out.println("No target");
		}
		
		RobotContainer.shooter.setLauncherSpeed(-launcherSpeed);
		RobotContainer.shooter.setAggravatorSpeed(-RMap.ShooterConstants.kAGGRAVATOR_SPEED);
		RobotContainer.driveTrain.drive(0, 0, rotationSpeed);

        // Delay the shooter's intake
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - startTime;

        if (deltaTime > ShooterConstants.kINTAKE_DELAY) {
            RobotContainer.shooter.setIntakeSpeed(-ShooterConstants.kINTAKE_SPEED);
        }
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
