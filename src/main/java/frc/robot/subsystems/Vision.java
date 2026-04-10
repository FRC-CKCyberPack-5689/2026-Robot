// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RMap.VisionConstants;

public class Vision extends SubsystemBase {
	public Vision() {}
	
	/**
	 * Looks for a target within the camera's view.
	 * 
	 * @return Information aboutd a valid target.
	 */
	public PhotonTrackedTarget getTarget() {
		// Get the current alliance
		// When doing testing, make sure the alliance is set
		// correctly in the driver station!
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) {
			return null;
		}

		// Depending on the alliance, we aim for a different target
		int targetID = (alliance.get() == Alliance.Red) ? 10 : 26;

		// Check if our camera can see any targets
		PhotonPipelineResult result = RobotContainer.camera.getLatestResult();
		
		if (result.hasTargets()) {
			// Check each target to see if it matches
			// the one we are looking for.
			for (PhotonTrackedTarget target : result.getTargets()) {
				if (target.getFiducialId() == targetID) {
					return target;
				}
			}
		}

		return null;
	}

	/**
	 * Gets the estimated distance to a target in meters.
	 * 
	 * @param target The target to get a distance from.
	 * @return An estimated distance to it.
	 */
	public double getDistanceToTarget(PhotonTrackedTarget target) {
		// Get the current pitch of the target.
		double targetPitch = Units.degreesToRadians(target.getPitch());

		return PhotonUtils.calculateDistanceToTargetMeters(
			VisionConstants.kCAMERA_HEIGHT_METERS,
			VisionConstants.kTARGET_HEIGHT_METERS,
			VisionConstants.kCAMERA_PITCH_METERS,
			targetPitch
		);
	}

	/**
	 * If we're looking at a target from further away, we
	 * are no longer facing directly towards the center of the bin.
	 * This method gets a yaw that adjusts for that.
	 * 
	 * @param target The target to get a yaw from.
	 * @return An adjusted yaw in degrees.
	 */
	public double getAdjustedYaw(PhotonTrackedTarget target) {
		Transform3d cameraToTarget = target.getBestCameraToTarget();
		
		double adjustedYawRadians = Math.atan2(cameraToTarget.getY(), cameraToTarget.getX());
		return Math.toDegrees(adjustedYawRadians);
	}
}
