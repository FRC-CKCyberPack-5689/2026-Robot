// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;

  public VisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  public PhotonTrackedTarget getTarget() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty())
      return null;

    int targetID = (alliance.get() == Alliance.Red) ? 10 : 26;
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      for (var target : result.getTargets()) {
        if (target.getFiducialId() == targetID)
          return target;
      }
    }
    return null;
  }

  public double getDistanceToTarget(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.kCameraHeightMeters,
        VisionConstants.kTargetHeightMeters,
        VisionConstants.kCameraPitchRadians,
        Units.degreesToRadians(target.getPitch()));
  }

  public double getAdjustedYaw(PhotonTrackedTarget target) {
    Transform3d cameraToTarget = target.getBestCameraToTarget();
    // Calculate angle to the actual center of the bin, not just the tag face
    return Math.toDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
