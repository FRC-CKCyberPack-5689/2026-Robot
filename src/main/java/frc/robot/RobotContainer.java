// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleDrive;
import frc.robot.commands.TeleIntake;
import frc.robot.commands.TeleShooter;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Main components of the robot
    public static DriveTrain driveTrain;
    public static Intake intake;
    public static Shooter shooter;

    public static ADIS16470_IMU gyro;
    public static CommandXboxController controller;

    // Camera System
    public static PhotonCamera camera;

    // Initialize everything
    public RobotContainer() {
        gyro = new ADIS16470_IMU();
        controller = new CommandXboxController(RMap.OperatorConstants.kDRIVER_CONTROLLER_PORT);
        camera = new PhotonCamera(null);

        driveTrain = new DriveTrain();
        intake = new Intake();
        shooter = new Shooter();

        configureBindings();
    }

    // Map the controller to the functions of the robot
    private void configureBindings() {
        driveTrain.setDefaultCommand(new TeleDrive());
        controller.x().whileTrue(new TeleIntake());
        controller.y().whileTrue(new TeleShooter());
    }
}
