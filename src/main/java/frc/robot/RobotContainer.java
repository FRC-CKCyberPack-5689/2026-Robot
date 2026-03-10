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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Main components of the robot
    public static DriveTrain driveTrain;
    public static Intake intake;
    public static Shooter shooter;

    // Other devices
    public static ADIS16470_IMU gyro;
    public static CommandXboxController controller;
    public static PhotonCamera camera;

    // UI
    public static Shuffleboard shuffleboard;

    // Initialize everything
    public RobotContainer() {
        driveTrain = new DriveTrain();
        intake = new Intake();
        shooter = new Shooter();

        gyro = new ADIS16470_IMU();
        controller = new CommandXboxController(RMap.OperatorConstants.kDRIVER_CONTROLLER_PORT);
        camera = new PhotonCamera(null);

        configureBindings();
    }

    // Map the controller to the functions of the robot
    private void configureBindings() {
        driveTrain.setDefaultCommand(new TeleDrive());

        // -----------------------------|
        // Configure the bindings here! |
        // -----------------------------|

        // Shooter
        controller.y().whileTrue(new TeleShooter());
        
        // Intake Motor
        controller.x().whileTrue(new TeleIntake());

        // Intake Arm: Pos A
        controller.a().onTrue(new InstantCommand(() -> {
            intake.setArmPosition(10);
        }));

        // Intake Arm: Pos B
        controller.b().onTrue(new InstantCommand(() -> {
            intake.setArmPosition(0);
        }));
    }
}
