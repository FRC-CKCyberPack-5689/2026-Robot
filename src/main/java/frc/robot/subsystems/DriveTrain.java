// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import frc.robot.RMap.MotorConstants;
import frc.robot.RMap.DriveConstants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
    // The main drive
    private MecanumDrive mecanumDrive;

    // Motors corresponding to each wheel
    private SparkMax m_frontLeft;
    private SparkMax m_frontRight;
    private SparkMax m_backLeft;
    private SparkMax m_backRight;

    // Allow the motors to be measured and tracked
    private RelativeEncoder m_frontLeftEncoder;
    private RelativeEncoder m_frontRightEncoder;
    private RelativeEncoder m_backLeftEncoder;
    private RelativeEncoder m_backRightEncoder;
  
    // Create a new drive train
    public DriveTrain() {
        // Create motors
        m_frontLeft = new SparkMax(MotorConstants.kFRONT_LEFT_WHEEL_ID, MotorType.kBrushless);
        m_frontRight = new SparkMax(MotorConstants.kFRONT_RIGHT_WHEEL_ID, MotorType.kBrushless);
        m_backLeft = new SparkMax(MotorConstants.kBACK_LEFT_WHEEL_ID, MotorType.kBrushless);
        m_backRight = new SparkMax(MotorConstants.kBACK_RIGHT_WHEEL_ID, MotorType.kBrushless);

        // Get encoders
        m_frontLeftEncoder = m_frontLeft.getEncoder();
        m_frontRightEncoder = m_frontRight.getEncoder();
        m_backLeftEncoder = m_backLeft.getEncoder();
        m_backRightEncoder = m_backRight.getEncoder();

        // Create the drive
        mecanumDrive = new MecanumDrive(
            m_frontLeft, m_backLeft, 
            m_frontRight, m_backRight
        );
        // Prevents the motor from constantly starting and stopping
        // by considering low values as completely off
        mecanumDrive.setDeadband(DriveConstants.kDRIVE_DEADBAND);

        // Setup Kinematics
        // Locations of the wheels relative to the robot center
        // +X/-X is forward/backward, +Y/-Y is left/right
        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        // Create the kinematics object using wheel locations
        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, 
            m_backLeftLocation, m_backRightLocation
        );

        // Create the odometry object from the kinematics object and the initial wheel positions
        // too complicated for now
        /* 
        odometry = new MecanumDriveOdometry(
        SpeedConstants.kDriveKinematics,
        new Rotation2d(getHeading()),
        new MecanumDriveWheelPositions());
        MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        null, new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getPosition()
        )
        );*/
    }

    // Returns the current angle of the robot
    public double getHeading() {
        // For some weird reason, the gyro returns an angle that
        // goes BACKWARDS (Counter-clockwise). So let's invert.
        final double gyroAngle = RobotContainer.gyro.getAngle();
        final double invertedGyroAngle = -gyroAngle;

        return invertedGyroAngle;
    }
}
