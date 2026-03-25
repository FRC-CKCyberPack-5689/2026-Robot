// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * This class stores all the constant values
 * used across the robot.
 */
public final class RMap {
    public static class OperatorConstants {
        // In the USB tab of the driver station, this is the port
        // of our controller.
        public static final int kDRIVER_CONTROLLER_PORT = 0;
    }

    public static class MotorIds {
        // 0 is reserved for the power distribution
        // 1 is reserved for the RoboRIO

        public static final int kFRONT_LEFT_WHEEL_ID = 2;
        public static final int kFRONT_RIGHT_WHEEL_ID = 3;
        public static final int kBACK_LEFT_WHEEL_ID = 4;
        public static final int kBACK_RIGHT_WHEEL_ID = 5;

        public static final int kINTAKE_ID = 7;
        public static final int kINTAKE_ARM_ID = 8;

        public static final int kSHOOTER_INTAKE_ID = 9;
        public static final int kSHOOTER_LAUNCHER_ID = 10;
        public static final int kSHOOTER_AGGRIVATOR_ID = 11;
    }

    public static class DriveConstants {
        // Consider values below this as completely off.
        public static final double kDEADBAND = 0.05;

        // Controls the sensitivity of the robot's movement.
        public static final double kMAX_ACCELERATION = 0.04;
        public static final double kMAX_DECELERATION = 0.08;
    }

    public static class ShooterConstants {
        public static final double kLAUNCHER_SPEED = 0.8;
        public static final double kINTAKE_SPEED = 0.8;

        // Delays the startup of the shooter's intake
        // motor when the shooter is activated.
        public static final double kINTAKE_DELAY = 0.4;

        // THE AGGRIVATOR SPEED VAR
        public static final double kAGGRIVATOR_SPEED = -0.2;
    }

    public static class IntakeConstants {
        public static final double kINTAKE_SPEED = 0.9;

        public static final int kARM_CURRENT_LIMIT = 27;
        public static final double kARM_DOWN_POSITION = -0.05;
        public static final double kARM_UP_POSITION = 0.5;

        public static final double kARM_OUTPUT_MIN = -0.25;
        public static final double kARM_OUTPUT_MAX = 0.25;
    }

    public static final class VisionConstants {
        public static final String kCameraName = "DinoCam"; // Match your PhotonVision name

        // Physical measurements (Meters)
        public static final double kCameraHeightMeters = 0.50;
        public static final double kTargetHeightMeters = 1.22; // 2026 Hub Height
        public static final double kCameraPitchRadians = Units.degreesToRadians(15.0);

        // Max distance to allow shooting (Meters)
        public static final double kMaxShootDistance = 4.5;

        // Interpolation Table: {Distance in Meters, Motor Velocity in RPM}
        public static final InterpolatingDoubleTreeMap kShooterMap = new InterpolatingDoubleTreeMap();
        static {
            kShooterMap.put(1.0, 0.7);
            kShooterMap.put(1.74, 0.8);
            kShooterMap.put(2.5, 0.9);
        }
    }
}


// 1.0m Middle 0.7
// 1.74m Middle 0.8
// 2.5m Middle 0.9