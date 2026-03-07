// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * This class stores all the constant values
 * used across the robot.
 */
public final class RMap {
    public static class OperatorConstants {
        public static final int kDRIVER_CONTROLLER_PORT = 0;
    }

    public static class DriveConstants {
        public static final double kDRIVE_DEADBAND = 0.05;
        public static final double kDRIVE_MAX_SPEED = 1;
        public static final double kDRIVE_MAX_ACCELERATION = 0.005;
        public static final double kDRIVE_MAX_DECELERATION = 0.01;
    }

    public static class MotorConstants {
        // Ids
        public static final int kFRONT_LEFT_WHEEL_ID = 2;
        public static final int kFRONT_RIGHT_WHEEL_ID = 3;
        public static final int kBACK_LEFT_WHEEL_ID = 4;
        public static final int kBACK_RIGHT_WHEEL_ID = 5;
        
        public static final int kINTAKE_ID = 7;
        public static final int kINTAKE_ARM_ID = 8;

        public static final int kSHOOTER_INTAKE_ID = 9;
        public static final int kSHOOTER_LAUNCHER_ID = 10;

        // Speeds
        public static final double kSHOOTER_LAUNCHER_SPEED = 0.8;
        public static final double kSHOOTER_INTAKE_SPEED = 0.8;
        public static final double kSHOOTER_INTAKE_DELAY = 0.4;

        // Currents
        public static final double kINTAKE_ARM_CURRENT = 0.5;
        public static final double kINTAKE_SPEED = -0.4;

        // Intake Arm
        public static final double kINTAKE_ARM_P = 0.01;
        public static final double kINTAKE_ARM_I = 0.002;
        public static final double kINTAKE_ARM_D = 0.001;

        public static final double kINTAKE_ARM_OUTPUT_MIN = -0.25;
        public static final double kINTAKE_ARM_OUTPUT_MAX = 1.0;
    }
}
