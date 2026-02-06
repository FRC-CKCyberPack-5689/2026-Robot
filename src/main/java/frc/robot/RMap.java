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
        public static final double kDRIVE_MAX_ACCELERATION = 0.02;
        public static final double kDRIVE_MAX_DECELERATION = 0.04;
    }

    public static class MotorConstants {
        public static final int kFRONT_LEFT_WHEEL_ID = 100;
        public static final int kFRONT_RIGHT_WHEEL_ID = 101;
        public static final int kBACK_LEFT_WHEEL_ID = 102;
        public static final int kBACK_RIGHT_WHEEL_ID = 103;
    }
}
