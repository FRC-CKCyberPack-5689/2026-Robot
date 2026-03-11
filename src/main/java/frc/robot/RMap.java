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
        // In the USB tab of the driver station, 
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
    }

    public static class DriveConstants {
        // Consider values below this as completely off.
        public static final double kDEADBAND = 0.05;

        // Controls the sensitivity of the robot's directional movement
        public static final double kAXIS_SPEED = 1;
        public static final double kAXIS_ACCELERATION = 0.005;
        public static final double kAXIS_DECELERATION = 0.01;

        // Controls the sensitivity of the robot's rotational movement
        public static final double kROTATION_SPEED = 1;
        public static final double kROTATION_ACCELERATION = 0.003;
        public static final double kROTATION_DECELERATION = 0.06;
    }

    public static class ShooterConstants {
        public static final double kLAUNCHER_SPEED = 0.8;
        public static final double kINTAKE_SPEED = 0.8;

        // Delays the startup of the shooter's intake
        // motor when the shooter is activated.
        public static final double kINTAKE_DELAY = 0.4;
    }

    public static class IntakeConstants {
        public static final double kINTAKE_SPEED = -0.4;

        public static final double kARM_P = 0.001;
        public static final double kARM_I = 0.002;
        public static final double kARM_D = 0.001;

        public static final double kARM_OUTPUT_MIN = -0.25;
        public static final double kARM_OUTPUT_MAX = 1.0;
    }
}
