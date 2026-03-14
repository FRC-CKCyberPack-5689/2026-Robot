// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RMap.MotorIds;
import frc.robot.RMap.IntakeConstants;


public class Intake extends SubsystemBase {
    private SparkFlex m_intake;
    private SparkMax m_intakeArm;

    private SparkMaxConfig intakeArmConfig;

    public double position;
    private boolean armIsUp;

    // Create a new intake
    public Intake() {
        m_intake = new SparkFlex(MotorIds.kINTAKE_ID, MotorType.kBrushless);

        // Intake Arm
        m_intakeArm = new SparkMax(MotorIds.kINTAKE_ARM_ID, MotorType.kBrushless);

        intakeArmConfig = new SparkMaxConfig();
        intakeArmConfig.closedLoop.outputRange(
            IntakeConstants.kARM_OUTPUT_MIN, 
            IntakeConstants.kARM_OUTPUT_MAX
        );
        intakeArmConfig.smartCurrentLimit(IntakeConstants.kARM_CURRENT_LIMIT, 2);
        intakeArmConfig.inverted(true);
        intakeArmConfig.idleMode(IdleMode.kBrake);

        m_intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // The intake arm starts up.
        position = IntakeConstants.kARM_DOWN_POSITION;
        armIsUp = false;
    }

    @Override
    public void periodic() {
        m_intakeArm.set(position);
    }

    /// Toggles the position of the arm, moving it into
    /// the up state if it is down, or moving it into
    /// the down state if it is up.
    public void toggleArmPosition() {
        if (armIsUp) {
            position = IntakeConstants.kARM_DOWN_POSITION;
        } else {
            position = IntakeConstants.kARM_UP_POSITION;
        }

        armIsUp = !armIsUp;
    }

    public void makeArmGoDownPlease() {
        ////armIsUp = false;
        //position = IntakeConstants.kARM_DOWN_POSITION;
    }

    public void setIntakeSpeed(double speed) {
        /* 
        if (speed != 0) {
            makeArmGoDownPlease();
        } else {
            armIsUp = true;
            position = IntakeConstants.kARM_UP_POSITION;
        }*/
        m_intake.set(speed);
    }
}
