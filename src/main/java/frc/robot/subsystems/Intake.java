// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RMap.MotorConstants;

public class Intake extends SubsystemBase {
    private SparkMax m_intake;
    private SparkMax m_intakeArm;

    private SparkMaxConfig intakeArmConfig;
    private SparkClosedLoopController armController;

    // Create a new intake
    public Intake() {
        m_intake = new SparkMax(MotorConstants.kINTAKE_ID, MotorType.kBrushless);

        // Intake Arm
        m_intakeArm = new SparkMax(MotorConstants.kINTAKE_ARM_ID, MotorType.kBrushless); 
        armController = m_intakeArm.getClosedLoopController();

        intakeArmConfig = new SparkMaxConfig();
        intakeArmConfig.closedLoop
            .p(MotorConstants.kINTAKE_ARM_P)
            .i(MotorConstants.kINTAKE_ARM_I)
            .d(MotorConstants.kINTAKE_ARM_D);
        intakeArmConfig.closedLoop.outputRange(
            MotorConstants.kINTAKE_ARM_OUTPUT_MIN, 
            MotorConstants.kINTAKE_ARM_OUTPUT_MAX
        );

        m_intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setArmPosition(double position) {
        armController.setSetpoint(position, ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }
}
