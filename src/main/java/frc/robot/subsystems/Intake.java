// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RMap.MotorConstants;

public class Intake extends SubsystemBase {
    private SparkMax m_intake;
    private SparkMax m_intakePivot;

    private SparkClosedLoopController pivotController;

    // Create a new intake
    public Intake() {
        m_intakePivot = new SparkMax(MotorConstants.kINTAKE_PIVOT_ID, MotorType.kBrushless); 
        pivotController = m_intakePivot.getClosedLoopController();
        
        m_intake = new SparkMax(MotorConstants.kINTAKE_ID, MotorType.kBrushless);
    }

    public void setPivotCurrent(double current) {
        pivotController.setSetpoint(current, ControlType.kCurrent);
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }
}
