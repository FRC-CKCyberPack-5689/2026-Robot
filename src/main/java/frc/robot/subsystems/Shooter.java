package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.MotorConstants;

public class Shooter extends SubsystemBase {
    private SparkMax m_intake;
    private SparkMax m_launcher;

    // Create a new shooter
    public Shooter() {
        m_intake = new SparkMax(MotorConstants.kSHOOTER_INTAKE_ID, MotorType.kBrushless);
        m_launcher = new SparkMax(MotorConstants.kSHOOTER_LAUNCHER_ID, MotorType.kBrushless);
    }

    public void setLauncherSpeed(double speed) {
        m_launcher.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }
}
