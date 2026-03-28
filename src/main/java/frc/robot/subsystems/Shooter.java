package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap.MotorIds;

public class Shooter extends SubsystemBase {
    private SparkMax m_intake;
    private SparkFlex m_launcher;
    private SparkFlex m_aggravator;

    // Create a new shooter
    public Shooter() {
        m_intake = new SparkMax(MotorIds.kSHOOTER_INTAKE_ID, MotorType.kBrushless);
        m_launcher = new SparkFlex(MotorIds.kSHOOTER_LAUNCHER_ID, MotorType.kBrushless);
        m_aggravator = new SparkFlex(MotorIds.kSHOOTER_AGGRAVATOR_ID, MotorType.kBrushless);
    }

    /**
     * Sets the speed of the shooter's launcher motor.
     * 
     * @param speed A speed value between -1.0 and 1.0
     */
    public void setLauncherSpeed(double speed) {
        m_launcher.set(speed);
    }

    /**
     * Sets the speed of the shooter's intake motor.
     * 
     * @param speed A speed value between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }

    /**
     * Sets the speed of the shooter's aggravator motor.
     * 
     * @param speed A speed value between -1.0 and 1.0
     */
    public void setAggravatorSpeed(double speed) {
        m_aggravator.set(speed);
    }
}
