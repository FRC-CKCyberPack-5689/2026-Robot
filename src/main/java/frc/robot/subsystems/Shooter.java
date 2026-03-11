package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RMap.MotorIds;

public class Shooter extends SubsystemBase {
    private SparkMax m_intake;
    private SparkFlex m_launcher;

    private double targetYaw;
    private boolean targetVisible;
    private Alliance currentAlliance;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    public GenericEntry launcherSpeed =
        tab.add("Launcher Speed", 0.8)
            .getEntry();

    // Create a new shooter
    public Shooter() {
        m_intake = new SparkMax(MotorIds.kSHOOTER_INTAKE_ID, MotorType.kBrushless);
        m_launcher = new SparkFlex(MotorIds.kSHOOTER_LAUNCHER_ID, MotorType.kBrushless);

        targetYaw = 0;
        targetVisible = false;

        currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    @Override
    public void periodic() {
        var results = RobotContainer.camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var latestResult = results.get(results.size() - 1);

            for (var target : latestResult.getTargets()) {
                if (currentAlliance == Alliance.Red && target.getFiducialId() == 10) {
                    targetYaw = target.getYaw();
                    targetVisible = true;
                    break;
                }

                if (currentAlliance == Alliance.Blue && target.getFiducialId() == 26) {
                    targetYaw = target.getYaw();
                    targetVisible = true;
                    break;
                }

                targetYaw = 0;
                targetVisible = false;
            }
        }
    }

    public void setLauncherSpeed(double speed) {
        m_launcher.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }

    public double getTargetYaw() {
        return targetYaw;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }
}
