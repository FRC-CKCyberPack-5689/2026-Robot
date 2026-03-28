// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AssistedShoot extends Command {

	private DriveTrain m_drive;
	private Vision m_vision;
	private Shooter m_Shooter;
	private PIDController pidController;

	public AssistedShoot() {
		addRequirements(
				RobotContainer.driveTrain,
				RobotContainer.shooter,
				RobotContainer.vision);

		pidController = new PIDController(0.001, 0, 0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Get a target from the camera
		var target = m_vision.getTarget();

		boolean targetExists = target != null;
		if (targetExists) {
			
			double distance = m_vision.getDistanceToTarget(target);
			System.out.println("distance from target: " + distance);

			if (distance <= RMap.VisionConstants.kMaxShootDistance) {
				double rotationSpeed = pidController.calculate(m_vision.getAdjustedYaw(target), 0);
				m_drive.drive(0, 0, rotationSpeed);

				double targetShootSpeed = RMap.VisionConstants.kSHOOTER_MAP.get(distance);
				m_Shooter.setLauncherSpeed(targetShootSpeed);
				return;
			}
		}

		m_Shooter.setLauncherSpeed(0);
		m_drive.drive(0, 0, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
