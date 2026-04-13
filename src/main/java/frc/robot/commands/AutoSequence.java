// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RMap.DriveConstants;

public class AutoSequence extends SequentialCommandGroup {
	public AutoSequence() {
		// Run all the commands contained within, one after the other.
		addCommands(
			new AutoDrive().withTimeout(DriveConstants.kAUTO_DRIVE_MAX_TIME), 
			new AutoShoot()
		);
	}
}
