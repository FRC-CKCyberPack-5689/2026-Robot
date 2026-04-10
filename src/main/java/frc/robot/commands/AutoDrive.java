package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoDrive extends Command {
    public AutoDrive() {
        addRequirements(RobotContainer.driveTrain);
        
        
    }  
}
