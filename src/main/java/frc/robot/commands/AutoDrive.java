package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.DriveConstants;
import frc.robot.RobotContainer;

public class AutoDrive extends Command {
    public AutoDrive() {
        addRequirements(RobotContainer.driveTrain, RobotContainer.vision);
    }  

    @Override
    public void execute() {
        // Get a target from the camera
		PhotonTrackedTarget target = RobotContainer.vision.getTarget();

		// Check if a valid target exists
		if (target != null) {
            System.out.println("Auto Drive finished, found a target!");
            // If it does, we are done here.
            this.cancel();
            return;
        }

        // Reverse away from the hub
        RobotContainer.driveTrain.drive(-DriveConstants.kAUTO_DRIVE_SPEED, 0, 0);
        
        System.out.println("Auto Driving!");
    }
}
