package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap.ShooterConstants;
import frc.robot.RobotContainer;

// This command handles the shooter during the tele-op phase
// of a game.
public class TeleShooter extends Command {
    // Keep track of when the shooter starts up
    // so that we can delay when the intake starts.
    private double startTime;
    // public PhotonPoseEstimator photonEstimator;
    // public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    // public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.25, 0.66, 0), new Rotation3d(0, 0, 0));

    //public 
    // Main methods
    public TeleShooter() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
        // photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    @Override
    public void execute() {
        // Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(RobotContainer.shooter.currentResult);
        // if (visionEst.isEmpty()) {
        //     visionEst = photonEstimator.estimateLowestAmbiguityPose(RobotContainer.shooter.currentResult);
        // }

        // Start the launcher immediately
        RobotContainer.shooter.setLauncherSpeed(-RobotContainer.shooter.launcherSpeed.getDouble(0.8));

        // Start the aggravator immediately
        RobotContainer.shooter.setAggrivatorSpeed(ShooterConstants.kAGGRIVATOR_SPEED);
        
        // Delay the shooter's intake
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - startTime;

        if (deltaTime > ShooterConstants.kINTAKE_DELAY) {
            RobotContainer.shooter.setIntakeSpeed(-ShooterConstants.kINTAKE_SPEED);
        }
    }

    @Override
    public void end(boolean interupted) {
        RobotContainer.shooter.setIntakeSpeed(0);
        RobotContainer.shooter.setLauncherSpeed(0);
        RobotContainer.shooter.setAggrivatorSpeed(0);
    }
}
