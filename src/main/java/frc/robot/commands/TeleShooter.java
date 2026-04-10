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

    // Option #3 (index 2) is selected by default
    private static int currentLauncherSpeedOption = 2;
    
    // Main methods
    public TeleShooter() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Calibration (use this instead)
        //double shooterPower = SmartDashboard.getNumber("Shooter Power", 0.65);
        //RobotContainer.shooter.setLauncherSpeed(shooterPower);

        // Start the launcher immediately
        double currentLauncherSpeed = ShooterConstants.kLAUNCHER_SPEED_OPTIONS[currentLauncherSpeedOption];
        RobotContainer.shooter.setLauncherSpeed(currentLauncherSpeed);
        
        // Delay the shooter's intake and aggravator
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - startTime;

        if (deltaTime > ShooterConstants.kMANUAL_DELAY) {
            RobotContainer.shooter.setIntakeSpeed(ShooterConstants.kINTAKE_SPEED);
            RobotContainer.shooter.setAggravatorSpeed(ShooterConstants.kAGGRAVATOR_SPEED);
        }
    }

    public static void incrementLauncherSpeed() {
        currentLauncherSpeedOption = Math.min(
            currentLauncherSpeedOption + 1, 
            ShooterConstants.kLAUNCHER_SPEED_OPTIONS.length - 1
        );

        System.out.println(
            "Launcher Speed: " + 
            (currentLauncherSpeedOption+1) + "/" + 
            ShooterConstants.kLAUNCHER_SPEED_OPTIONS.length + " (" +
            ShooterConstants.kLAUNCHER_SPEED_OPTIONS[currentLauncherSpeedOption] + ")"
        );
    }

    public static void decrementLauncherSpeed() {
        currentLauncherSpeedOption = Math.max(currentLauncherSpeedOption - 1, 0);

        System.out.println(
            "Launcher Speed: " + 
            (currentLauncherSpeedOption+1) + "/" + 
            ShooterConstants.kLAUNCHER_SPEED_OPTIONS.length + " (" +
            ShooterConstants.kLAUNCHER_SPEED_OPTIONS[currentLauncherSpeedOption] + ")"
        );
    }

    @Override
    public void end(boolean interupted) {
        RobotContainer.shooter.setIntakeSpeed(0);
        RobotContainer.shooter.setLauncherSpeed(0);
        RobotContainer.shooter.setAggravatorSpeed(0);
    }
}
