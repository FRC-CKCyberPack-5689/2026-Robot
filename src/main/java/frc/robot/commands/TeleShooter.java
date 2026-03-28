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
        // Start the launcher immediately
        RobotContainer.shooter.setLauncherSpeed(-ShooterConstants.kLAUNCHER_SPEED);

        // Start the aggravator immediately
        RobotContainer.shooter.setAggravatorSpeed(-ShooterConstants.kAGGRAVATOR_SPEED);
        
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
        RobotContainer.shooter.setAggravatorSpeed(0);
    }
}
