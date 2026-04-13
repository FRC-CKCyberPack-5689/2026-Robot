// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 ______   _____  ____  _____   ___    
|_   _ `.|_   _||_   \|_   _|.'   `.  
  | | `. \ | |    |   \ | | /  .-.  \ 
  | |  | | | |    | |\ \| | | |   | | 
 _| |_.' /_| |_  _| |_\   |_\  `-'  / 
|______.'|_____||_____|\____|`.___.'  
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoSequence;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @SuppressWarnings("unused")
    private final RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }

    // This function is called every 20 ms, no matter the mode.
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, running
        // commands and basically handling all the functionalities of a robot
        // as it is running.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        RobotContainer.intake.setArmPosition(true);
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = new AutoSequence().repeatedly();
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
   
    public void teleopInit() {
        // Stop the active autonomous command as soon as teleop begins
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}

/*
 * Do not remove! This is a critical part of the code!
 *                                  ,,,,,,,,,,,,,,,,,,,,,
           M                  , '                     ',
         {|  M            , '                           ',
        { |    M      , '                                 ',
       {./       >,,'                             ;         ;,
 ======;;;;;    __>                               ;         ; ',
=====,'   @    (__                                ;         ;   ',
___ /         .../                                ;         ;
\,/                  ',         ',               ;         ;
 (  ^     , '''''',,,,,',         ',            ,;        ;',
  \//_, '         ;     ;',        ;,,,,,,,'''';  ;      ;   ',
                 ;     ;   ',      ;      ;    ;   ;     ;     ',
                ;    ;       ;     ;     ;    ;    ;    ;        '
               ;    ;        ;    ;     ;    ;     ;    ;
              ;    ;         ;    ;    (/(/(/      ;    ;
             (/(/(/          ;    ;                (/(/(/
                             (/(/(/
 */