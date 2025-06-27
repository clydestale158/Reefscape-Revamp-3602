// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3602.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  private RobotContainer robotContainer = new RobotContainer();
  private Command autonomousCommand;

  public Robot() {
    CanBridge.runTCP();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);//TODO question this. maybe take out if proven uselessS
    CommandScheduler.getInstance().run();
    robotContainer.preventTipping();

    Threads.setCurrentThreadPriority(false, 10);
    robotContainer.updateDriverMultiplier();

  }

  @Override
  public void autonomousInit() {

    autonomousCommand = robotContainer.autoChooser.getSelected();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    } else {
      SmartDashboard.putString("Errors", "No auton selected!! somebody is an absolute buns skibidi monkey");
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
