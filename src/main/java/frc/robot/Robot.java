// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.annotation.processing.SupportedOptions;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Object sprocketSubsystem;

  public double startTime;
  public DriveSubsystem navX;

private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final DriveSubsystem driveTrain = new DriveSubsystem();
  private boolean isBalanced;
  
  // private final gripperSubsystem gripper = new gripperSubsystem();
  // private final armRotateSubsystem armUpDown = new armRotateSubsystem();
  // private final telescopicSubsystem arm = new telescopicSubsystem();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    this.startTime = Timer.getFPGATimestamp();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    

    // System.out.println("XXXXXXXXXXXXXXXXXX");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

 
    //ARM DOWN AUTO
    if (time - startTime > 0 && time - startTime < 1.75){
      armRotateSubsystem.rotateGroup.set(-0.2);
      driveTrain.Drive.arcadeDrive(0, 0);
    }
    //GRIPPER RUN AUTO
    else if(time - startTime > 1.75 && time - startTime < 3.25){
      // arm.extendarm1.set(0.2);
      gripperSubsystem.gripperGroup.set(0.275);
      armRotateSubsystem.rotateGroup.set(0);
      // armRotateSubsystem.rotateGroup.set(0.2);
      driveTrain.Drive.arcadeDrive(0, 0);
    }
    // BLUE ALLIANCE AUTO
    else if(time - startTime > 3.25 && time - startTime < 3.85){
      armRotateSubsystem.rotateGroup.set(0.2);
      gripperSubsystem.gripperGroup.set(0); 
    }
    else if(time - startTime > 3.85 && time - startTime < 4.86){
      driveTrain.Drive.arcadeDrive(0.77, 0);
      armRotateSubsystem.rotateGroup.set(0);
      gripperSubsystem.gripperGroup.set(0); 
    }
    // else if(time - startTime > 5.75 && time - startTime < 7){
    //   driveTrain.Drive.arcadeDrive(0.25, 0);
    //   armRotateSubsystem.rotateGroup.set(0);
    //   gripperSubsystem.gripperGroup.set(0); 
    // }
    // else if(time - startTime > 7 && time - startTime < 9){
    //   driveTrain.Drive.arcadeDrive(0.15, 0);
    //   armRotateSubsystem.rotateGroup.set(0);
    //   gripperSubsystem.gripperGroup.set(0); 
    // }
    //RED ALLAINCE AUTO 
    // else if(time - startTime > 3.75 && time - startTime < 7.3){
    //   driveTrain.Drive.arcadeDrive(0.6, 0);
    //   armRotateSubsystem.rotateGroup.set(0);
    //   gripperSubsystem.gripperGroup.set(0); 
    // }
    //TAXI
    // else if (time - startTime > 3.75 && time - startTime < 7.35){
    //   driveTrain.Drive.arcadeDrive(0.6, 0);
    //   armRotateSubsystem.rotateGroup.set(0);
    //   gripperSubsystem.gripperGroup.set(0); 
    // }
    //ROBOT STOP AUTO
     else if ((Math.abs(DriveSubsystem.navX.getRoll())) < Constants.AutoConstants.BALANCE_THRESHOLD) { // if the robot is balanced
      if (!isBalanced) { // if this is the first time the robot is balanced in this autonomous mode
        isBalanced = true;
      } 
        else { // if the robot is not balanced
       
      //   // keep the robot balanced and drive forward
      //   // isBalanced = false;
      //   // double balanceCorrection = BALANCE_KP * angle + BALANCE_KD * navX.getPitch();
      //   // double leftSpeed = MAX_DRIVE_SPEED - balanceCorrection;
      //   // double rightSpeed = MAX_DRIVE_SPEED + balanceCorrection;
        driveTrain.Drive.arcadeDrive(0.3, 0.3);
      }
    }
      
      // correct the balance by driving in the opposite direction of the tilt
      else if ( isBalanced == false){
     
      double correction = Constants.AutoConstants.BALANCE_KP * DriveSubsystem.navX.getRoll() + Constants.AutoConstants.BALANCE_KD * DriveSubsystem.navX.getRoll();
      double leftSpeed = Constants.AutoConstants.MAX_DRIVE_SPEED - correction;
      double rightSpeed = Constants.AutoConstants.MAX_DRIVE_SPEED - correction;
      DriveSubsystem.LeftGroup.set(leftSpeed*-0.13125);
      DriveSubsystem.RightGroup.set(rightSpeed*-0.13125);
      if(DriveSubsystem.navX.getRoll()==19.7){
      DriveSubsystem.LeftGroup.set(leftSpeed*0);
      DriveSubsystem.RightGroup.set(rightSpeed*0);
      }
      else {

      }
    }
      else{
      gripperSubsystem.gripperGroup.set(0);
      armRotateSubsystem.rotateGroup.set(0);
      driveTrain.Drive.arcadeDrive(0, 0);
    }
    SmartDashboard.putNumber("roll", DriveSubsystem.navX.getRoll());
    
    
    //System.out.println(time - startTime);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.driveTrain.arcadeDrive(RobotContainer.driveStick.getY()*Constants.SpeedConstants.driveSpeed, RobotContainer.driveStick.getZ()*Constants.SpeedConstants.rotateSpeed);
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
