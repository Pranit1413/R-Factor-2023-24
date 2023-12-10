// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.primaryJoystick;
import frc.robot.Constants.secondaryJoystick;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Joystick driveStick = new Joystick(Constants.primaryJoystick.primaryStick);
  public static final Joystick systemStick = new Joystick(Constants.secondaryJoystick.secondaryStick);
 
  public final DriveSubsystem driveTrain = new DriveSubsystem();
  public final sprocketSubsystem turntable = new sprocketSubsystem();
  public final telescopicSubsystem arm = new telescopicSubsystem();
  public final armRotateSubsystem armUpDown = new armRotateSubsystem();
  public final gripperSubsystem gripper = new gripperSubsystem();
  // public final visionSubsystem vision = new visionSubsystem();
  public final PIDController turnController = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
  public DriveSubsystem navX;//gyroSetpoint zero;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // driveTrain.setDefaultCommand(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    
    
    new JoystickButton(driveStick, primaryJoystick.brake)
      .whileTrue(new DriveCommand(driveTrain, () -> 0,() -> 0))
      .whileFalse(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.rotateSpeed));
    
    // new JoystickButton(driveStick, primaryJoystick.robotLeft).whileTrue(new gyroSetpoint(driveTrain, 90*-1)).whileFalse(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));
    // new JoystickButton(driveStick, primaryJoystick.robotRight).whileTrue(new gyroSetpoint(driveTrain, 90)).whileFalse(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));
    new JoystickButton(driveStick, primaryJoystick.robot0).whileTrue(new robot0Command(driveTrain, robot0Command.zero)).whileFalse(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));

    new JoystickButton(driveStick, primaryJoystick.boost).whileTrue(new DriveCommand(driveTrain, () -> driveStick.getY() * 0.85,() -> driveStick.getZ() * 0.75)).whileFalse(new DriveCommand(driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.rotateSpeed));
  
    
  //   new JoystickButton(driveStick, JoystickConstants.rotateClockwise).toggleOnTrue(new sprocketRotateCommand(turntable, SpeedConstants.turnSpeed)).toggleOnFalse(new sprocketRotateCommand(turntable, 0));
    // new JoystickButton(driveStick, JoystickConstants.visionActivateS).toggleOnTrue(new visionTurnToTag(driveTrain, vision)).toggleOnFalse(new DriveCommand (driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));
    new JoystickButton(systemStick, secondaryJoystick.turntableright).whileTrue(new sprocketRotateCommand(turntable, SpeedConstants.turnSpeed)).whileFalse(new sprocketRotateCommand(turntable, 0));
    new JoystickButton(systemStick, secondaryJoystick.turntableleft).whileTrue(new sprocketRotateCommand(turntable, -SpeedConstants.turnSpeed)).whileFalse(new sprocketRotateCommand(turntable, 0));
    
    new JoystickButton(systemStick, secondaryJoystick.retract).whileTrue(new extendArmCommand(arm, SpeedConstants.retractSpeed)).whileFalse(new extendArmCommand(arm, 0));
    new JoystickButton(systemStick, secondaryJoystick.extend).whileTrue(new extendArmCommand(arm, -1*SpeedConstants.extendSpeed)).whileFalse(new extendArmCommand(arm, 0));

    new JoystickButton(systemStick, secondaryJoystick.armUp).whileTrue(new rotateArmup(armUpDown, SpeedConstants.telescopicSpeed)).whileFalse(new rotateArmup(armUpDown, 0));
    new JoystickButton(systemStick, secondaryJoystick.armDown).whileTrue(new rotateArmdown(armUpDown, -1*SpeedConstants.telescopicSpeed)).whileFalse(new rotateArmdown(armUpDown, 0));

    new JoystickButton(systemStick, secondaryJoystick.gripperOpen).whileTrue(new gripperCommand(gripper, SpeedConstants.gripperSpeed)).whileFalse(new gripperCommand(gripper, 0));
    new JoystickButton(systemStick, secondaryJoystick.gripperClose).whileTrue(new gripperCommand(gripper, -1*SpeedConstants.gripperSpeed)).whileFalse(new gripperCommand(gripper, 0));

    new JoystickButton(driveStick, 3).whileTrue(new autoBalanceCommand(navX, driveTrain)).whileFalse(new autoBalanceCommand(navX, driveTrain));
    // new JoystickButton(driveStick, JoystickConstants.visionActivate).toggleOnTrue(new visionTurnToTag(driveTrain, vision)).toggleOnFalse(new DriveCommand (driveTrain, () -> driveStick.getY()*Constants.SpeedConstants.driveSpeed,() -> driveStick.getZ()*Constants.SpeedConstants.driveSpeed));
    // new JoystickButton(systemStick, 7).onTrue(new InstantCommand(() -> driveTrain.changeSetpoint(1)));
    // new JoystickButton(systemStick, 8).onTrue(new InstantCommand(() -> driveTrain.incrementSetPoint()));
    // new JoystickButton(systemStick, 9).onTrue(new InstantCommand(() -> driveTrain.changeSetpoint(4)));
    // new JoystickButton(systemStick, 10).onTrue(new InstantCommand(() -> driveTrain.decrementSetPoint()));
    // new JoystickButton(systemStick, 11).onTrue(new InstantCommand(() -> driveTrain.changeSetpoint(0)));
    // new JoystickButton(systemStick, 0).onTrue(Commands.race(new retractArmCommand(arm, SpeedConstants.retractSpeed), new rotateArmup(armUpDown, SpeedConstants.telescopicSpeed)));
   
  }
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // new driveTime(driveTrain);
    // double startTime = Timer.getFPGATimestamp();
    // if (Robot.time - startTime < 3){
    //   driveTrain.Drive.arcadeDrive(0.6, 0);
    // }
    // System.out.println(Robot.time - startTime);
    
    return null;
  
}
}