// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class driveTime extends CommandBase {
  /** Creates a new driveTime. */
  private DriveSubsystem driveTrain;
  // double startTime;
  public driveTime(DriveSubsystem driveTrain) {
   this.driveTrain = driveTrain;
   addRequirements(driveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double time = Timer.getFPGATimestamp();
    // System.out.println(time - startTime);

    // if (time - startTime < 3) {
      // FrontLeft.set(0.6);
      // CenterLeft.set(0.6);
      // BackLeft.set(0.6);
      // driveTrain.Drive.arcadeDrive(0.6,0);
    // } 
    driveTrain.Drive1M(0.6,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
