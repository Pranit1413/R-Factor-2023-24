// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.visionSubsystem;

public class visionTurnToTag extends CommandBase {
  /** Creates a new visionTurnToTag. */
  public DriveSubsystem driveTrain;
  public visionSubsystem vision;
  double targetAngle; 
  double kp; 
  double error;

  public visionTurnToTag(DriveSubsystem driveTrain, visionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.vision = vision;
    addRequirements(driveTrain,vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getHasTarget()) { // if we find a target,
      error = vision.getBestTarget().getYaw(); // calculate error based off Yaw value of our current best target
      double value = -Math.min(error*kp, 1); // calculate motor percentage value

      driveTrain.visionturn(-value, value); // write values to motors, negative and positive value in order for turning to occur
    } else {
      driveTrain.arcadeDrive(0, 0); // otherwise, don't do anything
    }
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
