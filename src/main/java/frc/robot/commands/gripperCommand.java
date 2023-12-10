// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripperSubsystem;

public class gripperCommand extends CommandBase {
  /** Creates a new rotateArmSubsystem. */
  public gripperSubsystem gripper;
  public double speed;

  public gripperCommand(gripperSubsystem gripper, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    this.speed = speed;
    addRequirements(gripper);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.movingGripper(speed);
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
