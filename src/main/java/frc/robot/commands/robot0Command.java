package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class robot0Command extends CommandBase {
    private final DriveSubsystem driveTrain;
    
    private final double targetAngle;
    public double getnavXAngle;
    private DoubleSupplier lsp;
    private DoubleSupplier rsp;
    public static double zero;

    public robot0Command(DriveSubsystem driveTrain, double targetAngle) {
        this.driveTrain = driveTrain;
        
        this.targetAngle = targetAngle;
        
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        driveTrain.arcadeDrive(0,0);
        zero = DriveSubsystem.navX.getAngle();

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
      return false;
        }
}