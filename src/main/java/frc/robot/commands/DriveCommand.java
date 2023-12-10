package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    
    private DriveSubsystem driveTrain;
    private DoubleSupplier lsp;
    private DoubleSupplier rsp;
    
    public DriveCommand(DriveSubsystem driveTrain, DoubleSupplier leftspeed, DoubleSupplier rightspeed){
        this.driveTrain = driveTrain;
        this.lsp = leftspeed;
        this.rsp = rightspeed;
        addRequirements(driveTrain);
    }

    

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        driveTrain.arcadeDrive(lsp.getAsDouble(),rsp.getAsDouble());
    }

    @Override
    public void end (boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}