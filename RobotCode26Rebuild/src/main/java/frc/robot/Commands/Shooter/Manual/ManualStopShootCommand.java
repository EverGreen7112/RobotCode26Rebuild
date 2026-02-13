package frc.robot.Commands.Shooter.Manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shoter;

public class ManualStopShootCommand extends Command{
    
    public ManualStopShootCommand(){
        addRequirements(Shoter.getInstance());
    }

    @Override
    public void initialize() {
        Shoter.getInstance().setIsShooting(false);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
