package frc.robot.Commands.Shooter.Manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shoter;

public class ManualShootCommand extends Command {

    public ManualShootCommand(){
        addRequirements(Shoter.getInstance());
    }

    @Override
    public void initialize() {
        Shoter.getInstance().setIsShooting(true);
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
