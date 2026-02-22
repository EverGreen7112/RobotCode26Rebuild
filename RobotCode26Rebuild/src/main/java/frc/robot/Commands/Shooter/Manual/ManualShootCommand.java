package frc.robot.Commands.Shooter.Manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class ManualShootCommand extends Command {

    public ManualShootCommand(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().setIsShooting(true);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        Command stopShootCommand = new ManualStopShootCommand();
        stopShootCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
