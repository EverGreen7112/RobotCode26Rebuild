package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterConsts;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class ScoreCommand extends Command {

    public ScoreCommand(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterConsts.ShooterState.kScoring);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        Command stopShootCommand = new StopShootCommand(); //TODO: why dont you just use Shooter.getInstance().setShooterState(ShooterConsts.ShooterState.kStop) instead of the command
        stopShootCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
