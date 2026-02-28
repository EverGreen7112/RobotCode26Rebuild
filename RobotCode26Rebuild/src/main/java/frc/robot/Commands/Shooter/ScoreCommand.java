package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class ScoreCommand extends Command{

    public ScoreCommand(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterState.kScoring);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setShooterState(ShooterState.kStop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
