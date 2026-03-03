package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class StopShootingCommand extends Command{
    
    public StopShootingCommand(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterState.kStop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
