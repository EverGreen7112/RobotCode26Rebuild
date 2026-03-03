package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class CloseShooterCommand extends Command {

    public CloseShooterCommand(){

    }

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterState.kClose);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setShooterState(ShooterState.kStop);
    }
}
