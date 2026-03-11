package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Conveyor.ConveyIntakeCommand;
import frc.robot.Commands.Feeder.FeedCommand;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class DeliverCommand extends Command {

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterState.kDelivery);
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
