package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Conveyor.ConveyCommand;
import frc.robot.Commands.Feeder.FeedCommand;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class DeliverCommand extends Command {

    FeedCommand m_feed;

    @Override
    public void initialize() {
        Shooter.getInstance().setShooterState(ShooterState.kDelivery);
        m_feed = new FeedCommand();
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
