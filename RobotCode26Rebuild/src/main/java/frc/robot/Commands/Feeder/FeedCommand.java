package frc.robot.Commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder.Feeder;

public class FeedCommand extends Command {

    public FeedCommand(){
        addRequirements(Feeder.getInstance());
    }

    @Override
    public void initialize(){
        Feeder.getInstance().startFeed();
    } 

    @Override
    public boolean isFinished() {
        return Feeder.getInstance().getIsEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        Feeder.getInstance().stopFeed();
    }
    
}
