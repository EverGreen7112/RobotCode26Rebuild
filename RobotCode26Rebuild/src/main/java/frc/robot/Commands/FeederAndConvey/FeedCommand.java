package frc.robot.Commands.FeederAndConvey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeedAndConvey.Feeder;

public class FeedCommand extends Command {

    public FeedCommand(){
        addRequirements(Feeder.getInstance());
    }

    @Override
    public void initialize(){
        Feeder.getInstance().setIsFeeding(true);
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Feeder.getInstance().setIsFeeding(false);
    }
    
}
