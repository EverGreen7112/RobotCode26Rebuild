package frc.robot.Commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder.Feeder;

public class ReversFeeder extends Command{


    public ReversFeeder(){
        addRequirements(Feeder.getInstance());
    }

    @Override
    public void initialize(){
        Feeder.getInstance().reversFeed();
    } 

    @Override
    public boolean isFinished() {
        return false;
        //Feeder.getInstance().getIsEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        Feeder.getInstance().stopFeed();
    }
    
}
