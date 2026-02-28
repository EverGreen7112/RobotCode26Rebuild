package frc.robot.Commands.FeederAndConvey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeedAndConvey.Conveyer;

public class ConveyCommand extends Command {

    public ConveyCommand(){
        addRequirements(frc.robot.Subsystems.FeedAndConvey.Conveyer.getInstance());
    }

    @Override
    public void initialize(){
        Conveyer.getInstance().setConveying(true);
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Conveyer.getInstance().setConveying(false);
    }
    
}
