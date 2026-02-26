package frc.robot.Commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder.Feeder;

public class manualFeedCommand extends Command {

    public manualFeedCommand(){
        addRequirements(Feeder.getInstance());
    }

    @Override
    public void initialize(){
        Feeder.getInstance().setIsFeeding(true);
        Feeder.getInstance().setManualControl(true);
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Feeder.getInstance().setIsFeeding(false);
        Feeder.getInstance().setManualControl(false);
    }
    
}
