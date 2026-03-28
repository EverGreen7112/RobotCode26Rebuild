package frc.robot.Commands.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
import frc.robot.Subsystems.Conveyor.Conveyer;

public class ReverseConveyCommand extends Command implements FeedAndConveyConsts{


    public ReverseConveyCommand(){
        addRequirements(frc.robot.Subsystems.Conveyor.Conveyer.getInstance());
    }

    @Override
    public void initialize(){
        Conveyer.getInstance().startConveying(-CONVEYING_TO_FEEDER_SPEED);
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Conveyer.getInstance().stopConveying();
    }
    
    
}
