package frc.robot.Commands.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
import frc.robot.Subsystems.Conveyor.Conveyer;
import frc.robot.Subsystems.Feeder.Feeder;

public class ConveyIntakeCommand extends Command implements FeedAndConveyConsts {

    public ConveyIntakeCommand(){
        addRequirements(frc.robot.Subsystems.Conveyor.Conveyer.getInstance());
    }

    @Override
    public void initialize(){
        Conveyer.getInstance().startConveying(INTAKE_CONVEY_SPEED);
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
