package frc.robot.Commands.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor.Conveyer;

public class AutoStopConveyCommand extends Command {

    @Override
    public void initialize(){
        Conveyer.getInstance().stopConveying();
    } 

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }

    
}
