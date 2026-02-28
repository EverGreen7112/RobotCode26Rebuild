package frc.robot.Commands.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor.Conveyer;

public class ConveyCommand extends Command {

    public ConveyCommand(){
        addRequirements(frc.robot.Subsystems.Conveyor.Conveyer.getInstance());
    }

    @Override
    public void initialize(){
        Conveyer.getInstance().startConveying();
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
