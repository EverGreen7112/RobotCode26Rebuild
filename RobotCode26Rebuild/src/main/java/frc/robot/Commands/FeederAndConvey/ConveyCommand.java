package frc.robot.Commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder.Conveyer;

public class manualConveyCommand extends Command {

    public manualConveyCommand(){
        addRequirements(Conveyer.getInstance());
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
