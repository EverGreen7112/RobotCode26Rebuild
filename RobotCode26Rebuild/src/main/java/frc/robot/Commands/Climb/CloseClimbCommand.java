package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class CloseClimbCommand extends Command {
    
    public CloseClimbCommand(){

    }

    @Override
    public void initialize() {
        Climb.getInstance().close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Climb.getInstance().stop();
    }



}
