package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class OpenClimbCommand extends Command {
    
    public OpenClimbCommand(){
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize() {
        Climb.getInstance().open();
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
