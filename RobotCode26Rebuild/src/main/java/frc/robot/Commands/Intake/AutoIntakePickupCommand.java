package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class AutoIntakePickupCommand extends Command {


    public AutoIntakePickupCommand(){

    }

    @Override
    public void initialize() {
        Intake.getInstance().startPickup();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    public void end(boolean interrupted) {

    }

    
}
