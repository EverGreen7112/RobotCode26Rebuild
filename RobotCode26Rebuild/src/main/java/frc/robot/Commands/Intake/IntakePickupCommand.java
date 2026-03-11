package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakePickupCommand extends Command{

    public IntakePickupCommand(){

    }

    @Override
    public void initialize() {
        Intake.getInstance().startPickup();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().stopPickup();
    }
    
}
