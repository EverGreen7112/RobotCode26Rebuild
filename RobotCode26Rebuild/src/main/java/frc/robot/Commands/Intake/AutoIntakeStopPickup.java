package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class AutoIntakeStopPickup extends Command{

    public AutoIntakeStopPickup(){

    }

    @Override
    public void initialize() {
        Intake.getInstance().stopPickup();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    public void end(boolean interrupted) {
    }
    
}
