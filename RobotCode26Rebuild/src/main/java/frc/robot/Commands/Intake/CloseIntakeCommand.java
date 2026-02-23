package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class CloseIntakeCommand extends Command {
    
    public CloseIntakeCommand(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Intake.getInstance().setExtensionState(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
