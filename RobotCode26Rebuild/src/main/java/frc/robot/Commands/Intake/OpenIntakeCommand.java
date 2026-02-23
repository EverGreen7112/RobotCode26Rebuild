package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class OpenIntakeCommand extends Command {
    
    public OpenIntakeCommand(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Intake.getInstance().setExtensionState(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
