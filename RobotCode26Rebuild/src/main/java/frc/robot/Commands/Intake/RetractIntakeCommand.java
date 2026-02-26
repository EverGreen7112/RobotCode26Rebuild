package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class RetractIntakeCommand extends Command {
    
    public RetractIntakeCommand(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Intake.getInstance().startRetracting();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().stopExtending();
    }
    
}
