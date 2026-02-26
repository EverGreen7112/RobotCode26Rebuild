package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeCommand extends Command {
    
    public IntakeCommand(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Intake.getInstance().startExtending();
        Intake.getInstance().startPickup();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stopExtending();
        Intake.getInstance().stopPickup();
    }
}
