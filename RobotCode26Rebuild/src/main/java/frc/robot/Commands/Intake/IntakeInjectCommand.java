package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts.IntakeConsts;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeInjectCommand extends Command{
    

    public IntakeInjectCommand(){

    }

    @Override
    public void initialize() {
        Intake.getInstance().startPickup(-IntakeConsts.PICKUP_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().stopPickup();
    }
    


}
