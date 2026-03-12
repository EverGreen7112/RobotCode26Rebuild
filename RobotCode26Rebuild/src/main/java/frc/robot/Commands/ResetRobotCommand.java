package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Climb.CloseClimbCommand;
import frc.robot.Commands.Intake.RetractIntakeCommand;
import frc.robot.Commands.Shooter.CloseShooterCommand;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class ResetRobotCommand extends Command implements Consts{

    private ParallelCommandGroup m_resetCommandGroup;

    public static boolean resetting;

    public ResetRobotCommand(){
        resetting  = false;
    }

    @Override
    public void initialize(){
        resetting = true;
        Shooter.getInstance().setShooterState(ShooterState.kClose);
    }
    
    @Override
    public boolean isFinished() {
        return Shooter.getInstance().isClosed() && Climb.getInstance().cantClose() && Intake.getInstance().isClosed();
    }

    @Override
    public void end(boolean interrupted) {
        resetting = false;
    }


    
}
