package frc.robot.Commands.Swerve;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Shooter.Manual.ManualStopShootCommand;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConsts;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Vector2d;

public class TurnToShootingAngle extends Command{

    boolean m_isShooting = Shooter.getInstance().getIsShooting();

    double m_targetAngle;


    private void calculateTargetAngle(){

    }

    

    @Override
    public void execute() {
        if(m_isShooting){
            calculateTargetAngle();
            SwerveAngleController.getInstance().setTargetAngle(m_targetAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Command stopShootCommand = new ManualStopShootCommand();
        stopShootCommand.schedule();
        SwerveAngleController.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return true; // change this to the real condition for when the fuel runs out
    }
    
}
