package frc.robot.Commands.Swerve;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Shooter.ScoreCommand;
import frc.robot.Commands.Shooter.StopShootCommand;
import frc.robot.Subsystems.Shooter.Shooter;

import frc.robot.Subsystems.Swerve.SwerveAngleController;


public class TurnToShootingAngle extends Command{

    @Override
    public void execute() {
        new ScoreCommand().schedule();
        double m_targetAngle = Shooter.getInstance().calcRobotShootingOffsetAngle();
        SwerveAngleController.getInstance().setTargetAngle(m_targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return true; // change this to the real condition for when the fuel runs out
    }
    
}
