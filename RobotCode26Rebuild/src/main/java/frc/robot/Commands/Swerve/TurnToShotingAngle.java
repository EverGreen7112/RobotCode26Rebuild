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

public class TurnToShotingAngle extends Command{

    boolean m_isShooting = Shooter.getInstance().getIsShooting();

    double m_targetAngle;

    Pose2d m_targetHub = ShooterConsts.HUB_POINT;

    private void calculateTargetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOfSetAngle = Math.atan2(y, x);

        Vector2d fuelVectorAngle = new Vector2d(Shooter.getInstance().getCurrentVel(), 0);
        Vector2d robotVector = Swerve.getInstance().getRobotOrientedVelocity();

        double dot = fuelVectorAngle.dot(robotVector);
        double mag = fuelVectorAngle.mag() * robotVector.mag();
        
        m_targetAngle = robotsOfSetAngle + Math.acos(dot / mag); 
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
