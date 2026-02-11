package frc.robot.Commands.Swerve.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Constents;

public class LockHubAngle extends Command{

    Pose2d m_targetHub;
    double m_targetAngle;

    public LockHubAngle(){
        m_targetHub = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ?
        Constents.HUB_POSES[0] : Constents.HUB_POSES[1];
    }

    private void calculateTargetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        m_targetAngle = Math.atan2(y, x);
    }

    @Override
    public void initialize() {
        calculateTargetAngle();
        SwerveAngleController.getInstance().start(m_targetAngle,true);
    }

    @Override
    public void execute() {
        calculateTargetAngle();
        SwerveAngleController.getInstance().setTargetAngle(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
    }



    
}
