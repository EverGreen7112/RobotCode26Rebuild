package frc.robot.Commands.Swerve.Auto;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Constents;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;
public class DriveToTowerCommand extends Command{



    private static final double POS_ERROR_TOLERANCE = 0;
    
    private final double ALIGNMENT_DIS = 0.1;

    private Pose2d m_targetTower;

    private ProfiledPIDController m_xController;
    private ProfiledPIDController m_yController;

    public DriveToTowerCommand() {
        addRequirements(Swerve.getInstance());
        m_xController = new ProfiledPIDController(4, 0, 0, new Constraints(0.7, 1));
        m_yController = new ProfiledPIDController(4, 0, 0, new Constraints(0.7, 1));

        m_targetTower = SwerveAutoController.getInstance().getAlliance().compareTo(DriverStation.Alliance.Red) == 0 ? Constents.HUB_POSES[0] : Constents.HUB_POSES[1];
    }

    @Override
    public void initialize() {
        
        SwerveAngleController.getInstance().start(m_targetTower.getRotation().getDegrees(), true);

        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        m_xController.reset(pose.getX());
        m_yController.reset(pose.getY());

        m_xController.setGoal(m_targetTower.getX());
        m_yController.setGoal(m_targetTower.getY());

        m_xController.setTolerance(POS_ERROR_TOLERANCE);
        m_yController.setTolerance(POS_ERROR_TOLERANCE); 
    }

    @Override
    public void execute() {
        
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = m_xController.calculate(pose.getX(), m_targetTower.getX());
        double yOutput = m_yController.calculate(pose.getY(), m_targetTower.getY());

        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());

        if(Math.abs(pose.getX() - m_targetTower.getX()) < POS_ERROR_TOLERANCE)   
            xOutput = 0;
        
        if(Math.abs(pose.getY() - m_targetTower.getY()) < POS_ERROR_TOLERANCE)
            yOutput = 0;


        Vector2d fieldOrientedVel = new Vector2d(xOutput, yOutput);
        fieldOrientedVel.rotate(pose.getRotation().getRadians() * SwerveConsts.GYRO_DIRECTION);
        Swerve.getInstance().driveByVelocity(fieldOrientedVel, false);
    }

    @Override
    public boolean isFinished() {
        return m_xController.atGoal() && m_yController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
        //new ClimbOpenCommand.schedule();

    }
}
    
