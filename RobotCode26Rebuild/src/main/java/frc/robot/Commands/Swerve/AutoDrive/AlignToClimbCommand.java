package frc.robot.Commands.Swerve.AutoDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.ClimbConst;
import frc.robot.Subsystems.Consts.SwerveConsts;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Vector2d;

public class AlignToClimbCommand extends Command implements Consts.ClimbConst{
    private final double POS_ERROR = 0;
    private final double ANGLE_ERROR = 0;

    private boolean m_isBlue;

    private Pose2d m_targetClimb;
    private ProfiledPIDController m_xController;
    private ProfiledPIDController m_yController;

    public AlignToClimbCommand(boolean isBlue) {
        m_isBlue = isBlue;
        addRequirements(Swerve.getInstance());
        m_xController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        m_yController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
    
        m_isBlue = isBlue;
    }

    @Override
    public void initialize() {
        m_targetClimb = m_isBlue ? ClimbConst.BLUE_CLIMB_POSE_TOP : ClimbConst.RED_CLIMB_POSE_TOP;
        SwerveAngleController.getInstance().start(m_targetClimb.getRotation().getDegrees(), true);

        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        m_xController.reset(pose.getX());
        m_yController.reset(pose.getY());
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = m_xController.calculate(pose.getX(), m_targetClimb.getX());
        double yOutput = m_yController.calculate(pose.getY(), m_targetClimb.getY());

        if(Math.abs(pose.getX() - m_targetClimb.getX()) < POS_ERROR)   
           xOutput = 0;
        if(Math.abs(pose.getY() - m_targetClimb.getY()) < POS_ERROR)
           yOutput = 0;

        Vector2d fieldOrientedVel = new Vector2d(xOutput, yOutput);
        fieldOrientedVel.rotate(pose.getRotation().getRadians() * SwerveConsts.GYRO_DIRECTION);
        Swerve.getInstance().driveByVelocity(fieldOrientedVel, false);

    }

    @Override
    public boolean isFinished() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        return  MathUtil.isNear(m_targetClimb.getX(), pose.getX(), POS_ERROR) && 
                MathUtil.isNear(m_targetClimb.getY(), pose.getY(), POS_ERROR) &&
                MathUtil.isNear(m_targetClimb.getRotation().getDegrees(), pose.getRotation().getDegrees(), ANGLE_ERROR);

    }
    
    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
        Swerve.getInstance().stop();


    }

}
