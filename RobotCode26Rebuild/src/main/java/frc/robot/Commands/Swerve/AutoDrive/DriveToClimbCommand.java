package frc.robot.Commands.Swerve.AutoDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Funcs;

public class DriveToClimbCommand extends Command implements Consts.ClimbConst {
     private final double ALIGNMENT_DIS = 0.1;

    private Command m_driveCommand;
    private Pose2d m_targetClimbPose2d;

    public DriveToClimbCommand() {

    }

    @Override
    public void initialize() {
        boolean isBlue = Robot.m_alliance == Alliance.Blue;
        m_targetClimbPose2d = isBlue ? BLUE_CLIMB_POSE : RED_CLIMB_POSE;
        Pose2d currentPose = SwerveLocalizer.getInstance().getCurrentPoint();
        
        //use pathplanner only for long distances
        if(Funcs.getDis(currentPose, m_targetClimbPose2d) > ALIGNMENT_DIS){
            Pose2d beforeClimb = m_targetClimbPose2d.plus(new Transform2d(-ALIGNMENT_DIS, 0, new Rotation2d()));
            m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(beforeClimb, 0.1)
                             .andThen(new AlignToClimbCommand(isBlue));

        
        }
        else{
            m_driveCommand = new AlignToClimbCommand(isBlue);
        }

        m_driveCommand.schedule();
    }

    @Override
    public void execute() {
        SwerveAutoController.isRobotAligning = true;
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveCommand.cancel();
        SwerveAutoController.isRobotAligning = false;
   
    }
}
