package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.Climb.CloseClimbCommand;
import frc.robot.Commands.Conveyor.ConveyCommand;
import frc.robot.Commands.Shooter.CloseShooterCommand;
import frc.robot.Commands.Shooter.ScoreCommand;
import frc.robot.Subsystems.Consts.AutoConsts;
import frc.robot.Subsystems.Conveyor.Conveyer;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.EverKit.Periodic;

public class AutoOperationsController implements Periodic, Consts.AutoConsts {
    
    private static AutoOperationsController m_instance = new AutoOperationsController();
    
    private CloseShooterCommand m_closeShooterCommand;
    private ScoreCommand m_autoShootCommand;
    private CloseClimbCommand m_closeClimbCommand;

    private boolean m_shouldCloseForTrench, m_prevShouldCloseForTrench;
    private boolean m_ShouldShootAuto, m_prevShouldShootAuto;

    private ConveyCommand m_conveyCommand;

    private Pose2d m_robotPose;

    private Alliance m_alliance;

    private boolean m_autoMode;


    private AutoOperationsController(){
        m_closeShooterCommand = new CloseShooterCommand();
        m_closeClimbCommand = new CloseClimbCommand();
        m_autoShootCommand = new ScoreCommand();

        m_robotPose = SwerveLocalizer.getInstance().getCurrentPoint();

        m_alliance = Alliance.Blue;

        m_conveyCommand = new ConveyCommand();
        m_autoMode = true;

        m_conveyCommand.schedule();
    }

    public static AutoOperationsController getInstance(){
        return m_instance;
    }
    
    public void setAlliance(boolean isBlue){ 
        m_alliance = isBlue ? Alliance.Blue : Alliance.Red;
    }
    
    public void startRobotTrenchMode(){
        m_closeClimbCommand.schedule();
        m_closeShooterCommand.schedule();
    }

    public void stopRobotTrenchMode(){
        m_closeClimbCommand.cancel();
        m_closeShooterCommand.cancel();
    }

    //TODO: I would remove this function use the logic directly in periodic
    private boolean shouldCloseForTrench(){
        return isInLeftTrench() || isInRightTrench();
    }

    private boolean isInRightTrench(){
        double x = m_robotPose.getY();
        return x < AutoConsts.RIGHT_MAX_TRENCH_X && x > AutoConsts.RIGHT_MIN_TRENCH_X;
    }

    private boolean isInLeftTrench(){
        double x = m_robotPose.getY();
        return x < AutoConsts.LEFT_MAX_TRENCH_X && x > AutoConsts.LEFT_MIN_TRENCH_X;
    }


    //TODO: use this to dicide shooting button function
    public boolean shouldScore(){
        double x = m_robotPose.getY();

        
        
        boolean isInScoringZone;
        if(m_alliance == Alliance.Blue){
            isInScoringZone = x < AutoConsts.BLUE_ALLIANCE_ZONE_X;
        }
        else{
            isInScoringZone = x > AutoConsts.RED_ALLIANCE_ZONE_X;
        }

        boolean isNotEmpty = Feeder.getInstance().getIsEmpty();

        if(m_shouldCloseForTrench)
            return false;
        return isInScoringZone && isNotEmpty;
    }

    public void setAutoMode(boolean isOn){
        m_autoMode = isOn;
    }


    //TODO: if relevant need to add auto intake closing over the bump
    @Override
    public void periodic(){

        m_robotPose = SwerveLocalizer.getInstance().getCurrentPoint();
        
        //trench mode
        m_shouldCloseForTrench = shouldCloseForTrench();
        if(m_shouldCloseForTrench && !m_prevShouldCloseForTrench){
            startRobotTrenchMode();
        }
        else if(!m_shouldCloseForTrench && m_prevShouldCloseForTrench){
            stopRobotTrenchMode();
        }
        m_prevShouldCloseForTrench = m_shouldCloseForTrench;
        
        if(m_autoMode){

            //auto scoring
            m_ShouldShootAuto = shouldScore();
            if(m_ShouldShootAuto && !m_prevShouldShootAuto){
                m_autoShootCommand.schedule();
            }
            else if(!m_ShouldShootAuto && m_prevShouldShootAuto){
                m_autoShootCommand.cancel();
            }
            m_prevShouldShootAuto = m_ShouldShootAuto;
        }

    }

}
