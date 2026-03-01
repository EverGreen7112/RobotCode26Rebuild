package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.Climb.CloseClimbCommand;
import frc.robot.Commands.Shooter.CloseShooterCommand;
import frc.robot.Commands.Shooter.ScoreCommand;
import frc.robot.Subsystems.Consts.AutoConsts;
import frc.robot.Subsystems.Conveyor.Conveyer;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.EverKit.Periodic;

public class AutonamousOperationsController implements Periodic, Consts.AutoConsts {
    private CloseShooterCommand m_closeShooterCommand;
    private ScoreCommand m_autoumaticShootCommand;
    private CloseClimbCommand m_closeClimbCommand;

    private boolean m_currentShouldCloseForTrench, m_previousShouldCloseForTrench;
    private boolean m_currentSouldShootAutoumaticly, m_previousSouldShootAutoumaticly;


    public AutonamousOperationsController(){
        m_closeShooterCommand = new CloseShooterCommand();
        m_closeClimbCommand = new CloseClimbCommand();
        m_autoumaticShootCommand = new ScoreCommand();
    }

    @Override
    public void periodic(){
        
        m_currentShouldCloseForTrench = shouldCloseForTrench();
        if(m_currentShouldCloseForTrench && !m_previousShouldCloseForTrench){
            startRobotTrenchMode();
        }

        else if(!m_currentShouldCloseForTrench && m_previousShouldCloseForTrench){
            stopRobotTrenchMode();
        }
        m_previousShouldCloseForTrench = m_currentShouldCloseForTrench;

        m_currentSouldShootAutoumaticly = shouldShootAutomaticly() && !m_previousShouldCloseForTrench;
        if(m_currentShouldCloseForTrench && !m_previousShouldCloseForTrench){
            m_autoumaticShootCommand.schedule();
        }

        else if(!m_currentShouldCloseForTrench && m_previousShouldCloseForTrench){
            m_autoumaticShootCommand.cancel();
        }
    }

    public void startRobotTrenchMode(){
        m_closeClimbCommand.schedule();
        m_closeShooterCommand.schedule();
    }

    public void stopRobotTrenchMode(){
        m_closeClimbCommand.cancel();
        m_closeShooterCommand.cancel();
    }

    public boolean shouldCloseForTrench(){
        double x = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double y = SwerveLocalizer.getInstance().getCurrentPoint().getY();

        return (x < AutoConsts.MAX_TRENCH_X && x > AutoConsts.MIN_TRENCH_X) && (y < AutoConsts.MAX_TRENCH_Y && y > AutoConsts.MIN_TRENCH_Y);
    }

    public boolean shouldShootAutomaticly(){
        double x = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double y = SwerveLocalizer.getInstance().getCurrentPoint().getY();

        boolean isInScoringHalf;
        Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Red && x < AutoConsts.RED_ALLIANCE_X){
            isInScoringHalf = true;
        }

        else if(alliance == Alliance.Blue && y > AutoConsts.RED_ALLIANCE_X){
            isInScoringHalf = true;
        }

        else{
            isInScoringHalf = false;
        }

        boolean isInShootingDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) < AutoConsts.MAX_SHOOTING_DISTANCE;
        boolean isThereFuelInRobot = Feeder.getInstance().getIsEmpty();

        return isInScoringHalf && isInShootingDistance && isThereFuelInRobot;
    }
}
