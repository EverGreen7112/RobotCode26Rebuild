package frc.robot.Subsystems.FeedAndConvey;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Feeder extends SubsystemBase {

    private static Feeder m_instance = new Feeder();

    private EverMotorController m_feedingMotor;

    private DigitalInput m_enterLeftLM, m_enterRightLM;

    private double m_stallTimer, m_lastSpeed;

    private Timer m_timer;

    private boolean m_isFeeding;

    private Feeder(){
        m_timer = new Timer();
        m_stallTimer = m_timer.getFPGATimestamp();
        m_feedingMotor = FeedAndConveyConsts.FEEDING_MOTOR;
        m_enterLeftLM = FeedAndConveyConsts.ENTER_LEFT_LM;
        m_enterRightLM = FeedAndConveyConsts.ENTER_RIGHT_LM;
        
        m_isFeeding = false;
        m_lastSpeed = 0;
    }

    public static Feeder getInstance(){
        return m_instance;
    }


    // TODO: change this to DeltaTime class in ShooterB branch
    private boolean shouldStopFeeding(){
        double currentDeltaTime = 0;
        if(m_isFeeding){
            currentDeltaTime = m_timer.getFPGATimestamp() - m_stallTimer;
            if(!m_enterLeftLM.get() || !m_enterRightLM.get())
                m_stallTimer = m_timer.getFPGATimestamp();
            
        } else {
            m_stallTimer = m_timer.getFPGATimestamp();
        }
        return currentDeltaTime >= FeedAndConveyConsts.FEEDER_MAX_STALL_TIME;
    }

    public void setIsFeeding(boolean feeding){
        m_isFeeding = feeding;
    }

    private void controlFeedingSpeed(){
        double speed = shouldStopFeeding() ? 0 : FeedAndConveyConsts.FEEDING_SPEED;
        m_isFeeding = speed != 0;
        if(speed != m_lastSpeed){
            m_feedingMotor.set(speed);
        }
        m_lastSpeed = speed;
    }

    @Override
    public void periodic() {
        controlFeedingSpeed();
        if(FeedAndConveyConsts.DEBUG_MOD)
            log();

    }

    private void log(){
        SmartDashboard.putNumber("Feeder Stall Timer", m_stallTimer);
        SmartDashboard.putBoolean("Feeder Is Feeding", m_isFeeding);
        SmartDashboard.putNumber("Feeder Motor Speed", m_feedingMotor.get());
        SmartDashboard.putBoolean("Feeder Left LM", m_enterLeftLM.get());
        SmartDashboard.putBoolean("Feeder Right LM", m_enterRightLM.get());
    }

}
