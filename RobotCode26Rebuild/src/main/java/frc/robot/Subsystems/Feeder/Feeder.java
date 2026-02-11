package frc.robot.Subsystems.Feeder;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Feeder extends SubsystemBase {
    //!! fix names
    private static Feeder m_instance = new Feeder();

    private EverMotorController m_feedingMotor;

    private DigitalInput m_enterLM;

    private double m_stallTimer;

    private Timer m_timer;

    private final double MAX_STALL_TIME = 0.5, FEEDING_VEL = 0.25; // place holder
    private final boolean LOG = false; 

    private boolean m_isFeeding = false, m_manualControl = false;

    private Feeder(){
        m_feedingMotor = new EverTalonFX(0);// diigus nigus why do that if we have the everkit why not use it
        m_enterLM = new DigitalInput(0);
        m_timer = new Timer();
        m_stallTimer = m_timer.getFPGATimestamp();
    }

    public static Feeder getInstance(){
        return m_instance;
    }

    private boolean shouldStopFeeding(){
        if(m_isFeeding){
            m_stallTimer = m_timer.getFPGATimestamp() - m_stallTimer;
            if(!m_enterLM.get())
                m_stallTimer = m_timer.getFPGATimestamp();
            
        } else {
            m_stallTimer = m_timer.getFPGATimestamp();
        }
        return m_stallTimer >= MAX_STALL_TIME;
    }

    public void setIsFeeding(boolean feeding){
        m_isFeeding = feeding;
    }

    public void setManualControl(boolean manual){
        m_manualControl = manual;
    }

    private void setFeedingSpeed(){
        double speed;
        speed = shouldStopFeeding() ? 0 : FEEDING_VEL;
        if(m_manualControl){
            speed = m_isFeeding ? FEEDING_VEL : 0;
        }
        m_isFeeding = speed != 0;
        if(speed != m_feedingMotor.get())
            m_feedingMotor.set(speed);
    }

    @Override
    public void periodic() {
        setFeedingSpeed();
        if(LOG)
            log();

    }


    private void log(){
        // log limit switch state and motor current
    }

}
