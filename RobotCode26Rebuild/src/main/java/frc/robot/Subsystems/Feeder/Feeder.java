package frc.robot.Subsystems.Feeder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
import frc.robot.Utils.DeltaTime;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Feeder extends SubsystemBase implements Consts.FeedAndConveyConsts{

    private static Feeder m_instance = new Feeder();

    private EverMotorController m_feedingMotor;

    private DigitalInput m_enterLeftLM, m_enterRightLM;

    private DeltaTime m_counter;

    private boolean m_isEmpty;

    private Feeder(){
        m_feedingMotor = FeedAndConveyConsts.FEEDING_MOTOR;
        m_enterLeftLM = FeedAndConveyConsts.ENTER_LEFT_LM;
        m_enterRightLM = FeedAndConveyConsts.ENTER_RIGHT_LM;
        
        m_counter = new DeltaTime();

        m_isEmpty = true;

    }

    public static Feeder getInstance(){
        return m_instance;
    }

    public void startFeed(){
        m_feedingMotor.set(FEEDING_SPEED);
    }

    public void stopFeed(){
        m_feedingMotor.set(0);
    }

    public boolean getIsEmpty(){
        return m_isEmpty;
    }

    @Override
    public void periodic(){
        if(m_enterLeftLM.get() || m_enterRightLM.get()){
            m_counter.setNow();
            m_isEmpty = false;
        }

        if(m_counter.get() > FeedAndConveyConsts.FEEDER_MAX_STALL_TIME){
            m_isEmpty = true;
        }

        if(FeedAndConveyConsts.DEBUG_MOD)
            log();

    }

    private void log(){
        SmartDashboard.putNumber("Feeder Motor Speed", m_feedingMotor.get());
        SmartDashboard.putBoolean("Feeder Left LM", m_enterLeftLM.get());
        SmartDashboard.putBoolean("Feeder Right LM", m_enterRightLM.get());
    }

}
