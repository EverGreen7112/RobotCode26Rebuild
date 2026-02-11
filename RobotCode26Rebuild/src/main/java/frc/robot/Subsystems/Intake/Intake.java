package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Ounce;

import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkFlex;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Intake extends SubsystemBase{

    private static Intake m_instance = new Intake();

    private EverMotorController m_pickupMotor, m_extentionMotor;
    private EverEncoder m_openingEncoder;

    private DigitalInput m_retractionLM, m_extntionLM; //two thing first limit swtch doesnt realy have reason // why?

    private final double EXTENTION_SPEED = 0.25, PICKUP_SPEED = 0.6;
    private final boolean LOG = false;

    private boolean m_isOpen = true;

    private Intake(){
        m_pickupMotor = new EverSparkFlex(0);
        m_extentionMotor = new EverTalonFX(0);

        m_openingEncoder = new EverTalonFXInternalEncoder((EverTalonFX)m_extentionMotor);// you see you had to convert this is because you dont use the everkit correctley

        m_retractionLM = new DigitalInput(0);
        m_extntionLM = new DigitalInput(0);
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void setExtntionState(boolean isOpen){
        m_isOpen = isOpen;
    }

    public boolean getExtentionState(){
        return m_isOpen;
    }

    public void setExtntionSpeed(){
        double speed = m_isOpen ? EXTENTION_SPEED : -EXTENTION_SPEED;
        if((m_extntionLM.get() && m_extentionMotor.get() > 0) || (m_retractionLM.get() && m_extentionMotor.get() < 0)){
            speed = 0;
        }
        if(speed != m_extentionMotor.get()){
            m_extentionMotor.set(speed);
        }
        return;
    }

    public void setPickupSpeed(){
        double pickupSpeed = m_isOpen ? PICKUP_SPEED : 0;
        if(pickupSpeed != m_pickupMotor.get()){
            m_pickupMotor.set(pickupSpeed);
        }
        return;
    }

    @Override
    public void periodic() {
        setExtntionSpeed();
        setPickupSpeed();

        if(LOG){
            log();
        }
    }
    // no functions for rollers

    private void log(){
        // log motor output and encoder values
    }
    
}
