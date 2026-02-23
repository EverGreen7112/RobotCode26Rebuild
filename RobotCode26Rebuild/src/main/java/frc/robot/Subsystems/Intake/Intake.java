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

    private EverMotorController m_pickupMotor, m_extensionMotor;

    private DigitalInput m_retractionLM, m_extensionLM; 

    private boolean m_isOpen = false;

    private Intake(){

        m_pickupMotor = IntakeConsts.PICKUP_MOTOR;
        m_extensionMotor = IntakeConsts.EXTENSION_MOTOR;

        m_retractionLM = IntakeConsts.RETRACTION_LM;
        m_extensionLM = IntakeConsts.EXTENSION_LM;
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void setExtensionState(boolean isOpen){
        m_isOpen = isOpen;
    }

    public boolean getExtensionState(){
        return m_isOpen;
    }

    public void setExtensionSpeed(){
        double speed = m_isOpen ? IntakeConsts.EXTENSION_SPEED : -IntakeConsts.EXTENSION_SPEED;
        if((m_extensionLM.get() && m_extensionMotor.get() > 0) || (m_retractionLM.get() && m_extensionMotor.get() < 0)){
            speed = 0;
        }
        if(speed != m_extensionMotor.get()){
            m_extensionMotor.set(speed);
        }
        return;
    }

    public void setPickupSpeed(){
        double pickupSpeed = m_isOpen ? IntakeConsts.PICKUP_SPEED : 0;
        if(pickupSpeed != m_pickupMotor.get()){
            m_pickupMotor.set(pickupSpeed);
        }
        return;
    }

    @Override
    public void periodic() {
        
        setExtensionSpeed();
        setPickupSpeed();

        if(IntakeConsts.DEBUG_MODE){
            log();
        }
    }

    private void log(){
        // log motor output and encoder values
    }
    
}
