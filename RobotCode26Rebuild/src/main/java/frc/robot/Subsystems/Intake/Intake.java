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
    private EverEncoder m_extensionEncoder;

    private DigitalInput m_retractionLM, m_extensionLM; 

    private Intake(){

        m_pickupMotor = IntakeConsts.PICKUP_MOTOR;
        m_extensionMotor = IntakeConsts.EXTENSION_MOTOR;
        m_extensionEncoder = IntakeConsts.EXETNSION_ENCODER;

        m_retractionLM = IntakeConsts.RETRACTION_LM;
        m_extensionLM = IntakeConsts.EXTENSION_LM;
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void startExtending(){
        m_extensionMotor.set(IntakeConsts.EXTENSION_SPEED);
    }

    public void startRetracting(){
        m_extensionMotor.set(-IntakeConsts.EXTENSION_SPEED);
    }

    public void stopExtending(){
        m_extensionMotor.stop();
    }

    public void startPickup(){
        m_pickupMotor.set(IntakeConsts.PICKUP_SPEED);
    }

    public void stopPickup(){
        m_pickupMotor.stop();
    }

    @Override
    public void periodic() {

        if((m_extensionLM.get() && m_extensionMotor.get() > 0) || (m_retractionLM.get() && m_extensionMotor.get() < 0)){
            stopExtending();
        }

        if(m_retractionLM.get()){
            m_extensionEncoder.setPos(0);
        }

        if(m_extensionEncoder.getPos() > IntakeConsts.MAX_EXTENDING_ROTATIONS){
            m_extensionMotor.stop();
        }

        if(IntakeConsts.DEBUG_MODE){
            log();
        }
    }

    private void log(){
        // log motor output and encoder values
    }
    
}
