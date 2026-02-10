package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;

public class Shoter extends SubsystemBase{

    private static Shoter m_instance = new Shoter();

    private EverMotorController m_leftMotor, m_rigthMotor, m_angleMotor;
    private EverMotorControllerGroup m_shoting;
    private EverEncoder m_encoder;
    private DigitalInput m_topLM, m_buttomLM;   
    private final double SHOTER_GEAR_RATIO = 0; 
    private double m_targetAngle, m_targetVel;

    private EverPIDController m_anglePID, m_shotingPID;
 

    private Shoter(){
        m_leftMotor = new EverTalonFX(0);
        m_rigthMotor = new EverTalonFX(0);
        m_shoting = new EverMotorControllerGroup(m_rigthMotor, m_leftMotor);

        m_angleMotor = new EverSparkMax(0);
        m_encoder = new EverSparkInternalEncoder((EverSparkMax)m_angleMotor);

        m_topLM = new DigitalInput(0);
        m_buttomLM = new DigitalInput(0); // change all place holders

        m_anglePID = new EverSparkMaxPIDController((EverSparkMax)m_angleMotor);
        m_shotingPID = new EverTalonFXPIDController((EverTalonFX)m_leftMotor);
    }

    public static Shoter getInstance(){
        return m_instance;
    }

    public double getCurrentAngle(){
        return (m_encoder.getPos() * 360) / SHOTER_GEAR_RATIO;
    }

    public double getCurrentVel(){
        return m_targetVel;
    }

    public void setShotingAngle(double targetAngle){
        m_targetAngle = targetAngle;
    }

    public void setShotingVel(double targetVel){
        m_targetVel = targetVel;
    }

    @Override
    public void periodic() {
        if((m_topLM.get() || m_buttomLM.get()) && Math.abs(m_shoting.get()) > 0)
            m_shoting.stop();
        m_shotingPID.activate(m_targetVel, ControlType.kVel);
        m_anglePID.activate(m_targetAngle, ControlType.kPos);
    }

    


    
}
