package frc.robot.Subsystems.Intake;

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

    private EverMotorController m_feederMotor, m_openingMotor;// feeding is a diffrent subsystem and extend smotor is better
    private EverEncoder m_openingEncoder;

    private DigitalInput m_closingLM, m_openingLM;//two thing first limit swtch doesnt realy have reason

    private final double OPEN_VEL = 0.25, FEEDING_VEL = 0.6;// speed is a scalar usually and velocity is two dimetional 

    private boolean m_state = true;// m_state is a bad name it doesnt clarify anything about itself

    private Intake(){
        m_feederMotor = new EverSparkFlex(0);
        m_openingMotor = new EverTalonFX(0);

        m_openingEncoder = new EverTalonFXInternalEncoder((EverTalonFX)m_openingMotor);// you see you had to convert this is because you dont use the everkit correctley

        m_closingLM = new DigitalInput(0);
        m_openingLM = new DigitalInput(0);
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void setState(boolean state){
        m_state = state;
    }

    @Override
    public void periodic() {
        double vel = m_state ? OPEN_VEL : -OPEN_VEL;
        m_openingMotor.set(vel);// what do you want to close everytime write functons for it

        if(vel < 0){
            m_feederMotor.stop();
            if(m_closingLM.get() && m_openingEncoder.getVel() < 0){
                m_openingMotor.stop();
            }
        }
        else{
            m_feederMotor.set(FEEDING_VEL);
            if(m_openingLM.get() && m_openingEncoder.getVel() > 0){
                m_openingMotor.stop();
            }
        }
    }
    // no functions for rollers
    
}
