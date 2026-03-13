package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ResetRobotCommand;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.IntakeConsts;
import frc.robot.Utils.EverKit.EverAnalogToDigitalLimitSwitch;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;

public class Intake extends SubsystemBase implements Consts.IntakeConsts{

    private static Intake m_instance = new Intake();

    private EverMotorController m_pickupMotor, m_extensionMotor;
    private EverEncoder m_extensionEncoder;

    private EverAnalogToDigitalLimitSwitch m_retractionLM, m_extensionLM; 

    private Intake(){

        m_pickupMotor = IntakeConsts.PICKUP_MOTOR;
        m_extensionMotor = IntakeConsts.EXTENSION_MOTOR;
        m_extensionEncoder = IntakeConsts.EXTENSION_ENCODER;

        // m_retractionLM = IntakeConsts.RETRACTION_LM;
        // m_extensionLM = IntakeConsts.EXTENSION_LM;
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

    public boolean isClosed(){
        return m_retractionLM.get();
    }

    @Override
    public void periodic() {

        if(ResetRobotCommand.resetting){
            startRetracting();
        }

        // if((m_extensionLM.get() && m_extensionMotor.get() > 0) || (m_retractionLM.get() && m_extensionMotor.get() < 0)){
        //     stopExtending();
        // }

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
        SmartDashboard.putBoolean("Is Extending", m_extensionMotor.get() > 0);
        SmartDashboard.putBoolean("Is Retracting", m_extensionMotor.get() < 0);
        SmartDashboard.putNumber("Extension Encoder Position", m_extensionEncoder.getPos());
        SmartDashboard.putBoolean("Extension Limit Switch", m_extensionLM.get());
        SmartDashboard.putBoolean("Retraction Limit Switch", m_retractionLM.get());
    }
    
}
