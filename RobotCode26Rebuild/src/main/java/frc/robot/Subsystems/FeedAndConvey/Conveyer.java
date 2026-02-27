package frc.robot.Subsystems.FeedAndConvey;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Conveyer extends SubsystemBase {
    
    private static Conveyer m_instance = new Conveyer();

    private boolean m_isConveying; 
    private EverMotorController m_conveyingMotor;
    private Double m_lastSpeed;

    private Conveyer(){
        m_conveyingMotor = FeedAndConveyConsts.CONVEY_MOTOR;
        m_isConveying = false;
        m_lastSpeed = 0.0;
    }

    public static Conveyer getInstance(){
        return m_instance;
    }

    public void setConveying(boolean conveying){
        m_isConveying = conveying;
    }

    @Override
    public void periodic() {
        double speed = m_isConveying ? FeedAndConveyConsts.CONVEYING_SPEED : 0;
        if(speed != m_lastSpeed){
            m_conveyingMotor.set(speed);
        }

        m_lastSpeed = speed;

        if(FeedAndConveyConsts.DEBUG_MOD){
            log();
        }
    }

    private void log(){
        SmartDashboard.putBoolean("Is Conveying", m_isConveying);
        SmartDashboard.putNumber("Conveying Motor Speed", m_conveyingMotor.get());
    }
    
}
