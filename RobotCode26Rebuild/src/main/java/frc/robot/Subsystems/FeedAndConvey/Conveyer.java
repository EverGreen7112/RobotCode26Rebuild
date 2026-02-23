package frc.robot.Subsystems.FeedAndConvey;

import static edu.wpi.first.units.Units.Ounce;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Conveyer extends SubsystemBase {
    
    private static Conveyer m_instance = new Conveyer();

    private boolean m_isConveying = false; 
    private EverMotorController m_conveyingMotor;

    private Conveyer(){
        m_conveyingMotor = FeedAndConveyConsts.CONVEY_MOTOR;
    }

    public static Conveyer getInstance(){
        return m_instance;
    }

    public boolean getIsConveying(){
        return m_isConveying;
    }

    public void setConveying(boolean conveying){
        m_isConveying = conveying;
    }

    @Override
    public void periodic() {
        double speed = m_isConveying ? FeedAndConveyConsts.CONVEYING_VEL : 0;
        if(speed != m_conveyingMotor.get()){
            m_conveyingMotor.set(speed);
        }

        if(FeedAndConveyConsts.DEBUG_MOD){
            log();
        }
    }

    private void log(){
        // log motor output
    }
    
}
