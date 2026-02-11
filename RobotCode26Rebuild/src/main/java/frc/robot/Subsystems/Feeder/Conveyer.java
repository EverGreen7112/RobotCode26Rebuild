package frc.robot.Subsystems.Feeder;

import static edu.wpi.first.units.Units.Ounce;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Conveyer extends SubsystemBase {
    
    private static Conveyer m_instance = new Conveyer();

    private boolean m_isConveying = false; 
    private EverMotorController m_conveyingMotor;
    private final boolean LOG = false; 
    private final double CONVEYING_VEL = 0.5; // place holder

    private Conveyer(){
        m_conveyingMotor = new EverTalonFX(0);
        // diigus nigus why do that if we have the everkit why not use it
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
        double speed = m_isConveying ? CONVEYING_VEL : 0;
        if(speed != m_conveyingMotor.get()){
            m_conveyingMotor.set(speed);
        }

        if(LOG){
            log();
        }
    }

    private void log(){
        // log motor output
    }
    
}
