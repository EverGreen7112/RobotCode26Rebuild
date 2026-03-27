package frc.robot.Subsystems.Conveyor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Conveyer extends SubsystemBase implements Consts.FeedAndConveyConsts{
    
    private static Conveyer m_instance = new Conveyer();

    private EverMotorController m_conveyingMotor;

    private Conveyer(){
        m_conveyingMotor = FeedAndConveyConsts.CONVEY_MOTOR;
    }

    public static Conveyer getInstance(){
        return m_instance;
    }

    public void startConveying(double speed){
        m_conveyingMotor.set(speed);
    }

    public void stopConveying(){
        m_conveyingMotor.set(0);

    }

    @Override
    public void periodic() {
        if(FeedAndConveyConsts.DEBUG_MOD){
            log();
        }
    }

    private void log(){
        SmartDashboard.putNumber("Conveying Motor Speed", m_conveyingMotor.get());
    }

    public boolean isConnected(){
        return Consts.FeedAndConveyConsts.CONVEY_MOTOR.isConnected();
    }
    
}
