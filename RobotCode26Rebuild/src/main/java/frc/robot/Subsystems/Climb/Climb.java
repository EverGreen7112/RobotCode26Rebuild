package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Climb extends SubsystemBase{
    
    private static Climb m_instance = new Climb();

    private EverMotorController m_ClimbMotor;
    private EverEncoder m_ClimbEncoder;

    private final double MAX_OPEN = 0, OPEN_VEL = 0.25; // place holder

    private boolean log = false;

    private Climb(){
        m_ClimbMotor = new EverTalonFX(0);
        m_ClimbEncoder = new EverCANCoder(0);
    }

    public static Climb getInstance(){
        return m_instance;
    }

    public void open(){
        m_ClimbMotor.set(OPEN_VEL);
    }

    public void close(){
        m_ClimbMotor.set(-OPEN_VEL);
    }

    public void stop(){
        m_ClimbMotor.stop();
    }

    public boolean canOpen(){
        return m_ClimbEncoder.getPos() < MAX_OPEN;
    }

    public boolean canClose(){
        return m_ClimbEncoder.getPos() > 0;
    }

    private void log(){
        // log encoder position and velocity
    }

    @Override
    public void periodic() {
        if((!canOpen() && m_ClimbEncoder.getVel() > 0) || (!canClose() && m_ClimbEncoder.getVel() < 0))
            stop();

        if(log){
            log();
        }

    }


}