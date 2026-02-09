package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Climb extends SubsystemBase{
    
    private static Climb m_instance = new Climb();

    private EverMotorController m_motor;
    private EverEncoder m_encoder;

    private final double MAX_OPEN = 0, OPEN_VEL = 0.25; // place holder

    private boolean log = false;

    private Climb(){
        m_motor = new EverTalonFX(0);
        m_encoder = new EverCANCoder(0);
    }

    public static Climb getInstance(){
        return m_instance;
    }

    public void open(){
        m_motor.set(OPEN_VEL);
    }

    public void close(){
        m_motor.set(-OPEN_VEL);
    }

    public void stop(){
        m_motor.stop();
    }

    public boolean canOpen(){
        return m_encoder.getPos() < MAX_OPEN;
    }

    public boolean canClose(){
        return m_encoder.getPos() > 0;
    }

    @Override
    public void periodic() {
        if(!canOpen() && m_encoder.getVel() > 0)
            stop();
        else if(!canClose() && m_encoder.getVel() < 0)
            stop();

    }


}
