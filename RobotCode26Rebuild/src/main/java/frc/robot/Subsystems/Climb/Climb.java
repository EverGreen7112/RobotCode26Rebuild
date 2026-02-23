package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Climb extends SubsystemBase{
    
    private static Climb m_instance = new Climb();

    private EverMotorController m_ClimbMotor;
    private EverEncoder m_ClimbEncoder;
    private DigitalInput m_BottomLimitSwitch, m_TopLimitSwitch;

    private Climb(){
        m_ClimbMotor = ClimbConst.CLIMB_MOTOR;
        m_ClimbEncoder = ClimbConst.CLIMB_ENCODER;
        m_BottomLimitSwitch = ClimbConst.BOTTOM_LM;
        m_TopLimitSwitch = ClimbConst.TOP_RM;
    }

    public static Climb getInstance(){
        return m_instance;
    }

    public void open(){
        m_ClimbMotor.set(ClimbConst.OPEN_VEL);
    }

    public void close(){
        m_ClimbMotor.set(-ClimbConst.OPEN_VEL);
    }

    public void stop(){
        m_ClimbMotor.stop();
    }

    public boolean cantOpen(){
        return !m_TopLimitSwitch.get();
    }

    public boolean cantClose(){
        return m_BottomLimitSwitch.get();
    }

    private void log(){
        SmartDashboard.putBoolean("Climb Bottom Limit Switch", m_BottomLimitSwitch.get());
        SmartDashboard.putBoolean("Climb Top Limit Switch", m_TopLimitSwitch.get());
    }

    @Override
    public void periodic() {
        if((cantOpen() && m_ClimbEncoder.getVel() > 0) || (cantClose() && m_ClimbEncoder.getVel() < 0))
            stop();

        if(ClimbConst.DEBUG_MODE){
            log();
        }

    }


}