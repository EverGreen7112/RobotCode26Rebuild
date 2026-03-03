package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.ClimbConst;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;

public class Climb extends SubsystemBase implements Consts.ClimbConst{
    
    private static Climb m_instance = new Climb();

    private EverMotorController m_climbMotor;
    private EverEncoder m_climbEncoder;
    private DigitalInput m_bottomLimitSwitch, m_topLimitSwitch;

    private Climb(){
        m_climbMotor = ClimbConst.CLIMB_MOTOR;
        m_climbEncoder = ClimbConst.CLIMB_ENCODER;
        m_bottomLimitSwitch = ClimbConst.BOTTOM_LM;
        m_topLimitSwitch = ClimbConst.TOP_LM;
    }

    public static Climb getInstance(){
        return m_instance;
    }

    public void open(){
        m_climbMotor.set(ClimbConst.OPEN_VEL);
    }

    public void close(){
        m_climbMotor.set(-ClimbConst.OPEN_VEL);
    }

    public void stop(){
        m_climbMotor.stop();
    }

    private boolean cantOpen(){
        return m_topLimitSwitch.get();
    }

    private boolean cantClose(){
        return m_bottomLimitSwitch.get();
    }

    
    @Override
    public void periodic() {
        if(cantClose()){
            m_climbEncoder.setPos(0);
        }

        if((cantOpen() && m_climbEncoder.getVel() > 0) || (cantClose() && m_climbEncoder.getVel() < 0)){
            stop();
        }

        if(ClimbConst.DEBUG_MODE){
            log();
        }

    }

    //TODO: need to see if the condition is right for both limit switches

    private boolean isTopLmDisconnected(){
        return m_topLimitSwitch.get() && m_climbEncoder.getPos() < ClimbConst.MAX_OPEN;
    }

    private boolean isBottomLmDisconnected(){
        return m_bottomLimitSwitch.get() && m_climbEncoder.getPos() > 0;
    }

    private void log(){
        SmartDashboard.putBoolean("Climb Bottom Limit Switch", m_bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Climb Top Limit Switch", m_topLimitSwitch.get());
    }

}