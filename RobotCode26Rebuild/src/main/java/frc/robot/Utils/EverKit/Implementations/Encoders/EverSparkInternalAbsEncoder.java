package frc.robot.Utils.EverKit.Implementations.Encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class EverSparkInternalAbsEncoder extends EverAbsEncoder {

    private EverSparkMax m_controller; 
    private AbsoluteEncoder m_encoder;
    
    private double m_posFactor = 1.0;
    private double m_velFactor = 1.0;
    private double m_offSet = 0.0;

    public EverSparkInternalAbsEncoder(EverSparkMax controller){
        m_controller = controller;
        m_encoder = m_controller.getControllerInstance().getAbsoluteEncoder();
    }

    @Override
    public double getAbsPos() {
        // Since we can't set the factor on the hardware, 
        // we multiply the raw rotations by our factor here.
        return m_encoder.getPosition() * m_posFactor;
    }

    @Override
    public double getOffset() {
        return m_offSet * 360;
    }

    @Override
    public void setOffset(double offset) {
        m_offSet = offset / 360;
    }

    @Override
    public void setPosConversionFactor(double factor) {
        m_posFactor = factor;
    }

    @Override
    public void setVelConversionFactor(double factor) {
        m_velFactor = factor;
    }

    @Override
    public boolean isConnected() {
        // A simple check to see if the SparkMax is alive on the CAN bus
        return m_controller.getControllerInstance().getBusVoltage() > 0;
    }

    @Override
    public double getPos() {
        return (m_encoder.getPosition() - m_offSet) * m_posFactor;
    }

    //dont use now...
    @Override
    public void setPos(double pos) {
        return;
    }

    @Override
    public double getVel() {
        return m_encoder.getVelocity() * m_velFactor;
    }
}