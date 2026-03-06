package frc.robot.Utils.EverKit.Implementations.Encoders;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Utils.EverKit.EverAbsEncoder;

public class EverDutyCycleEncoder extends EverAbsEncoder {

    private DutyCycleEncoder m_dutyCycleEncoder;
    private double m_offSet;
    private double m_posConversionFactor;
    // offset measured internally in percentage but give externally in degrees for convince
    public EverDutyCycleEncoder(int port){
        m_dutyCycleEncoder = new DutyCycleEncoder(port);
        m_offSet = 0;
        m_posConversionFactor = 1.0;
    }

    @Override
    public double getAbsPos() {
        return (m_dutyCycleEncoder.get() - m_offSet) * 360 * m_posConversionFactor;
    }

    @Override
    public double getOffset() {
        return m_offSet * 360.0;
    }

    @Override
    public void setOffset(double offset) {
        m_offSet = offset / 360.0;
    }

    @Override
    public boolean isConnected() {
        return m_dutyCycleEncoder.isConnected();
    }

    @Override
    public double getPos() {
        return (m_dutyCycleEncoder.get() - m_offSet) * 360;
    }

    //under contruction
    @Override
    public void setPos(double pos) {
    }


    //velocity under contruction coming soon
    @Override
    public double getVel() {
        return 0;
    }

    @Override
    public void setPosConversionFactor(double factor) {
        m_posConversionFactor = factor;
    }

    //velocity under contruction coming soon
    @Override
    public void setVelConversionFactor(double factor) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelConversionFactor'");
    }
    
}