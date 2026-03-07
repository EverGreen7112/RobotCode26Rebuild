package frc.robot.Utils.EverKit;

import edu.wpi.first.wpilibj.AnalogInput;

public class EverAnalogToDigitalLimitSwitch {
    private AnalogInput m_analogInput;
    private double m_digitalThreshHold;

    public EverAnalogToDigitalLimitSwitch(int port){
        m_analogInput = new AnalogInput(port);
    }

    /**
     * get analog sensor as digital
     * @return 
     */
    public boolean get(){
        return m_analogInput.getAverageVoltage() > m_digitalThreshHold;
    }

    public double getAnalog(){
        return m_analogInput.getAverageVoltage();
    }

    public double getDigitalThresh(){
        return m_digitalThreshHold;
    }

    public void setDigitalThreshold(double digitalThreshold){
        m_digitalThreshHold = digitalThreshold;
    }
}
