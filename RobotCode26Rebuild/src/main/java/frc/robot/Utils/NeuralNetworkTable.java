package frc.robot.Utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class NeuralNetworkTable implements Periodic {
    private static NeuralNetworkTable m_instance = new NeuralNetworkTable();

    private NetworkTableInstance m_nTInst;
    
    private NetworkTable m_nT;

    private BooleanPublisher m_transmitStart;
    private BooleanPublisher m_transmitStop;
    private BooleanPublisher m_transmitRestart;


    private DoublePublisher m_robotSpeedX; 
    private DoublePublisher m_robotSpeedY; 
    private DoublePublisher m_robotAngularSpeed;
    private DoublePublisher m_shooterSpeed; 
    private DoublePublisher m_shooterAngle; 
    private DoublePublisher m_feederSpeed;

    public enum transmitMode{
        Start,
        Stop,
        Restart
    }

    private EverMotorController m_motor;

    public NeuralNetworkTable(){

        m_nTInst = NetworkTableInstance.getDefault();
    
        m_nT = m_nTInst.getTable("shooter_table");

        m_transmitStart = m_nT.getBooleanTopic("transmit_start").publish();
        m_transmitStop = m_nT.getBooleanTopic("transmit_stop").publish();
        m_transmitRestart = m_nT.getBooleanTopic("transmit_restart").publish();


        m_robotSpeedX = m_nT.getDoubleTopic("robot_speed_x").publish();
        m_robotSpeedY = m_nT.getDoubleTopic("robot_speed_y").publish(); 
        m_robotAngularSpeed = m_nT.getDoubleTopic("robot_angular_speed").publish();
        m_shooterSpeed = m_nT.getDoubleTopic("shooter_speed").publish(); 
        m_shooterAngle = m_nT.getDoubleTopic("shooter_angle").publish(); 
        m_feederSpeed = m_nT.getDoubleTopic("feeder_speed").publish();

        m_nTInst.startClient4("7112");
        m_nTInst.setServer("7112");

        m_motor = new EverTalonFX(0);
    }

    public static NeuralNetworkTable getInstance(){
        return m_instance;
    }

    @Override
    public void periodic(){
        // for the mean time some of the stats will be hard coded into the code
        m_robotSpeedX.set(Swerve.getInstance().getRobotOrientedSpeeds().vxMetersPerSecond);
        m_robotSpeedY.set(Swerve.getInstance().getRobotOrientedSpeeds().vyMetersPerSecond);
        m_robotAngularSpeed.set(Swerve.getInstance().getAngularVelocity());
        m_shooterSpeed.set(6.2);
        m_shooterAngle.set(0);
        m_feederSpeed.set(0);
    }

    public void setTransmit(transmitMode action, boolean isButtonPressed){
        switch(action){
            case Start:
                m_transmitStart.set(isButtonPressed);
                m_motor.set(0.8);
                break;

            case Stop:
                m_transmitStop.set(isButtonPressed);
                m_motor.set(0);
                break;

            case Restart:
                m_transmitRestart.set(isButtonPressed);
                m_motor.set(0);
                break;
        }
    }
}
