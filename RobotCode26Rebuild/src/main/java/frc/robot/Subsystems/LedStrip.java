package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Conveyor.Conveyer;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.EverKit.Periodic;

public class LedStrip implements Periodic {

    private static final Distance LED_SPACING = Meters.of(1.0 / 120.0);

    private static LedStrip m_instance = new LedStrip();

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private LEDPattern m_ledPattern;

    public enum LedPattern{
        
        DEFAULT_COLOR(
            LEDPattern.solid(Color.kGreen)
        ),
        ERROR_SWERVE(
            LEDPattern.solid(Color.kRed)
        ),
        ERROR_INTAKE(
            LEDPattern.solid(Color.kCrimson)
        ),
        ERROR_CONVEYER(
            LEDPattern.solid(Color.kDeepPink)
        ),
        ERROR_FEEDER(
            LEDPattern.solid(Color.kChocolate)
        ),
        ERROR_SHOOTER(
            LEDPattern.solid(Color.kAliceBlue)
        ),
        ERROR_CAMS(
            LEDPattern.solid(Color.kBlue)
        ),
        ERROR_GYRO(
            LEDPattern.solid(Color.kPurple)
        ),
        ERROR_SWERVE_CANCODERS(
            LEDPattern.solid(Color.kYellow)
        ),
        INTAKE_EXTENDED(
            LEDPattern.solid(Color.kBrown)
        );
        
        public final LEDPattern pattern;

        private LedPattern(LEDPattern Pattern){
            this.pattern = Pattern;
        }
    }
    

    public LedStrip(){
        m_led = new AddressableLED(8);
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
        
        m_ledPattern = LedPattern.DEFAULT_COLOR.pattern;

        m_led.start();
    }

    public static LedStrip getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        
        
        // //error leds    
          if(!DriverStation.isEnabled()){
             if(!Swerve.getInstance().areMotorControllersConnected() ){
                 setLedPattern(LedPattern.ERROR_SWERVE);
             }
             else if(!Swerve.getInstance().areAbsEncodersConnected() ){
                 setLedPattern(LedPattern.ERROR_SWERVE_CANCODERS);
             }
             else if(!Swerve.getInstance().isGyroConnected() ){
                 setLedPattern(LedPattern.ERROR_GYRO);
             }
             else if(!Shooter.getInstance().areMotorControllersConnected() ){
                 setLedPattern(LedPattern.ERROR_SHOOTER);
             }
             else if(!Feeder.getInstance().isConnected()){
                 setLedPattern(LedPattern.ERROR_FEEDER);
             }
             else if(!Conveyer.getInstance().isConnected()){
                 setLedPattern(LedPattern.ERROR_CONVEYER);
             }
             else if(!Intake.getInstance().isConnected()){
                 setLedPattern(LedPattern.ERROR_INTAKE);
             }
             /*else if(!Localization.getInstance().areCamsConnected() ){
                 setLedPattern(LedPattern.ERROR_CAMS);*/
             }
             else if(Intake.getInstance().isOpen()){
                setLedPattern(null);
             }
             else{
                 setLedPattern(LedPattern.DEFAULT_COLOR);
             }

    }

    public void setLedPattern(LedPattern pattern){
        m_ledPattern = pattern.pattern;
        m_ledPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void initialize(){
        start(PeriodicTime.kRobotPeriodic);
    }
    
}