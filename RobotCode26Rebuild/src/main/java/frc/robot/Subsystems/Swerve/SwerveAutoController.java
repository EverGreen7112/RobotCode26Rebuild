package frc.robot.Subsystems.Swerve;

import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Conveyor.AutoStopConveyCommand;
import frc.robot.Commands.Conveyor.ConveyIntakeCommand;
import frc.robot.Commands.Conveyor.ConveyToFeederCommand;
import frc.robot.Commands.Feeder.FeedCommand;
import frc.robot.Commands.Intake.AutoIntakePickupCommand;
import frc.robot.Commands.Intake.AutoIntakeStopPickup;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.IntakePickupCommand;
import frc.robot.Commands.Intake.RetractIntakeCommand;
import frc.robot.Commands.Shooter.ScoreCommand;

public class SwerveAutoController {

    public static boolean isRobotAligning = false;

    private static final PIDConstants TRANSLATION_PID =  new PIDConstants(5.0, 0.0, 0.0),
                                      ROTATION_PID = new PIDConstants(1.0, 0.0 ,0.0);
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3, 3, 1 * Math.PI, 4 * Math.PI);
    private static final double GOAL_END_VELOCITY = 0;

    private static SwerveAutoController m_instance = new SwerveAutoController();
    private SendableChooser<Command> m_autoChooser;
    private SendableChooser<Alliance> m_allianceChooser;

    private SwerveAutoController(){
        
        configureCommands(); //configure commands must be registered before the creation of any paths
        RobotConfig config = null;

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            SmartDashboard.putBoolean("couldnt load robot config, expect problems in auto", false);
        }

        AutoBuilder.configure(
            SwerveLocalizer.getInstance()::getCurrentPoint, 
            SwerveLocalizer.getInstance()::setCurrentPoint, 
            Swerve.getInstance()::getRobotOrientedSpeeds, 
            ((speeds, feedforwards) -> Swerve.getInstance().driveRobotOrientedBySpeeds(speeds)), 
            new PPHolonomicDriveController( 
                TRANSLATION_PID, 
                ROTATION_PID 
            ),
            config, 
            () -> { //flip path
              return  getAlliance() == DriverStation.Alliance.Red;
            },
            Swerve.getInstance()
        );

    
        m_allianceChooser.setDefaultOption("Red Alliance", Alliance.Red);
        m_allianceChooser.addOption("Blue Alliance", Alliance.Blue);

        // Set up the Auto Command

        m_autoChooser.setDefaultOption("Audition Auto", new PathPlannerAuto("auditionAuto"));

    }

    public static SwerveAutoController getInstance(){
        return m_instance;
    }

    public void addChoosersToDashboard(){
        SmartDashboard.putData("auto", m_autoChooser);
        SmartDashboard.putData("alliance", m_allianceChooser);
    }

    public Command getAutoCommand(){
        return m_autoChooser.getSelected();
    }

    public Alliance getAlliance(){
        return m_allianceChooser.getSelected();
    }
    

    public Command generateDriveToCommand(Pose2d waypoint){
        return AutoBuilder.pathfindToPose(waypoint, PATH_CONSTRAINTS, GOAL_END_VELOCITY);
    };
    
    public Command generateDriveToCommand(Pose2d waypoint, double goalEndVelocity){
        return AutoBuilder.pathfindToPose(waypoint, PATH_CONSTRAINTS, goalEndVelocity);
    };

    public void configureCommands(){

        ParallelCommandGroup m_shooterCommands = new ParallelCommandGroup(new ConveyToFeederCommand(), new FeedCommand(), new ScoreCommand());
        ParallelCommandGroup m_pickupCommands = new ParallelCommandGroup(new AutoIntakePickupCommand(), new ConveyIntakeCommand());
        ParallelCommandGroup m_stopPickupCommand = new ParallelCommandGroup(new AutoIntakeStopPickup(), new AutoStopConveyCommand());


        NamedCommands.registerCommand("openIntake", new IntakeCommand());

        NamedCommands.registerCommand("startIntake",m_pickupCommands);

        NamedCommands.registerCommand("stopIntake", m_stopPickupCommand);

        NamedCommands.registerCommand("scoring", m_shooterCommands);

    }
}