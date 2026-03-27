// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Shooter.ScoreCommand;
import frc.robot.Subsystems.AutoOperationsController;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.ClimbConst;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
import frc.robot.Subsystems.Consts.IntakeConsts;
import frc.robot.Subsystems.Consts.ShooterConsts;
import frc.robot.Subsystems.Consts.SwerveConsts;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.DeltaTime;
//import frc.robot.Utils.GamePieceDetector;
import frc.robot.Utils.EverKit.Periodic;
//import frc.robot.Utils.GamePieceCamera.GamePieceType
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.Math.Vector2d;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();
  private RobotContainer m_robotContainer;
  private SwerveLocalizer m_Localizer = SwerveLocalizer.getInstance();
  private Field2d m_field = new Field2d();
  private UsbCamera m_camera;


  
  // Adjust this to the actual number of LEDs on your strips


  public static Alliance m_alliance;
  

    public Robot() { 
      m_robotContainer = new RobotContainer();
    }
  
    @Override
    public void robotInit() {
      SmartDashboard.putData("Field", m_field);
      m_robotContainer = new RobotContainer();
      SwerveLocalizer.getInstance().initialize();
      SwerveAutoController.getInstance().addChoosersToDashboard();

      m_camera = CameraServer.startAutomaticCapture();

        // 2. Set basic settings (Optional but helpful)
      m_camera.setResolution(900, 900); // Keep it low to save bandwidth
      m_camera.setFPS(30);


    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SwerveLocalizer.getInstance().periodic();
    SmartDashboard.putBoolean("setAlliance", SmartDashboard.getBoolean("setAlliance", true));
    if(SmartDashboard.getBoolean("setAlliance", true))
        m_alliance = Alliance.Blue;
    else      
        m_alliance = Alliance.Red;

    m_field.setRobotPose(SwerveLocalizer.getInstance().getCurrentPoint());
    SmartDashboard.putData("Field", m_field);

    //Shooter.getInstance().ConfigureAllianceShootingSetting(m_alliance == Alliance.Blue);
    //AutoOperationsController.getInstance().setAlliance(m_alliance == Alliance.Blue);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_autonomousCommand = SwerveAutoController.getInstance().getAutoCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      
      m_autonomousCommand.cancel();
    }


  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}