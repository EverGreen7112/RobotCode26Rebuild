// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ResetRobotCommand;
import frc.robot.Commands.Conveyor.ConveyIntakeCommand;
import frc.robot.Commands.Conveyor.ConveyToFeederCommand;
import frc.robot.Commands.Feeder.FeedCommand;
import frc.robot.Commands.Shooter.DeliverCommand;
import frc.robot.Commands.Shooter.ScoreCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand;
import frc.robot.Commands.Swerve.ManualDrive.TeleopDriveCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand.SpeedMode;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
public class RobotContainer {

  private static final int CHASSIS_PORT = 0;
  private static final int OPERATOR_PORT = 1;


  //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);
  public static final CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);

  //Triggers
  public static final Trigger operatorA = operator.a();
  public static final Trigger operatorB = operator.b();
  public static final Trigger operatorX = operator.x();
  public static final Trigger operatorY = operator.y();
  public static final Trigger operatorPovUp = operator.povUp();
  public static final Trigger operatorPovRight = operator.povRight();
  public static final Trigger operatorRB = operator.rightBumper();
  public static final Trigger operatorLB = operator.leftBumper();
  public static final Trigger operatorRT = operator.rightTrigger();
  public static final Trigger operatorLT = operator.leftTrigger();
  public static final Trigger operatorStart = operator.start();

  public static final Trigger chassisStart = chassis.start();
  public static final Trigger chassisBack = chassis.back();
  public static final Trigger chassisA = chassis.a();
  public static final Trigger chassisB = chassis.b();
  public static final Trigger chassisX = chassis.x();
  public static final Trigger chassisRT = chassis.rightTrigger();
  public static final Trigger chassisLT = chassis.leftTrigger();
  public static final Trigger chassisPovUp = chassis.povUp();
  public static final Trigger chassisPovDown = chassis.povDown();
  public static final Trigger chassisPovLeft = chassis.povLeft();
  public static final Trigger chassisPovRight = chassis.povRight();
  //public static final TeleopDriveCommand teleopCommand = new TeleopDriveCommand(chassis::getLeftY, chassis::getLeftX, chassis::getRightX);


  private ParallelCommandGroup m_shooterCommands = new ParallelCommandGroup(new ConveyToFeederCommand(), new FeedCommand());
  
  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
  }

  private void registerNamedCommands(){
  
  }

  private void configureBindings() {

    //chassis
    //Swerve.getInstance().setDefaultCommand(teleopCommand);
    chassisRT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kTurbo));
    chassisLT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kSlow));
    chassisBack.onTrue(new InstantCommand(() -> Swerve.getInstance().resetGyro()));


    chassisA.whileTrue(new ConveyToFeederCommand());
    chassisB.whileTrue(new FeedCommand());

    //chassisX.whileTrue(new DeliverCommand());


  }

  
}