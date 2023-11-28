// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.StrafeBreakLegsCmd;
import frc.robot.commands.StrafeToTagCmd;
import frc.robot.commands.TurnToTagCmd;
import frc.robot.commands.zero;
import frc.robot.commands.Autos.Auto;
import frc.robot.commands.Autos.AutoFL;
import frc.robot.commands.Autos.AutoL;
import frc.robot.commands.Autos.challenge;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveSubsytem drivebase; 

  public final VisionSubsystem vision;

  CommandJoystick driverController = new CommandJoystick(1);

  XboxController driverXbox = new XboxController(0);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    drivebase = new SwerveSubsytem(new File(Filesystem.getDeployDirectory(), "neo"));
    vision = new VisionSubsystem();

    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driverXbox.getRightX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driverXbox.getRightY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          false);

    drivebase.setDefaultCommand(closedAbsoluteDrive);                                                 
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */ 
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(()->drivebase.resetOdometry(new Pose2d())));

    new JoystickButton(driverXbox, 2).whileTrue(new TurnToTagCmd(vision, drivebase));
    new JoystickButton(driverXbox, 3).whileTrue(new StrafeToTagCmd(vision, drivebase));
    new JoystickButton(driverXbox, 4).whileTrue(new StrafeBreakLegsCmd(vision, drivebase));

    // new JoystickButton(driverXbox, 1).
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Object autonChoice) {
    //return AutoFL.auto(drivebase);
    if (autonChoice.equals(1)) return new Auto(drivebase);
    else if (autonChoice.equals(2)) return new AutoFL(drivebase);
    else if (autonChoice.equals(3)) return new AutoL(drivebase);
    else if (autonChoice.equals(4)) return new challenge(drivebase);
    else return null;
  }

  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  } 
  
}
