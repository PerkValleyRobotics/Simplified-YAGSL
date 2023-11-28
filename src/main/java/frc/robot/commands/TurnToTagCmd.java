// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import edu.wpi.first.math.controller.PIDController;

public class TurnToTagCmd extends CommandBase {
  /** Creates a new TurnToTagCmd. */
  private VisionSubsystem vision;
  private SwerveSubsytem swerve;

  private PIDController controller;

  public TurnToTagCmd(VisionSubsystem vision, SwerveSubsytem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.swerve = swerve;

    controller = new PIDController(0.08, 0, 0); 

    controller.setSetpoint(0);

    addRequirements(vision);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(vision.getX()) > 0.05 && vision.getv()){
      swerve.drive(new Translation2d(0, 0), controller.calculate(vision.getX()), true , false);
     }
     else swerve.drive(new Translation2d(0, 0), 0, true , false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
