// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveController;
import frc.robot.subsystems.SwerveSubsytem;
import edu.wpi.first.math.controller.PIDController;

public class turnAndStrafeTagCmd extends CommandBase {
  /** Creates a new TurnToTagCmd. */
  private VisionSubsystem vision;
  private SwerveSubsytem swerve;

  private PIDController controllerx, controllerRotation;

  public turnAndStrafeTagCmd(VisionSubsystem vision, SwerveSubsytem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.swerve = swerve;

    controllerx = new PIDController(0.03, 0.02, 0.0015); 

    controllerx.setSetpoint(0);

    controllerRotation = new PIDController(0.04, 0.01, 0); 

    controllerRotation.setSetpoint(0);

    addRequirements(vision);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(vision.getArea()) != 0 && vision.getv()){
      swerve.drive(SwerveController.getTranslation2d(swerve.getTargetSpeeds(0, controllerx.calculate(vision.getX()), 0, 0)), -controllerRotation.calculate(vision.getRotation()), false , false);
     }
     else swerve.drive(new Translation2d(0, 0), 0, false , false);

     SmartDashboard.putBoolean("code run", true);
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
