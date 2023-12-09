// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Auton;
import frc.robot.commands.DriveCmds.AbsoluteDrive;
import frc.robot.subsystems.SwerveSubsytem;

public class AutoCMD extends CommandBase {
  /** Creates a new AutoCMD. */
  private SwerveSubsytem swerve;
  private PathPlannerTrajectory trajectory;
  Command runAutonomousPathCommand;
  private SwerveAutoBuilder autoBuilder;
  public AutoCMD(SwerveSubsytem swerve, PathPlannerTrajectory trajectory) {
    this.swerve = swerve;
    this.trajectory = trajectory;
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    
    runAutonomousPathCommand =  new SwerveAutoBuilder(
      swerve::getPose,
// Pose2d supplier
      swerve::resetOdometry,
// Pose2d consumer, used to reset odometry at the beginning of auto
      new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
// PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
// PID constants to correct for rotation error (used to create the rotation controller)
      swerve::setChassisSpeeds,
// Module states consumer used to output to the drive subsystem
      eventMap,
      false,
// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      swerve
// The drive subsystem. Used to properly set the requirements of path following commands
  ).fullAuto(trajectory);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    runAutonomousPathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     runAutonomousPathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runAutonomousPathCommand.end(interrupted);

    new AbsoluteDrive(swerve, ()->0,()->0,()->Math.cos(swerve.getHeading().getRadians()),()->Math.sin(swerve.getHeading().getRadians()),false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return runAutonomousPathCommand.isFinished();


  }
}
