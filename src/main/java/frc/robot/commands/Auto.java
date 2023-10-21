package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsytem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.PIDConstants;
import frc.robot.Constants.Auton;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import frc.robot.Constants;
import swervelib.parser.PIDFConfig;


public class Auto  {
    public static CommandBase auto(SwerveSubsytem swerve){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Line", new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
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
      );
      return Commands.sequence(autoBuilder.fullAuto(path));
    }
    
}
