package frc.robot.commands.DriveCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsytem;
import frc.robot.subsystems.VisionSubsystem;

import java.sql.Driver;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AprilTagDriveCmd extends CommandBase {

  private final SwerveSubsytem swerve;
  private final VisionSubsystem vision;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final boolean isOpenLoop;
  private final PIDController controller;

  public AprilTagDriveCmd(
    SwerveSubsytem swerve,
    VisionSubsystem vision,
    DoubleSupplier vX,
    DoubleSupplier vY,
    DoubleSupplier headingHorizontal,
    DoubleSupplier headingVertical,
    boolean isOpenLoop
  ) {
    this.swerve = swerve;
    this.vision = vision;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;

    controller = new PIDController(0.04, 0.01, 0); 

    controller.setSetpoint(0);

    addRequirements(swerve);
    addRequirements(vision);
  }

  @Override
  public void execute() {
    if (!DriverStation.isAutonomous()) {
    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
      vX.getAsDouble()/1,
      vY.getAsDouble()/1,
      headingHorizontal.getAsDouble(),
      headingVertical.getAsDouble()
    );

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation =
      SwerveMath.limitVelocity(
        translation,
        swerve.getFieldVelocity(),
        swerve.getPose(),
        Constants.LOOP_TIME,
        Constants.ROBOT_MASS,
        List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration()
      );

    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    if (vision.getv()){
        swerve.drive(translation, controller.calculate(vision.getX()),true, isOpenLoop);
    }else{
        swerve.drive(translation, headingHorizontal.getAsDouble()*2,true, isOpenLoop);
    }
        
    }
    // Make the robot move

    SmartDashboard.putNumber("x pos", swerve.getPose().getTranslation().getX());
    SmartDashboard.putNumber("y pos", swerve.getPose().getTranslation().getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
