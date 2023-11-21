package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsytem;
import java.sql.Driver;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive extends CommandBase {

  private final SwerveSubsytem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final boolean isOpenLoop;

  public AbsoluteDrive(
    SwerveSubsytem swerve,
    DoubleSupplier vX,
    DoubleSupplier vY,
    DoubleSupplier headingHorizontal,
    DoubleSupplier headingVertical,
    boolean isOpenLoop
  ) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    if (!DriverStation.isAutonomous()) {
    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
      vX.getAsDouble() / 1.5,
      vY.getAsDouble() / 1.5,
      headingHorizontal.getAsDouble(),
      headingVertical.getAsDouble()
    );

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(
      desiredSpeeds
    );
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

    if (
      Math.hypot(
        headingHorizontal.getAsDouble(),
        headingVertical.getAsDouble()
      ) <
      0.5
    ) {
      //swerve.SwerveController.lastAngleScalar = swerve.getHeading().getRadians();
      // desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble()/1.5, vY.getAsDouble()/1.5,
      // // 0.71, 0.71);
      // Math.cos(swerve.getHeading().getRadians()),
      // Math.sin(swerve.getHeading().getRadians()));
      // SmartDashboard.putBoolean("code run", true);
      swerve.drive(translation, 0, true, isOpenLoop);
    } else {
      swerve.drive(
        translation,
        desiredSpeeds.omegaRadiansPerSecond,
        true,
        isOpenLoop
      );
    }
    // Make the robot move

    }
    SmartDashboard.putNumber("x pos", swerve.getPose().getTranslation().getX());
    SmartDashboard.putNumber("y pos", swerve.getPose().getTranslation().getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
