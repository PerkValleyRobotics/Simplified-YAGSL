package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

public class SwerveSubsytem extends SubsystemBase{

    private final SwerveDrive swerveDrive;


    public SwerveSubsytem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }

        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry,this::getRobotVelocity, this::setChassisSpeeds, Constants.Auton.HPFConfig, this);
        
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isopenloop){
      swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        isopenloop); // Open loop is disabled since it shouldn't be used most of the time.
    }



    public SwerveDriveKinematics getKinematics(){
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose){
      swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }
        
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
      swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
      swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake(boolean brake){
      swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
      return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
      return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
      swerveDrive.lockPose();
    }

    public Rotation2d getPitch(){
      return swerveDrive.getPitch();
    }

    public void addFakeVisionReading(){
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), true, 4);
    }
    
    
}







        


    
