package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsytem;

public class zero extends CommandBase {

    SwerveSubsytem swerve;
    

    public zero(SwerveSubsytem swerve){
        this.swerve = swerve;
    }

    public void execute(){
        swerve.zeroGyro();
    }
    
}
