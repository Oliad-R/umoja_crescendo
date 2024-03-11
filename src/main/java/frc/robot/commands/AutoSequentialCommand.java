package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSequentialCommand extends SequentialCommandGroup{
    Arm arm;
    Intake intake;
    SwerveSubsystem swerveSubsystem;
    SwerveJoystick sj;
    // Command autoShoot = new AutoShoot(arm, intake);
    public AutoSequentialCommand(SwerveSubsystem swerveSubsystem, Arm arm, Intake intake){
        addCommands(
            new AutoShoot(arm, intake).withTimeout(5),

            new SwerveJoystick(swerveSubsystem,
                () -> {
                    // if(swerveSubsystem.odometer.getPoseMeters().getX() < 1.4){
                        SmartDashboard.putNumber("ODOMETER X", swerveSubsystem.odometer.getPoseMeters().getX());
                        return 0.3;
                    // } else {
                    //     return 0.0;
                    // }
                },
                () -> {return 0.0;},
                () -> {return 0.0;}).withTimeout(4)
            
            // new SwerveJoystick(swerveSubsystem,
            //     () -> {
            //         if(swerveSubsystem.odometer.getPoseMeters().getX() > 1.37){
            //             SmartDashboard.putNumber("ODOMETER X", swerveSubsystem.odometer.getPoseMeters().getX());
            //             return -0.3;
            //         } else {
            //             return 0.0;
            //         }
            //     },
            //     () -> {return 0.0;},
            //     () -> {return 0.0;}).withTimeout(2),

            // new AutoShoot(arm, intake, 1).withTimeout(3)
        );

        // andThen(
        //     new SwerveJoystick(swerveSubsystem,
        //         () -> {
        //             if(swerveSubsystem.odometer.getPoseMeters().getX() > 0){
        //                 SmartDashboard.putNumber("ODOMETER X", swerveSubsystem.odometer.getPoseMeters().getX());
        //                 return 0.5;
        //             } else {
        //                 return 0.0;
        //             }
        //         },
        //         () -> {return 0.0;},
        //         () -> {return 0.0;})
        //     );
        // addCommands(new AutoDrive(swerveSubsystem, 2.5));
    }
}
