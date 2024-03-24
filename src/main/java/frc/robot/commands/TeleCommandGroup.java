package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleCommandGroup extends ParallelCommandGroup{
    public TeleCommandGroup(SwerveSubsystem swerveSubsystem, Arm arm, Intake intake, Climber climber, Joystick driverController, Joystick operatorController){
        
        //Driver Command
        addCommands(
            new SwerveJoystick(
                swerveSubsystem,
                () -> Math.pow(-driverController.getRawAxis(OIConstants.kDriverYAxis), 3),
                () -> Math.pow(-driverController.getRawAxis(OIConstants.kDriverXAxis), 3),
                () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis),
                false
            )
        );

        //Operator Command
        addCommands(
            new ArmJoystick(arm,intake,climber,operatorController)
        );
    }
}
