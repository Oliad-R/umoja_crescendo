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

        addCommands(new SwerveJoystick(swerveSubsystem,
        () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverRotAxis)));

        addCommands(new ArmJoystick(arm,intake,climber,operatorController));
    }
}
