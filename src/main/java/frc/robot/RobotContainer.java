// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;

// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.commands.ArmJoystick;
import frc.robot.commands.AutoSequentialCommand;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TeleCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static Arm arm = new Arm();
  public final static Intake intake = new Intake();
  public final static Climber climber = new Climber();
  public final static Joystick driverController = new Joystick(USB.DRIVER_CONTROLLER);
  public final static Joystick operatorController = new Joystick(USB.OPERATOR_CONTROLLER);
  public final static PhotonCamera camera = new PhotonCamera("photonvision");

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

        NamedCommands.registerCommand("AutoAdjust", 
          new SwerveJoystick(RobotContainer.swerveSubsystem, 
            () -> {return 0.0;},
            () -> {return 0.0;},
            () -> {return 0.0;},
            true)
            .withTimeout(2)
            .andThen(() -> System.out.println("DONE")));

        NamedCommands.registerCommand("initShoot",
          new AutoShoot(RobotContainer.arm, RobotContainer.intake).withTimeout(4)
        );

        NamedCommands.registerCommand("autoAim",
          new AutoShoot(RobotContainer.arm, RobotContainer.intake, Constants.AutoShootState.AIM, Constants.AutoShootState.SHOOT).withTimeout(1.5)
        );

        NamedCommands.registerCommand("autoShoot",
          new AutoShoot(RobotContainer.arm, RobotContainer.intake, Constants.AutoShootState.AIM).withTimeout(2)
        );

        NamedCommands.registerCommand("lowerArm",
          new AutoShoot(RobotContainer.arm, RobotContainer.intake, 3).withTimeout(1.5)
        );

        NamedCommands.registerCommand("reverseIntake",
          new ReverseIntake(RobotContainer.intake).withTimeout(0.1)
        );

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // swerveSubsystem.setDefaultCommand(new TeleCommandGroup(swerveSubsystem,arm,intake,climber,driverController, operatorController));

    // Configure the button bindings
    configureBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new AutoCommandGroup(swerveSubsystem, arm, intake, climber);
    // return null;
    
    return autoChooser.getSelected();
    // return new SwerveJoystick(swerveSubsystem, () -> {return 0.0;},() -> {return 0.0;},() -> {return 0.0;},true);
    // return new AutoShoot(arm, intake);
    // return new AutoSequentialCommand(swerveSubsystem, arm, intake);
        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(0.075, 0),
        //                 new Translation2d(-0.075, 0)),
        //         new Pose2d(-0.25, 0.01, Rotation2d.fromDegrees(0)),
        //         trajectoryConfig);

        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         trajectory,
        //         swerveSubsystem::getPose,
        //         DriveConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         swerveSubsystem::setModuleStates,
        //         swerveSubsystem);

        //5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}