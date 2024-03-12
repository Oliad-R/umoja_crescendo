package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
// import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turnPidController;

    public final CANcoder absoluteEncoder;

    public final int absoluteEncoderID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        config.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");

        this.absoluteEncoderID = absoluteEncoderId;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

        //driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        //turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        //absoluteEncoder.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 10);

        // driveMotor.restoreFactoryDefaults();
        // turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);
        
        driveEncoder = driveMotor.getEncoder(Type.kHallSensor, 42);
        turnEncoder = turnMotor.getEncoder(Type.kHallSensor, 42);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        turnPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        Timer.delay(2);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        resetTurn();
    }

    public void resetTurn(){
        double position = getAbsoluteEncoderRad();
        turnEncoder.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        SmartDashboard.putNumber("ID: " + absoluteEncoderID, Math.toDegrees(getTurningPosition()));
        SmartDashboard.putNumber("GOAL: " + absoluteEncoderID, Math.toDegrees(state.angle.getRadians()));
        SmartDashboard.putNumber("Set motor percent: " + absoluteEncoderID, turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        
        turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(getTurningPosition()));
      }
}