package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration cancoderConfig;

    private double turningMotorSetpoint;
    private double turningMotorPIDOutput;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
                        int absoluteEncoderID, double absoluteEncoderOffsetDegree){
        absoluteEncoder = new CANcoder(absoluteEncoderID);
        cancoderConfig = new CANcoderConfiguration();

        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDegree;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.burnFlash();
        turningMotor.burnFlash();

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPIDController = new PIDController(SwerveModuleConstants.turningMotorkP, 0, 0);
        turningPIDController.enableContinuousInput(-180, 180);
        
        resetEncoders();
    }
   
    public double getDrivePosition(){
        return driveEncoder.getPosition()*SwerveModuleConstants.driveEncoderRot2Meter;
    }
    public double getTurningPosition(){
        return absoluteEncoder.getAbsolutePosition().getValue()*360;
    }
    public double getTurningEncoderPosition(){
        return turningEncoder.getPosition()*SwerveModuleConstants.turningEncoderRot2Rad;
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity()*SwerveModuleConstants.driveEncoderRPM2MeterPerSec;
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity()*SwerveModuleConstants.turningEncoderRPM2RadPerSec;
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
        absoluteEncoder.getConfigurator().apply(cancoderConfig);

    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    // setDesireState
    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        turningMotorSetpoint = state.angle.getDegrees();
        turningMotorPIDOutput = turningPIDController.calculate(getState().angle.getDegrees(), state.angle.getDegrees()); 
        turningMotor.set(turningMotorPIDOutput);
    }
    // Stop Motor
    public void stopModule(){
        driveMotor.set(0);
        turningMotor.set(0);
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("turningMotorSetpoint", turningMotorSetpoint);
        SmartDashboard.putNumber("turningMotorOutut", turningMotorPIDOutput);

    }
}