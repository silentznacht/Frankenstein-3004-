// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

        public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                absoluteEncoder = new AnalogInput(absoluteEncoderId);

                driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
                turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
                
                driveMotor.setInverted(driveMotorReversed);
                turningMotor.setInverted(turningMotorReversed);

                driveEncoder = driveMotor.getEncoder();
                turningEncoder = turningMotor.getEncoder();

                // Get encoders from motors and set conversion factors
                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

                // Initialize PID controller for turning
                turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                turningPidController.enableContinuousInput(-Math.PI, Math.PI); // system is circular
                resetEncoders();
          }

        // Method to get drive position
        public double getDrivePosition() {
            return driveEncoder.getPosition();
        }
        
        // Method to get turning position
        public double getTurningPosition() {
            return turningEncoder.getPosition();
        }
        
        // Method to get drive velocity
        public double getDriveVelocity() {
            return driveEncoder.getVelocity();
        }
        
        // Method to get turning velocity
        public double getTurningVelocity() {
            return turningEncoder.getVelocity();
        }

        // Method to get absolute encoder position in radians
        public double getAbsoluteEncoderRad() {
            /*
            * retrieves the voltage reading from an absolute encoder sensor, converts it to an angle in radians, 
            * adjusts it for any offset, and potentially reverses its direction based on the configuration, resulting 
            * in the absolute angular position of the encoder.
            */
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
            angle *= 2.0 * Math.PI;
            angle -= absoluteEncoderOffsetRad;
            return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        }

        // Method to reset encoders
        public void resetEncoders() {
            driveEncoder.setPosition(0);
            turningEncoder.setPosition(getAbsoluteEncoderRad());
        }

        // Method to get the state of the module
        public SwerveModuleState getState() {
              // new Rotation2d(getTurningPosition()): This part creates a new Rotation2d object representing the current position of the turning motor. 
              // getDriveVelocity(): This method retrieves the current velocity of the drive motor.
              // Rotation2d object representing the current position of the turning motor.
              return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        }

        // Method to set the desired state of the module
        public void setDesiredState(SwerveModuleState state) {

            /*
            * This part checks if the absolute value of the desired speed (in meters per second) is less than 0.001. If it is, it means 
            * that the desired speed is close to zero, so the method calls the stop() method (which sets both drive and turning motors' 
            * speeds to zero) and then returns, exiting the method early.
            */
              if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                  stop();
                  return;
              }
              state = SwerveModuleState.optimize(state, getState().angle);
              driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
              turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
              SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        }

        // Method to stop the module
        public void stop() {
              driveMotor.set(0);
              turningMotor.set(0);
        }
    
}
