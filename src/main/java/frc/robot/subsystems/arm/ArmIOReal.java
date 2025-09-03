// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.OurRobotState;

public class ArmIOReal implements ArmIO {
    private final TalonFX m_pivotMotor = new TalonFX(pivotMotorId);
    private final TalonFX m_gripperMotor = new TalonFX(gripperMotorId);
    private final DigitalInput m_gripperSensor = new DigitalInput(gripperSensorId);

    private final StatusSignal<Angle> m_pivotMotorPositionSignal = m_pivotMotor.getPosition();

    private final NeutralOut m_neutralRequest = new NeutralOut();

    private final DutyCycleOut m_gripperDutyCycleRequest = new DutyCycleOut(0d);

    private final MotionMagicVoltage m_pivotPositionRequest = new MotionMagicVoltage(pivotPositionGrab);

    private boolean m_gripperCoralOn = false;

    public ArmIOReal() {
        var pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = pivotNeutralMode;
        pivotConfig.MotorOutput.Inverted = pivotInverted;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = pivotCurrentLimit;
        pivotConfig.Feedback.SensorToMechanismRatio = pivotMotorReduction;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kP = pidP;
        pivotConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;

        var gripperConfig = new TalonFXConfiguration();
        gripperConfig.MotorOutput.NeutralMode = gripperNeutralMode;
        gripperConfig.MotorOutput.Inverted = gripperInverted;

        gripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripperConfig.CurrentLimits.SupplyCurrentLimit = gripperCurrentLimit;

        tryUntilOk(5, () -> m_pivotMotor.getConfigurator().apply(pivotConfig));
        tryUntilOk(5, () -> m_gripperMotor.getConfigurator().apply(gripperConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_pivotMotorPositionSignal);

        m_pivotMotor.setPosition(pivotPositionGrab);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_pivotMotorPositionSignal);
    }

    @Override
    public void periodic() {
        boolean gripperSensorTripped = !m_gripperSensor.get();
        if (m_gripperCoralOn && gripperSensorTripped)
            gripperOff();

        // TODO: we probably only set this here when it's true; then, when we score a piece (press score button / set state / etc) it becomes false
        OurRobotState.setIsCoralInGripper(gripperSensorTripped);
    }

    @Override
    public void pivotGoPosition(double position) {
        m_pivotMotor.setControl(m_pivotPositionRequest.withPosition(position));
    }

    @Override
    public boolean pivotIsAtPosition(double position) {
        return Math.abs(position - m_pivotMotorPositionSignal.getValueAsDouble()) <= pivotIsAtPositionThreshold;
    }

    @Override
    public void gripperCoralOn() {
        m_gripperCoralOn = true;
        m_gripperMotor.setControl(m_gripperDutyCycleRequest.withOutput(gripperCoralOutput));
    }
    @Override
    public void gripperAlgaeOn() {
        m_gripperMotor.setControl(m_gripperDutyCycleRequest.withOutput(gripperAlgaeOutput));
    }

    @Override
    public void gripperOff() {
        m_gripperCoralOn = false;
        m_gripperMotor.setControl(m_neutralRequest);
    }
}
