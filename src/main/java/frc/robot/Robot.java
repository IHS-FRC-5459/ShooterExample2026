// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.LinkedList;
import java.util.Queue;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private final CANBus canbus = new CANBus("rio");
  private final TalonFX m_fx = new TalonFX(16, canbus);
  private final TalonFX m_fllr = new TalonFX(29, canbus);
  private final TalonFX indexer = new TalonFX(14, canbus);
  private final SparkMax hoodController = new SparkMax(26, MotorType.kBrushless);
  private final Encoder hoodEncoder = new Encoder(7,8); 
  private final ArmFeedforward hoodFeedforward = new ArmFeedforward(0, 0.4, 0);
  private final PIDController hoodPID = new PIDController(1, 0.05, 0);
  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final XboxController m_joystick = new XboxController(1);

  private final Mechanisms m_mechanisms = new Mechanisms();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_fllr.setControl(new Follower(m_fx.getDeviceID(), MotorAlignmentValue.Opposed));
    hoodEncoder.setDistancePerPulse(0.02);
    //This happends to be about encoder dist = degrees of hood
    hoodEncoder.reset();
  }

  @Override
  public void robotPeriodic() {
    m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
    Logger.recordOutput("Encoder actual:  ", -hoodEncoder.getDistance());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}
  Queue<Double> encoderQ = new LinkedList<>();
  @Override
  public void teleopPeriodic() {
    double joyValue = m_joystick.getLeftY();
    // encoderQ.add(hoodEncoder.getDistance());
    // if(encoderQ.size() > 5){
    //   encoderQ.remove();
    // }
    // double total = 0;
    // for(double val : encoderQ){
    //   total += val;
    // }
    // total = total / encoderQ.size();

    // if(Math.abs(hoodGoal - reading) > 5){
    //   if(reading < hoodGoal){
    //     hoodController.set(MathUtil.clamp(hoodGoal - reading, 0, 0.01));
    //   }else{
    //     hoodController.set(MathUtil.clamp(hoodGoal - reading, -0.001, 0));
    //   }
    // }else{
    //   hoodController.set(0);
    // }
    double reading = -hoodEncoder.getDistance();
    double readingInRad = reading * Math.PI / 180;
    hoodPID.setSetpoint(10 * Math.PI / 180);
    Logger.recordOutput("Setpoint", hoodPID.getSetpoint());
    double pidVolts = hoodPID.calculate(readingInRad); 
    Logger.recordOutput("Rate", hoodEncoder.getRate());
    double ffVolts = 0;//hoodFeedforward.calculate(readingInRad, hoodEncoder.getRate());
    double volts = pidVolts + ffVolts;
    Logger.recordOutput("pid volts: ",  pidVolts);
    Logger.recordOutput("ffVolts ",  ffVolts);
    hoodController.setVoltage(volts);
    Logger.recordOutput("Volts",  volts);

    if (Math.abs(joyValue) < 0.1) joyValue = 0;

    double desiredRotationsPerSecond = joyValue * 90; // Go for plus/minus 50 rotations per second
    desiredRotationsPerSecond = 42;
    if (m_joystick.getLeftBumperButton()) {
      /* Use velocity voltage */
      System.out.println("Rot per sec"+ desiredRotationsPerSecond);
      m_fx.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
     // m_fllr.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
    } else if (m_joystick.getRightBumperButton()) {
      /* Use velocity torque */
      m_fx.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
    } else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
    }
    if(m_joystick.getAButton()){
      indexer.set(0.4);
    }else{
          indexer.set(0);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
