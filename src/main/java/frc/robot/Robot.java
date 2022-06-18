// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sim.QuadSwerveSim;
import frc.sim.SwerveModuleSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  final double SIM_TS = 0.020;
  final double WHEEL_RAD_M = Units.inchesToMeters(1.5);
  final double TEST_PLANT_MASS_RADPERSEC2_PER_N = 2.0 / 90.0;

  final Vector2d TEST_DIR_XHAT = new Vector2d(1, 0);
  final Vector2d TEST_DIR_YHAT = new Vector2d(0, 1);

  final int FL = 0;
  final int FR = 1;
  final int BL = 2;
  final int BR = 3;
  final int NUM_MOD = 4;

  ArrayList<PWMVictorSPX> wheelMotors = new ArrayList<PWMVictorSPX>(NUM_MOD);
  ArrayList<PWMVictorSPX> azmthMotors = new ArrayList<PWMVictorSPX>(NUM_MOD);
  ArrayList<Encoder> wheelEncoders = new ArrayList<Encoder>(NUM_MOD);
  ArrayList<Encoder> azmthEncoders = new ArrayList<Encoder>(NUM_MOD);
  ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(NUM_MOD);
  ArrayList<PIDController> wheelControllers = new ArrayList<PIDController>(NUM_MOD);
  ArrayList<PIDController> azmthControllers = new ArrayList<PIDController>(NUM_MOD);

  QuadSwerveSim sim;

  Field2d field = new Field2d();

  private SwerveModuleSim moduleFactory() {
    return new SwerveModuleSim(
        DCMotor.getCIM(1),
        DCMotor.getCIM(1),
        WHEEL_RAD_M,
        130,
        9.0,
        1.0,
        1.0,
        1.1,
        0.8,
        16.0,
        0.001);
  }

  private void initHardware() {

    wheelMotors.add(new PWMVictorSPX(2));
    wheelMotors.add(new PWMVictorSPX(3));
    wheelMotors.add(new PWMVictorSPX(4));
    wheelMotors.add(new PWMVictorSPX(5));

    azmthMotors.add(new PWMVictorSPX(6));
    azmthMotors.add(new PWMVictorSPX(7));
    azmthMotors.add(new PWMVictorSPX(8));
    azmthMotors.add(new PWMVictorSPX(9));

    wheelEncoders.add(new Encoder(4, 5));
    wheelEncoders.add(new Encoder(6, 7));
    wheelEncoders.add(new Encoder(8, 9));
    wheelEncoders.add(new Encoder(10, 11));

    azmthEncoders.add(new Encoder(12, 13));
    azmthEncoders.add(new Encoder(14, 15));
    azmthEncoders.add(new Encoder(16, 17));
    azmthEncoders.add(new Encoder(18, 19));

    azmthEncoders.forEach(enc -> enc.setDistancePerPulse(1.0 / 4096.0));
    wheelEncoders.forEach(enc -> enc.setDistancePerPulse(1.0 / 128.0));

    for (int idx = 0; idx < NUM_MOD; idx++) {
      modules.add(moduleFactory());
    }

    sim = new QuadSwerveSim(0.75, 0.75, 64.0, 1.0, modules);

  }

  private void closeHardware() {
    wheelMotors.forEach(m -> {
      m.close();
    });
    azmthMotors.forEach(m -> {
      m.close();
    });
    wheelEncoders.forEach(m -> {
      m.close();
    });
    azmthEncoders.forEach(m -> {
      m.close();
    });
  }

  @Override
  public void robotInit() {
    initHardware();
    for (int idx = 0; idx < NUM_MOD; idx++) {
      wheelControllers.add(new PIDController(3, 0, 0));
      azmthControllers.add(new PIDController(8, 0, 0));
    }

    // Set up sensor simulation
    var wheelEncoderSim = new ArrayList<EncoderSim>(NUM_MOD);
    var azmthEncoderSim = new ArrayList<EncoderSim>(NUM_MOD);
    wheelEncoders.forEach(enc -> wheelEncoderSim.add(new EncoderSim(enc)));
    azmthEncoders.forEach(enc -> azmthEncoderSim.add(new EncoderSim(enc)));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    wheelControllers.forEach(ctrl -> ctrl.setSetpoint(100));
    field.setRobotPose(sim.getCurPose());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    sim.update(SIM_TS);
    SmartDashboard.putData("field", field);
  }
}
