// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter.FlywheelShooter;
import frc.robot.util.LimelightAimer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_offset = 3;
  private int m_loopCount = 1;
  private final LimelightAimer aimer = new LimelightAimer();
  private final FlywheelShooter shooter = new FlywheelShooter();

  private static final int STRIPE_WIDTH = 3; // Number of LEDs of each color in the scrolling pattern

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    // Initialize the LED strip on PWM port 6
    // Must be a PWM header — NOT MXP or DIO
    m_led = new AddressableLED(7);

    // Create a buffer matching the number of LEDs on your strip
    // Setting length is expensive — only do this once!
    m_ledBuffer = new AddressableLEDBuffer(82);
    m_led.setLength(82);
    
    // Apply solid red and other colors using LEDPattern
    
    // Push the (currently empty/black) buffer to the strip
    m_led.setData(m_ledBuffer);

    // Start continuously writing output to the LED strip
    m_led.start();


    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // Update the scrolling pattern every loop
  //   setScrollingColors(255, 255, 255,   // White
  //                       0, 0, 255);  // Blue

  //   m_led.setData(m_ledBuffer);

  //   // Advance the offset to move the pattern forward
  //   // Increase this number to make it scroll faster
  // // Advance offset every 3 loops to keep scroll speed feeling natural
  //       m_loopCount++;
  //       if (m_loopCount >= 3) {
  //           m_offset = (m_offset + 1) % (STRIPE_WIDTH * 2);
  //           m_loopCount = 0;
  //       }

    CommandScheduler.getInstance().run();

    // Anything else to run periodically that we setup manually
    m_robotContainer.runRobotPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (shooter.getRPMError() < 150 && aimer.ValidTarget()) {
      BlueAndWhiteLED();
    } else if(aimer.getDistance(0) > 5.5 && aimer.getDistance(0) < 8.5) {
      GreenLED();
    } else if (aimer.ValidTarget()) {
      BlueLED();
    } else {
      RedLED();
    }
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

    // Show in the dashboard whether robot is in range of the Hub
    dashboardDisplayInRangeOfHub();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (shooter.getRPMError() < 150 && aimer.ValidTarget()) {
      BlueAndWhiteLED();
    } else if(aimer.getDistance(0) > 5.5 && aimer.getDistance(0) < 9.5) {
      GreenLED();
    } else if (aimer.ValidTarget()) {
      BlueLED();
    } else {
      RedLED();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  
  private void dashboardDisplayInRangeOfHub() {
    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");
    double limelightMountAngleDegrees = 25.0; // TODO: measure this
    double limelightLensHeightInches = 20.0; // TODO: measure this
    double hubATHeightInches = getCurrentHub().getY();

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    double distanceToGoalInches = (hubATHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    boolean inRangeOfHub = distanceToGoalInches < 120; // TODO: tune this
    // SmartDashboard.putBoolean("In Range of Hub", inRangeOfHub);
  }

  private Translation2d getCurrentHub() {
    return DriverStation.getAlliance().isPresent()
    ? (
      DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
       ? Constants.FieldConstants.HubBlue
       : Constants.FieldConstants.HubRed
    )
    : Constants.FieldConstants.HubBlue; // Default to blue hub if alliance is not present (e.g. in testing)
  }

  private void setScrollingColors(
        int r1, int g1, int b1,
        int r2, int g2, int b2) {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Divide LEDs into groups of STRIPE_WIDTH and alternate colors
        int stripeIndex = (i + m_offset) % (STRIPE_WIDTH * 2);

        if (stripeIndex < STRIPE_WIDTH) {
            m_ledBuffer.setRGB(i, r1, g1, b1); // First color
        } else {
            m_ledBuffer.setRGB(i, r2, g2, b2); // Second color
        }
    }
  }

  private void BlueAndWhiteLED(){
    setScrollingColors(255, 255, 255,   // White
                        0, 0, 255);  // Blue

    m_led.setData(m_ledBuffer);

    // Advance the offset to move the pattern forward
    // Increase this number to make it scroll faster
    // Advance offset every 3 loops to keep scroll speed feeling natural
    m_loopCount++;
    if (m_loopCount >= 3) {
      m_offset = (m_offset + 1) % (STRIPE_WIDTH * 2);
      m_loopCount = 0;
    }
  }

  private void RedLED(){
    setScrollingColors(255, 0, 0,
                          255, 0, 0);

    m_led.setData(m_ledBuffer);

    // Advance the offset to move the pattern forward
    // Increase this number to make it scroll faster
    // Advance offset every 3 loops to keep scroll speed feeling natural
    m_loopCount++;
    if (m_loopCount >= 3) {
      m_offset = (m_offset + 1) % (STRIPE_WIDTH * 2);
      m_loopCount = 0;
    }
  }

  private void BlueLED(){
    setScrollingColors(0, 0, 255,
                          0, 0, 255);

    m_led.setData(m_ledBuffer);

    // Advance the offset to move the pattern forward
    // Increase this number to make it scroll faster
    // Advance offset every 3 loops to keep scroll speed feeling natural
    m_loopCount++;
    if (m_loopCount >= 3) {
      m_offset = (m_offset + 1) % (STRIPE_WIDTH * 2);
      m_loopCount = 0;
    }
  }

    private void GreenLED(){
      setScrollingColors(0, 255, 0,
                            0, 255, 0);

    m_led.setData(m_ledBuffer);

    // Advance the offset to move the pattern forward
    // Increase this number to make it scroll faster
    // Advance offset every 3 loops to keep scroll speed feeling natural
    m_loopCount++;
    if (m_loopCount >= 3) {
        m_offset = (m_offset + 1) % (STRIPE_WIDTH * 2);
        m_loopCount = 0;
    }
  }
}

