package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//GIT test
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.paths.*;
import java.util.Arrays;

import frc.robot.commands.DriveDist;
import frc.robot.commands.TurnToAngle;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.util.CalcPathToTarget;
import frc.robot.util.CrashTracker;
import frc.robot.util.DriveSignal;
import frc.robot.util.VisionMath;
import frc.robot.util.Gyro;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// The SubsystemManager handles logging and looping for all registered subsystems.
	// I think it would be better to have the SubsystemManager own the looper and control all interactions
	// with the subsystems, but for now this is OK.
	private SubsystemManager mSubsystemManager = null;
	private Looper mEnabledLooper = null;

	// The subsystem that manages the drive base.
	// Again, it would be better for SubsystemManager to control the interactions with the subsystem.
	public static Drive getDrive(){
		return Drive.getInstance();
	}
	public static Climb getClimb(){
		return Climb.getInstance();
	}
	private static PowerDistributionPanel m_PDP;
	public static PowerDistributionPanel getPDP(){
		return m_PDP;
	}
	 
	/**
	 * Constructor
	 */
	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	private boolean m_robotInit_loggedError = false;
	@Override
	public void robotInit() {
		
		try {
			CrashTracker.logRobotInit();
		    //m_PDP = new PowerDistributionPanel(0);
			// Create all subsystems and register them with the subsystem manager.
			mEnabledLooper = new Looper();
			mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance()));//,Climb.getInstance()));
		    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			// Initialize the Operator Interface
			OI.getInstance();
			CalcPathToTarget calcPathToTarget = new CalcPathToTarget();
			calcPathToTarget.calcPath(0.0);


			
		    //CameraServer.getInstance().startAutomaticCapture();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"robotInit");
			if ( m_robotInit_loggedError == false ) {
				m_robotInit_loggedError = true ;
				System.out.println("robotInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}

		double test0 = Robot.getDrive().getLeftEnc();
		double test1 = Robot.getDrive().getRightEnc();
/*

		double m_desiredDist = 2.0;//feet
			
			double halfTime = Math.sqrt(m_desiredDist/Constants.kMaxAcceleration);
			int numIntervals = (int)(100*halfTime)+1;
			halfTime = numIntervals * 0.01;
			double m_acceleration = m_desiredDist / (halfTime*halfTime);

			Segment[] path = new Segment[(numIntervals*2)+2];
			double yaw = Gyro.getYaw();
			double xMax=0;
			double vMax=0;
			for(int i = 0;i<numIntervals+1;i++){
				double v = m_acceleration * (i * 0.01);
				double x = 0.5 * m_acceleration * ((i * 0.01)*(i * 0.01));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i] = new Segment(0.01, 0, 0, x, v, m_acceleration, 0, yaw);
				xMax = x;
				vMax = v;
			}
			for(int i = 0;i<numIntervals+1;i++){
				double v = vMax - (m_acceleration * (i * 0.01));
				double x = xMax + (vMax*(i*0.01)) - (0.5 * m_acceleration * ((i * 0.01)*(i * 0.01)));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i+numIntervals+1] = new Segment(0.01, 0, 0, x, v, -m_acceleration, 0, yaw);
			}
			Trajectory m_leftTrajectory = new Trajectory(path);
			Trajectory m_rightTrajectory = new Trajectory(path);

			PathFollower follower = new PathFollower(m_leftTrajectory, m_rightTrajectory);

			follower.initialize();

			long[] timeArray = new long[100];
			int tpos = -1;
			timeArray[++tpos] = System.nanoTime();
			follower.execute();
			timeArray[++tpos] = System.nanoTime();
			follower.execute();
			timeArray[++tpos] = System.nanoTime();
			follower.execute();
			timeArray[++tpos] = System.nanoTime();
			System.out.println("step0: "+(timeArray[1]-timeArray[0])/1000000.0);
			System.out.println("step1: "+(timeArray[2]-timeArray[1])/1000000.0);
			System.out.println("step2: "+(timeArray[3]-timeArray[2])/1000000.0);
*/

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	private boolean m_disabledInit_loggedError = false;
	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();

			// Stop all subsystem loops.
			mEnabledLooper.stop();

			// Call stop() on all our registered Subsystems.
			mSubsystemManager.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledInit");
			if ( m_disabledInit_loggedError == false ) {
				m_disabledInit_loggedError = true;
				System.out.println("disabledInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Disabled mode.
	 */

	private boolean m_disabledPeriodic_loggedError = false;
	@Override
	public void disabledPeriodic() {
		try {
			long startTime = System.nanoTime();
			VisionMath vm = new VisionMath();
			vm.findRobotPos();
			double xRobot = vm.getRobotX();
			double yRobot = vm.getRobotY();
			SmartDashboard.putNumber("xRobot", xRobot);
			SmartDashboard.putNumber("yRobot", yRobot);
			
			double halfWheelBase = ((12* Constants.kWheelBaseFeet) / 2.0);//inches
			double m_hdg = Gyro.getYaw() * Math.PI / 180.0;
  	      	double m_xCenter = xRobot + (Constants.kCamToBumper * Math.cos(m_hdg));
        	double m_yCenter = yRobot + (Constants.kCamToBumper * Math.sin(m_hdg));
			double m_xRightCorner =    m_xCenter - (halfWheelBase*Math.sin(m_hdg));
			double m_xLeftCorner =     m_xCenter + (halfWheelBase*Math.sin(m_hdg));
			double m_yRightCorner =    m_yCenter + Math.signum(Gyro.getYaw()) * ((halfWheelBase*Math.cos(m_hdg)));
			double m_yLeftCorner =     m_yCenter - Math.signum(Gyro.getYaw()) * ((halfWheelBase*Math.cos(m_hdg)));
			double criticalHdg = Math.signum(m_yCenter) * -1.0 * Math.abs(Math.atan(Math.abs(m_yCenter) / (Math.abs(m_xCenter) - Constants.kVisionApproachDist)));//radians
			SmartDashboard.putNumber("m_xCenter", m_xCenter);
			SmartDashboard.putNumber("m_yCenter", m_yCenter);
			SmartDashboard.putNumber("m_xLeftCorner", m_xLeftCorner);
			SmartDashboard.putNumber("m_yLeftCorner", m_yLeftCorner);
			SmartDashboard.putNumber("m_xRightCorner", m_xRightCorner);
			SmartDashboard.putNumber("m_yRightCorner", m_yRightCorner);
			SmartDashboard.putNumber("criticalHeading", criticalHdg*180/Math.PI);

			//Robot.getDrive().setDesiredArc(radiusTurn, degreesToTurn);
			

			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledPeriodic");
			if ( m_disabledPeriodic_loggedError == false ) {
				m_disabledPeriodic_loggedError = true;
				System.out.println("disabledPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Autonomous mode.
	 * You can use it to set the subsystems up to run the autonomous commands.
	 */
	Command autonomousCommand;
	private boolean m_autonomousInit_loggedError = false;
	private static boolean mInTeleop = false;
	public static boolean getInTeleop(){
		return mInTeleop;
	}
	static int pathsRan = 0;
	public static int getPathsRan(){
		return pathsRan;
	}
	int delayCount;
	double radius,theta;
	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutonomousInit();
			// Start the subsystem loops.
			mEnabledLooper.start();
			getClimb().setDesiredDist(10);
			getClimb().startClimbing();
			//getClimb().startHold();

			VisionMath vm = new VisionMath();
			vm.findRobotPos();
			double xRobot = vm.getRobotX();
			double yRobot = vm.getRobotY();
			SmartDashboard.putNumber("xRobot", xRobot);
			SmartDashboard.putNumber("yRobot", yRobot);

			double halfWheelBase = ((12* Constants.kWheelBaseFeet) / 2.0);
			double hdg = -Gyro.getYaw() * Math.PI / 180.0;
			double camToBumper = 12;

			double xFrontCorner = (xRobot) - (halfWheelBase*Math.sin(hdg)) + (camToBumper * Math.cos(hdg));
			double yFrontCorner = (yRobot) + Math.signum(Gyro.getYaw()) * ((halfWheelBase*Math.cos(hdg)) + (camToBumper * Math.sin(hdg)));

			double xGoal = -24;
			double yGoal = -Constants.kWheelBaseFeet*12/2.0;
			double radiusTurn = Math.abs(xFrontCorner - xGoal)/Math.tan(hdg) + (yFrontCorner - yGoal);

			//Robot.getDrive().setDesiredArc(radiusTurn, -Gyro.getYaw());
			//Robot.getDrive().setDesiredArc(76, -Gyro.getYaw());
			//Robot.getDrive().startSimpleVisionDrive();
    		//Robot.getDrive().startPath();
			//Command autonomousCommand = new DriveDist(36);
			//Command autonomousCommand = new TurnToAngle(50);
			//autonomousCommand.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousInit");
			if ( m_autonomousInit_loggedError == false ) {
				m_autonomousInit_loggedError = true;
				System.out.println("autonomousInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Autonomous mode.
	 */
	private boolean m_autonomousPeriodic_loggedError = false;
	boolean done = false;
	@Override
	public void autonomousPeriodic() {
		try {
			allPeriodic();

			if(getDrive().getState() != DriveControlState.PATH_FOLLOWING){
				if(done==false){
					Robot.getDrive().startSimpleVisionDrive();
					done = true;
				}
			}
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousPeriodic");
			if ( m_autonomousPeriodic_loggedError == false ) {
				m_autonomousPeriodic_loggedError = true;
				System.out.println("autonomousPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Teleop mode.
	 * You can use it to set the subsystems up to run under operator control.
	 */
	private boolean m_teleopInit_loggedError = false;
	@Override
	public void teleopInit() {
		try {
			mInTeleop = true;
			CrashTracker.logTeleopInit();

			// Start the subsystem loops.
			mEnabledLooper.start();

			// Change the Drive subsystem to manual control.
			getDrive().setOpenLoop(DriveSignal.NEUTRAL);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopInit");
			if ( m_teleopInit_loggedError == false ) {
				m_teleopInit_loggedError = true;
				System.out.println("teleopInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Teleop mode.
	 */
	private boolean m_teleopPeriodic_loggedError = false;
	@Override
	public void teleopPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopPeriodic");
			if ( m_teleopPeriodic_loggedError == false ) {
				m_teleopPeriodic_loggedError = true;
				System.out.println("teleopPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Test mode.
	 * You can use it to set the subsystems up to run their self-test routines.
	 */
	private boolean m_testInit_loggedError = false;
	@Override
	public void testInit() {
		try {
			CrashTracker.logTestInit();

			// ===== TEMPORARY CODE - REMOVE THIS =====
	        mEnabledLooper.start();
	        getDrive().runTest();
	        // ========================================

	        // ... Start a separate thread that runs through the self-test for each registered subsystem.
	        // ... Create and manage the thread in SubsystemManager.

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testInit");
			if ( m_testInit_loggedError == false ) {
				m_testInit_loggedError = true;
				System.out.println("testInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Test mode.
	 */
	private boolean m_testPeriodic_loggedError = false;
	@Override
	public void testPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testPeriodic");
			if ( m_testPeriodic_loggedError == false ) {
				m_testPeriodic_loggedError = true;
				System.out.println("testPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}
	
	/*
	 * This is the method called periodically during every periodic mode.
	 * It runs all the logging methods, and then runs the WPI scheduler.
	 */
	public void allPeriodic() {
		mSubsystemManager.outputToSmartDashboard();
		mSubsystemManager.writeToLog();
		mEnabledLooper.outputToSmartDashboard();
		Scheduler.getInstance().run();
	}
}