package frc.robot.Sims;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.utils.ConstantsSRXDriveTrain;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class SimpleDifferentialDriveSim extends TimedRobot {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.381 * 2; // meters
    private static final double kWheelRadius = 0.0508; // meters
     private static final int kEncoderResolution = 4096;

     private final MotorController m_leftLeader = new WPI_TalonSRX(4);
     private final MotorController m_leftFollower = new WPI_TalonSRX(3); 
    private final MotorController m_rightLeader = new WPI_TalonSRX(2);
     private final MotorController m_rightFollower = new WPI_TalonSRX(1);

     private final Encoder m_leftEncoder = new Encoder(5, 6);
    private final Encoder m_rightEncoder = new Encoder(7, 8);

     private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

     private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
     private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());


    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);



    
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final Field2d m_fieldSim = new Field2d();
    private static final double KVlinear = 0;
    private static final double KAlinear = 0;
    private static final double KVAngular = 0;
    private static final double KAAngular = 0;
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(KVlinear, KAlinear, KVAngular, KAAngular);
    private DifferentialDrivetrainSim m_DifferentialDrivetrainSimulator = new DifferentialDrivetrainSim
            (m_drivetrainSystem,
             DCMotor.getCIM(2),
              8, kTrackWidth,
               kWheelRadius,
                null);
       
  public SimpleDifferentialDriveSim(){
         m_rightGroup.setInverted(true); 
         m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_leftEncoder.reset();
        m_rightEncoder.reset();
         m_rightGroup.setInverted(true);

         SmartDashboard.putData("Field", m_fieldSim);
       }
        /* FEED FORWARD IS NOT SET UP YET */
        //m_leftGroup.setVoltage(leftOutput +leftFeedforward);
        //m_rightGroup.setVoltage(rightOutput + rightFeedForward);

        public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
         var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
         var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
          double leftOutput =
              m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
          double rightOutput =
              m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
      
          m_leftGroup.setVoltage(leftOutput + leftFeedforward);
          m_rightGroup.setVoltage(rightOutput + rightFeedforward);
       }

       public void drive(double xSpeed, double rot){
        setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
       }
       public void updateOdometry(){
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
       }
       public void resetOdometry(Pose2d pose){
          m_leftEncoder.reset();
          m_rightEncoder.reset();
          m_DifferentialDrivetrainSimulator.setPose(pose);
          m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
       }



       public void simulationPeriodic(){
        m_DifferentialDrivetrainSimulator.setInputs(m_leftGroup.get() * RobotController.getInputVoltage(),
                                                   m_rightGroup.get() * RobotController.getInputVoltage());
        m_DifferentialDrivetrainSimulator.update(0.02);
        
        m_leftEncoderSim.setDistance(m_DifferentialDrivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_DifferentialDrivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_DifferentialDrivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_DifferentialDrivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_DifferentialDrivetrainSimulator.getHeading().getDegrees());

       }



    

}