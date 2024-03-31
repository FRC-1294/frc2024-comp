// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robots;

import frc.robot.swerve.KrakenSwerveModule;
import frc.robot.Util.PIDParameters;

/** Add your docs here. */
public class CompetitionBotSwerveConfig extends SwerveConfig{
         public CompetitionBotSwerveConfig() {
                super();

        }

        public void config() {
                TRANS_GEAR_RATIO_ROT = 1 / 6.12;
                ABS_ENC_GEAR_RATIO_ROT = 1;
      
                // Conversion Factors
                WHEEL_DIAMETER_METERS = .102;

      
                // Physical Max
                PHYSICAL_MAX_SPEED_MPS = 4.625;
        
                TELE_MAX_SPEED_MPS = 4.625;
                TELE_MAX_ROT_SPEED_RAD_SEC = 2 * Math.PI * 1.5;
        
                TELE_MAX_ACC_MPS2 = 5.0;
                TELE_MAX_ROT_ACC_RAD_SEC2 = 4 * Math.PI;

                TRACK_WIDTH_METERS = .495;
                TRACK_LENGTH_METERS = .495;

                PIGEON_ID = 25;

                // Inverse Booleans
                FRONT_LEFT_ROT_INVERSE = false;
                FRONT_LEFT_TRANS_INVERSE = false;

                FRONT_RIGHT_ROT_INVERSE = false;
                FRONT_RIGHT_TRANS_INVERSE = false;

                BACK_LEFT_ROT_INVERSE = false;
                BACK_LEFT_TRANS_INVERSE = false;

                BACK_RIGHT_ROT_INVERSE = false;
                BACK_RIGHT_TRANS_INVERSE = false;



                // PID Controllers

                FRONT_LEFT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.0,0.0); //TODO: https://github.com/FRC-1294/frc2024/issues/280
                FRONT_RIGHT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.0,0.0); //TODO: https://github.com/FRC-1294/frc2024/issues/280
                BACK_LEFT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.0,0.0); //TODO: https://github.com/FRC-1294/frc2024/issues/280
                BACK_RIGHT_TRANS_PID = new PIDParameters(0.1, 0, 0,0.0,0.0); //TODO: https://github.com/FRC-1294/frc2024/issues/280

                FRONT_LEFT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.1946*1); //TODO: https://github.com/FRC-1294/frc2024/issues/280
                FRONT_RIGHT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.1946*1);//TODO: https://github.com/FRC-1294/frc2024/issues/280
                BACK_LEFT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.1946*1);//TODO: https://github.com/FRC-1294/frc2024/issues/280       
                BACK_RIGHT_TRANS_CARPET_PID = new PIDParameters(0.1, 0, 0,0.01,0.1946*1);//TODO: https://github.com/FRC-1294/frc2024/issues/280
        }

        @Override
        public void initializeSwerveModules() { 
                                // Swerve Modules and Other Hardware
                FRONT_LEFT_MODULE = new KrakenSwerveModule(FRONT_LEFT_ROT_ID,
                                FRONT_LEFT_TRANS_ID, FRONT_LEFT_ROT_ENC_ID, FRONT_LEFT_ROT_INVERSE,
                                FRONT_LEFT_TRANS_INVERSE, FRONT_LEFT_ROT_PID,FRONT_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                FRONT_RIGHT_MODULE = new KrakenSwerveModule(FRONT_RIGHT_ROT_ID,
                                FRONT_RIGHT_TRANS_ID, FRONT_RIGHT_ROT_ENC_ID, FRONT_RIGHT_ROT_INVERSE,
                                FRONT_RIGHT_TRANS_INVERSE, FRONT_RIGHT_ROT_PID,FRONT_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                BACK_LEFT_MODULE = new KrakenSwerveModule(BACK_LEFT_ROT_ID,
                                BACK_LEFT_TRANS_ID, BACK_LEFT_ROT_ENC_ID, BACK_LEFT_ROT_INVERSE,
                                BACK_LEFT_TRANS_INVERSE, BACK_LEFT_ROT_PID,BACK_LEFT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);

                BACK_RIGHT_MODULE = new KrakenSwerveModule(BACK_RIGHT_ROT_ID,
                                BACK_RIGHT_TRANS_ID, BACK_RIGHT_ROT_ENC_ID, BACK_RIGHT_ROT_INVERSE,
                                BACK_RIGHT_TRANS_INVERSE, BACK_RIGHT_ROT_PID,BACK_RIGHT_TRANS_CARPET_PID,TRANS_GEAR_RATIO_ROT,
                                WHEEL_CIRCUMFERENCE_METERS,PHYSICAL_MAX_SPEED_MPS,ABS_ENC_GEAR_RATIO_ROT,REL_ENC_GEAR_RATIO_ROT);
        }



}
