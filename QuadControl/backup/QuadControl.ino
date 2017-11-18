

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_NavEKF.h>
#include <AP_HAL_Linux.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>

#include <PID.h>

#define MODE_PILOTAGE 1

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_InertialSensor ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;

AP_AHRS_DCM  ahrs(ins, baro, gps);

// Radio min/max values for each stick for my radio (worked out at beginning of article)

#define RC_THR_MIN   1174
#define RC_THR_MAX   1830
#define RC_YAW_MIN   1150
#define RC_YAW_MAX   1914
#define RC_PIT_MIN   1215
#define RC_PIT_MAX   1853
#define RC_ROL_MIN   1150
#define RC_ROL_MAX   190
#define RC_MODE_MIN  1010
#define RC_MODE_MAX  2030

// Motor numbers definitions
#define MOTOR_FL   2    // Front left #3
#define MOTOR_FR   0    // Front right #1
#define MOTOR_BL   1    // back left #2
#define MOTOR_BR   3    // back right #4

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino constrain function
float constrain(float x, float x_min, float x_max)
{
  if(x < x_min){
    return x_min;
  }
  else if(x > x_max){
    return x_max;
  }
  else{
    return x;
  }
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

uint16_t channels[8];

uint16_t time;
static uint16_t last_print;
uint16_t mot_out_fl, mot_out_bl, mot_out_fr, mot_out_br;


void calibration(void){

  long thrust_cmd = hal.rcin->read(2);

  hal.rcout->write(MOTOR_FR, thrust_cmd);
  hal.rcout->write(MOTOR_FL, thrust_cmd);
  hal.rcout->write(MOTOR_BR, thrust_cmd);
  hal.rcout->write(MOTOR_BL, thrust_cmd);
}


void pilotage(){

  uint16_t now = hal.scheduler->millis();
  float heading = 0;

  time = time + (now-last_print)/1.0e3;
  last_print = now;

  static float yaw_target = 0;
  // Wait until new orientation data (normally 5ms max)
  //while (ins.num_samples_available() == 0);
  ins.wait_for_sample();

  uint16_t channels[8];

  // Read RC transmitter and map to sensible values
  hal.rcin->read(channels, 8);

  uint16_t rcthr;

  int rcyaw, rcpit, rcroll;  // Variables to store radio in

  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);

  // Ask MPU6050 for orientation
  ins.update();
  gps.update();
  ahrs.update();

  int roll, pitch, yaw;
  //ins.quaternion.to_euler(&roll, &pitch, &yaw);
  /*roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = ToDeg(yaw) ;*/

  roll = ToDeg(ahrs.roll) ;
  pitch = ToDeg(ahrs.pitch) ;
  yaw = ToDeg(ahrs.yaw) ;
  // hal.console->printf("%d, %d, %d \n",  roll, pitch, yaw);

  // Ask MPU6050 for gyro data
  //Vector3f gyro = ins.get_gyro();
  Vector3f gyro = ahrs.get_gyro();
  Vector3f drift  = ahrs.get_gyro_drift();

  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);

  /*hal.console->printf("roll = %f, pitch = %f, yaw = %f",
  gyroRoll, gyroPitch, gyroYaw);
  hal.console->println();  */
  float pitch_stab_output = 0;
  float roll_stab_output = 0;
  float yaw_stab_output = 0;

  // Do the magic
  if(rcthr > RC_THR_MIN + 200) {  // Throttle raised, turn on stablisation.
    // Stablise PIDS
    /*
    pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250);
    roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
    */
    yaw_target = rcyaw;

    pitch_stab_output = pids[PID_PITCH_STAB].get_pid(rcpit - pitch, 1);
    roll_stab_output = pids[PID_ROLL_STAB].get_pid(rcroll - roll, 1);
    yaw_stab_output = pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1);


  long pitch_output =  (long) pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1);
  long roll_output =  (long) pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1);
  long yaw_output =  (long) pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1);

  // mix pid outputs and send to the motors.

  long pitch_total = pitch_output;
  long roll_total = roll_output;
  //long yaw_total = yaw_output + yaw_stab_output;
  long yaw_total =  yaw_output;

  // VTAIL CONF
  /*
  mot_out_fl = rcthr + roll_total + pitch_total - 0;
  mot_out_bl = rcthr + 0 - pitch_total + yaw_output;
  mot_out_fr = rcthr - roll_total + pitch_total + 0;
  mot_out_br = rcthr - 0 - pitch_total - yaw_output; */


  // QUAD CLASSIC
  mot_out_fl = rcthr + roll_total + pitch_total - yaw_total;
  mot_out_bl = rcthr + roll_total - pitch_total + yaw_total;
  mot_out_fr = rcthr - roll_total + pitch_total + yaw_total;
  mot_out_br = rcthr - roll_total - pitch_total - yaw_total;

  /*hal.console->printf("MOT1 calc %d, MOT1 cmd %d, MOT2 calc %d, MOT2 cmd %d, MOT3 calc %d, MOT3 cmd %d,MOT4 calc %d, MOT4 cmd %d, time %5.1f \n",
  rcthr + roll_total + pitch_total - yaw_total,
  mot_out_fl,
  rcthr + roll_total - pitch_total + yaw_total,
  mot_out_bl,
  rcthr - roll_total + pitch_total + yaw_total,
  mot_out_fr,
  rcthr - roll_total - pitch_total - yaw_total,
  mot_out_br,
  time);*/

  hal.rcout->write(MOTOR_FL, mot_out_fl);
  hal.rcout->write(MOTOR_BL, mot_out_bl);
  hal.rcout->write(MOTOR_FR, mot_out_fr);
  hal.rcout->write(MOTOR_BR, mot_out_br);

} else {
  // motors off

  mot_out_fl = rcthr;
  mot_out_bl = rcthr;
  mot_out_fr = rcthr;
  mot_out_br = rcthr;

  hal.rcout->write(MOTOR_FL, mot_out_fl);
  hal.rcout->write(MOTOR_BL, mot_out_bl);
  hal.rcout->write(MOTOR_FR, mot_out_fr);
  hal.rcout->write(MOTOR_BR, mot_out_br);

  // reset PID integrals whilst on the ground
  for(int i=0; i<6; i++)
  pids[i].reset_I();

}

// %4.1f,  %4.1f, %4.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f,
hal.console->printf("%d, %d, %d, %d, %d, %d, %d, %u, %u, %u, %u, %5.1f \n",
rcroll,
rcpit,
rcthr,
rcyaw,
roll,
pitch,
yaw,
mot_out_fl,
mot_out_bl,
mot_out_fr,
mot_out_br,
time);


} // Fin fonction pilote

void setup()
{
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);

  //hal.rcout->enable_mask(0xFF);
  for (int i=0; i<4; i++) {
    hal.rcout->enable_ch(i);
  }

    #if MODE_PILOTAGE == 1
    // PID Configuration
    pids[PID_PITCH_RATE].kP(3);
    pids[PID_PITCH_RATE].kI(1);
    pids[PID_PITCH_RATE].imax(50);

    pids[PID_ROLL_RATE].kP(3);
    pids[PID_ROLL_RATE].kI(1);
    pids[PID_ROLL_RATE].imax(50);

    pids[PID_YAW_RATE].kP(0); //3
    pids[PID_YAW_RATE].kI(0); //1
    pids[PID_YAW_RATE].imax(50); //50

    pids[PID_PITCH_STAB].kP(6);
    pids[PID_ROLL_STAB].kP(6);
    pids[PID_YAW_STAB].kP(0); //6

    // ajout des integrateurs en attitude

    //pids[PID_PITCH_STAB].kI(0.05);
    //pids[PID_ROLL_STAB].kI(0.05);
    //pids[PID_YAW_STAB].kI(0.05);

    /*
    pids[PID_PITCH_STAB].imax(15);
    pids[PID_ROLL_STAB].imax(15);
    pids[PID_YAW_STAB].imax(15);*/

    ins.init(AP_InertialSensor::COLD_START,
    AP_InertialSensor::RATE_100HZ);

    ins.init_accel();
    ahrs.init();

    gps.init(NULL);

      // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
    hal.scheduler->suspend_timer_procs();  // stop bus collisions
      //ins.dmp_init();
    hal.scheduler->resume_timer_procs();

      // We're ready to go! Now over to loop()
    last_print = hal.scheduler->micros();

      //hal.console->printf("Roll cmd (°), Pitch cmd (°), Yaw cmd (°), Roll (°), Pitch (°), Yaw (°), drift.x, drift.y, drift.z, Roll rate cmd (°/s), Pitch rate cmd (°/s), Yaw rate cmd (°/s), gyro.x, gyro.y, gyro.z, MOT FL, MOT BL, MOT FR, MOT BR, Temps \n");


    time = 0;
    #endif

  }


  void loop()
  {
    #if MODE_PILOTAGE == 0
      calibration();
    #elif MODE_PILOTAGE == 1
      pilotage();
    #endif


  }


  AP_HAL_MAIN();
