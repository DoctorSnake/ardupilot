

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


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_InertialSensor ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;

AP_AHRS_DCM  ahrs(ins, baro, gps);


float time_count;
static uint16_t last_print;


void setup()
{
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

    time_count = 0;

  }


  void loop()
  {
      pilotage();
  }


  void pilotage(){

    uint16_t now = hal.scheduler->millis();
    float heading = 0;

    time_count = time_count + (now-last_print)/1.0e3;
    last_print = now;

    static float yaw_target = 0;
    ins.wait_for_sample();

    // Ask MPU6050 for orientation
    //ins.update();
    gps.update();
    ahrs.update();

    float roll, pitch, yaw;

    roll = ToDeg(ahrs.roll) ;
    pitch = ToDeg(ahrs.pitch) ;
    yaw = ToDeg(ahrs.yaw) ;

    // Ask MPU6050 for gyro data
    //Vector3f gyro = ins.get_gyro();
    Vector3f gyro = ahrs.get_gyro();
    Vector3f drift  = ahrs.get_gyro_drift();

    float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);

  // %4.1f,  %4.1f, %4.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f,
  hal.console->printf("%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f \n",
  roll,
  pitch,
  yaw,
  time_count,
  gyroRoll,
  gyroPitch,
  gyroYaw
  );



  } // Fin fonction pilote

  AP_HAL_MAIN();
