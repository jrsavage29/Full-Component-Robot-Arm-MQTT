//
//#include <robot_arm_fsm.h>
//
//// Sources:
//// - https://stackoverflow.com/questions/26883698/accessing-the-elements-of-a-char
//// - https://cboard.cprogramming.com/c-programming/38507-double-string-conversion.html
//
//// FSM subroutine.
//void FSMsubroutine()
//{
//    typedef enum {Idle, Active} FSMcurrentState;
//
//    static FSMcurrentState FSMstate = Idle;
//    static int taskStatus = IDLE_MODE;
//    int static tempholder = 0;
//    int static firstRun = 1;
//    firstRead = 1;
//
//    switch(FSMstate)
//    {
//        case Idle:
//        {
//           if(taskStatus == IDLE_MODE)
//           {
//               FSMstate = Idle;
//               taskStatus = IDLE_MODE;
//               setStatusofConfiguration(taskStatus);
//               //Do something. Maybe default servo settings I guess
//
//           }
//
//           if(peek_Read_Angle_Queue())
//          {
//              taskStatus = SERVO_SETUP_MODE;
//              setStatusofConfiguration(taskStatus);
//              FSMstate =  Active;
//          }
//
//           break;
//        }
//
//        case Active:
//        {
//           //All motors will be actively moving now so we should start the software timer so they move smoothly if needed.
//           if(firstRun == 1)
//           {
//               startSoftwareTimer();
//               firstRun = 0;
//           }
//
//           if (taskStatus == SERVO_SETUP_MODE)
//           {
//               //setStatusofConfiguration(SERVO_SETUP_MODE);
//               FSMstate = Active;
//           }
//
//           else if (taskStatus == PICK_UP_MODE)
//           {
//               //setStatusofConfiguration(PICK_UP_MODE);
//               FSMstate = Active;
//           }
//
//           else if(taskStatus == DROP_OFF_MODE)
//           {
//               //setStatusofConfiguration(DROP_OFF_MODE);
//               FSMstate = Active;
//           }
//
//
//           //BaseType_t check = checkIfTimerActive();
//           //
//           //readTimerStatus() == TIMER_EXPIRED
//           if(checkIfTimerActive() == pdFALSE)
//           {
//               /*tempholder++;
//
//               if(tempholder == 5)
//               {
//                   setStatusofConfiguration(IDLE_MODE);
//                   taskStatus = IDLE_MODE;
//                   FSMstate = Idle;
//                   firstRun = 1;
//                   firstRead = 1;
//               }
//
//               else
//               {
//                   firstRun = 1;
//                   firstRead = 0;
//               }*/
//
//               if(firstRead == 1)
//               {
//                   readAngleReadQueueBlocking( &angle );
//
//                   baseAngle = angle.angle_Base;
//                   extendAngle = angle.angle_Extend;
//                   liftAngle = angle.angle_Lift;
//                   clawAngle = angle.angle_Claw;
//
//                   firstRead = 0;
//               }
//
//               //increment/ decrement the duty cycle/ degree of the pwm by 1
//               //PWM formulas for each motor will return a completion status on it's progress to configuration.
//               status0 = setPWM_Base(baseAngle);
//               status1 = CONFIG_COMPLETE;
//               status2 = CONFIG_COMPLETE;
//               status3 = CONFIG_COMPLETE;
//
//               //status1 = setPWM_Extend(extendAngle);
//               //status2 = setPWM_Lift(liftAngle);
//               //status3 = setPWM_Claw(clawAngle);
//
//               if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
//               {
//                   //setStatusofConfiguration(IDLE_MODE);
//                   taskStatus = IDLE_MODE;
//                   FSMstate = Idle;
//                   firstRun = 1;
//                   firstRead = 1;
//               }
//
//               else
//               {
//                   FSMstate = Active;
//                   firstRun = 1;
//                   firstRead = 0;
//               }
//
//           }
//
//           break;
//        }
//
//        default:
//        {
//            // Code.
//            break;
//        }
//   }
//}
