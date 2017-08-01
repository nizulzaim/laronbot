/*
 *   GOHIT.C -- A simple RoboTar program with uCOS-II
 *   Written by: Rosbi Mamat 6/5/2014
 */

#include "..\ucosobj\kernel.h"              /* Always include these to use uCOS-II      */
#include "..\hal\hal_robo.h"                /*   and RoboTar HAL                        */

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          1          /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO     3
#define TASK_CTRLMOTOR_PRIO      4
#define TASK_NAVIG_PRIO          5          /* Lowest priority                          */
#define TASK_LIGHT_PRIO			 2
OS_STK CtrlmotorStk[TASK_STK_SZ];           /* Task CtrlMotors stack                    */
OS_STK NavigBotStk[TASK_STK_SZ];            /* Task NavigRobot stack                    */
OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack*/
OS_STK LightTrackingStk[TASK_STK_SZ];       

/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate
{
    int rspeed;                             /* right motor speed  (-100 -- +100)        */
    int lspeed;                             /* leftt motor speed  (-100 -- +100)        */
    char collided;                          /* collision? 1 = yes, 0 = no               */
} myrobot;

/*------High pririority task----------*/
void CheckCollision (void *data)
{
    for(;;)
    {
        if ( (robo_bumpSensorR() == HIT) ||         /* collide ?                         */
             (robo_bumpSensorL() == HIT)    )
        {
            myrobot.collided = 1;                   /* signal that collisioin            */
         }
        else
        {
            myrobot.collided = 0;                   /* signal no collisioin              */
        }
        OSTimeDlyHMSM(0, 0, 0, 100);                /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors (void *data)
{
    int speed_r, speed_l;

    for(;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 200);                /* Task period ~ 200 ms              */
    }
}

/* -----------------------------------------------
 * Task for navigating robot
 * Write you own navigation task here
 */

void NavigRobot (void *data)
{
    int dist;

    for (;;)
    {
        dist = robo_distSensor();                   /* Read distance sensor                 */

        if (myrobot.collided == 1)                  /* If collided then stop                */
        {
            //myrobot.rspeed   = STOP_SPEED;          /* STOP */
            //myrobot.lspeed   = STOP_SPEED;
			//OSTimeDlyHMSM(0, 0, 0, 200);
			myrobot.rspeed   = -HIGH_SPEED;                             
            myrobot.lspeed   = -HIGH_SPEED;
			OSTimeDlyHMSM(0, 0, 2, 0);
			myrobot.rspeed   = -HIGH_SPEED;          /* turn right                            */
            myrobot.lspeed   = LOW_SPEED;
			OSTimeDlyHMSM(0, 0, 0, 700);
			myrobot.rspeed   =  100;          /* go straight                           */
            myrobot.lspeed   =  100;

        }
		else if (dist < 40 && dist >= 30) {
			myrobot.lspeed   = LOW_SPEED;
			myrobot.rspeed   = LOW_SPEED;
		}
        else if (dist < 30)                         /* if obstacle is too close              */
        {
            myrobot.rspeed   = -HIGH_SPEED;          /* turn right                            */
            myrobot.lspeed   = LOW_SPEED;
            OSTimeDlyHMSM(0, 0, 0, 700);              /* Wait 1 sec for right turn to complete */
            myrobot.rspeed   =  100;          /* go straight                           */
            myrobot.lspeed   =  100;
        }
        else                                        /* obstacle is far away & no collision   */
        {
            myrobot.rspeed   = 100;           /* move forward at straight              */
            myrobot.lspeed   = 100;
        }
        OSTimeDlyHMSM(0, 0, 0, 500);                /* Task period ~ 500 ms                  */
    }
}



/* Light Sensor TASK */
void LightTracking(void *data)
{
    //int speed_r , speed_l;
	int llight, rlight;

    for(;;)
    {
        //speed_r = myrobot.rspeed;
        //speed_l = myrobot.lspeed;
		llight = robo_lightSensorL();
        rlight = robo_lightSensorR();
		
		if (llight >= 400 && rlight >= 400) {
			robo_Honk();
			if (llight > rlight) {
				myrobot.lspeed = -70;
				myrobot.rspeed = 100;
				
				OSTimeDlyHMSM(0, 0, 0, 600);
			}

			else if (rlight > llight) {
				myrobot.lspeed = 100;
				myrobot.rspeed = -70;
				OSTimeDlyHMSM(0, 0, 0, 600);
			}  
		} else {
			OSTimeDlyHMSM(0, 0, 0, 100);   
		}
           
    }
}
/*------Highest pririority task----------*/
/* Create all other tasks here           */
void TaskStart( void *data )
{
    OS_ticks_init();                                        /*-enable RTOS timer tick        */

    OSTaskCreate(CheckCollision,                            /*-Task function                 */
                (void *)0,                                  /*-nothing passed to task        */
                (void *)&ChkCollideStk[TASK_STK_SZ - 1],    /*-stack allocated to task       */
                TASK_CHKCOLLIDE_PRIO);                      /*-priority of task              */

    OSTaskCreate(CntrlMotors,                               /*-Task function                 */
                (void *)0,                                  /*-nothing passed to task        */
                (void *)&CtrlmotorStk[TASK_STK_SZ - 1],     /*-stack allocated to task       */
                TASK_CTRLMOTOR_PRIO);                       /*-priority of task              */

    OSTaskCreate(NavigRobot,                                /*-Task function                 */
                (void *)0,                                  /*-nothing passed to task        */
                (void *)&NavigBotStk[TASK_STK_SZ - 1],      /*-stack allocated to task       */
                TASK_NAVIG_PRIO);                           /*-priority of task              */
	
	OSTaskCreate(LightTracking,                                /*-Task function                 */
                (void *)0,                                  /*-nothing passed to task        */
                (void *)&LightTrackingStk[TASK_STK_SZ - 1],      /*-stack allocated to task       */
                TASK_LIGHT_PRIO);                           /*-priority of task              */

    while(1)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);                          /* Task period ~ 5 secs          */
        robo_LED_toggle();                                  /* Show that we are alive        */
    }

}

int main( void )
{
    robo_Setup();                                          /* initialize HAL for robot       */
    OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.collided = 0;                                  /*  No collisioin                 */

    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}
