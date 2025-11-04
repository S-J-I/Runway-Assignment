//Seth Ingersoll 1002207234
/* Copyright (c) 2025 Trevor Bakker
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILTY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/license/>.
*/
 
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <time.h>

/*** Constants that define parameters of the simulation ***/

#define MAX_RUNWAY_CAPACITY 2    /* Number of aircraft that can use runway simultaneously */
#define CONTROLLER_LIMIT 8       /* Number of aircraft the controller can manage before break */
#define MAX_AIRCRAFT 1000        /* Maximum number of aircraft in the simulation */
#define FUEL_MIN 20              /* Minimum fuel reserve in seconds */
#define FUEL_MAX 60              /* Maximum fuel reserve in seconds */
#define EMERGENCY_TIMEOUT 30     /* Max wait time for emergency aircraft in seconds */
#define DIRECTION_SWITCH_TIME 5  /* Time required to switch runway direction */
#define DIRECTION_LIMIT 3        /* Max consecutive aircraft in same direction */

#define COMMERCIAL 0
#define CARGO 1
#define EMERGENCY 2

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  4

/* TODO */
/* Add your synchronization variables here */
sem_t RunwayCapacity; // Runway capacity only lets 2 on at a time
sem_t Mutex; // Only letting Aircraft_on_runway, etc change 1 at a time
/* basic information about simulation.  they are printed/checked at the end 
 * and in assert statements during execution.
 *
 * you are responsible for maintaining the integrity of these variables in the 
 * code that you develop. 
 */

static int aircraft_on_runway = 0;       /* Total number of aircraft currently on runway */
static int commercial_on_runway = 0;     /* Total number of commercial aircraft on runway */
static int cargo_on_runway = 0;          /* Total number of cargo aircraft on runway */
static int emergency_on_runway = 0;      /* Total number of emergency aircraft on runway */
static int aircraft_since_break = 0;     /* Aircraft processed since last controller break */
static int current_direction = NORTH;    /* Current runway direction (NORTH or SOUTH) */
static int consecutive_direction = 0;    /* Consecutive aircraft in current direction */
int CommercialWaiting = 0;  // Counter for Waiting
int CargoWaiting = 0;  // Coutner for Cargo
int SwitchDirection = 0; // If 1 then pause everything while switching
int Break = 0; //If 1 then pause everything while on break
int EmergencyWaiting = 0; //Counter for Emergency
time_t EmergencyTime = 0; //Emergency timestamp for skipping line
int CommercialFuelWaiting = 0; //Counter for Commercial fuel emergences for controller
int CargoFuelWaiting = 0; ////Counter for Cargo fuel emergences for controller

typedef struct 
{
  int arrival_time;         // time between the arrival of this aircraft and the previous aircraft
  int runway_time;          // time the aircraft needs to spend on the runway
  int aircraft_id;
  int aircraft_type;        // COMMERCIAL, CARGO, or EMERGENCY
  int fuel_reserve;         // Randomly assigned fuel reserve (FUEL_MIN to FUEL_MAX seconds)
  time_t arrival_timestamp; // timestamp when aircraft thread was created
} aircraft_info;

/* Called at beginning of simulation.  
 * TODO: Create/initialize all synchronization
 * variables and other global variables that you add.
 */
static int initialize(aircraft_info *ai, char *filename) 
{
  aircraft_on_runway    = 0;
  commercial_on_runway  = 0;
  cargo_on_runway       = 0;
  emergency_on_runway   = 0;
  aircraft_since_break  = 0;
  current_direction     = NORTH;
  consecutive_direction = 0;
  CommercialWaiting = 0;
  CargoWaiting = 0;
  SwitchDirection = 0;
  Break = 0;
  EmergencyWaiting = 0;
  EmergencyTime = 0;
  CommercialFuelWaiting = 0;
  CargoFuelWaiting = 0;

  /* Initialize your synchronization variables (and 
   * other variables you might use) here
   */
  sem_init(&RunwayCapacity, 0, MAX_RUNWAY_CAPACITY); //Only allowing 2 planes on runway. Hits 0 when busy & others wait in holding pattern
  sem_init(&Mutex, 0, 1); // Only letting 1 on_runway change at a time

  /* seed random number generator for fuel reserves */
  srand(time(NULL));

  /* Read in the data file and initialize the aircraft array */
  FILE *fp;

  if((fp=fopen(filename, "r")) == NULL) 
  {
    printf("Cannot open input file %s for reading.\n", filename);
    exit(1);
  }

  int i = 0;
  char line[256];
  while (fgets(line, sizeof(line), fp) && i < MAX_AIRCRAFT) 
  {
    /* Skip comment lines and empty lines */
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
      continue;
    }
    
    /* Parse the line */
    if (sscanf(line, "%d%d%d", &(ai[i].aircraft_type), &(ai[i].arrival_time), 
               &(ai[i].runway_time)) == 3) {
      /* Assign random fuel reserve between FUEL_MIN and FUEL_MAX */
      ai[i].fuel_reserve = FUEL_MIN + (rand() % (FUEL_MAX - FUEL_MIN + 1));
      i = i + 1;
    }
  }

  fclose(fp);
  return i;
}

/* Code executed by controller to simulate taking a break 
 * You do not need to add anything here.  
 */
__attribute__((unused)) static void take_break() 
{
  printf("The air traffic controller is taking a break now.\n");
  sleep(5);
  assert( aircraft_on_runway == 0 );
  aircraft_since_break = 0;
}

/* Code executed to switch runway direction
 * You do not need to add anything here.
 */
__attribute__((unused)) static void switch_direction()
{
  printf("Switching runway direction from %s to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH",
         current_direction == NORTH ? "SOUTH" : "NORTH");
  
  assert( aircraft_on_runway == 0 );  // Runway must be empty to switch
  
  sleep(DIRECTION_SWITCH_TIME);
  
  current_direction = (current_direction == NORTH) ? SOUTH : NORTH;
  consecutive_direction = 0;
  
  printf("Runway direction switched to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH");
}

/* Code for the air traffic controller thread. This is fully implemented except for 
 * synchronization with the aircraft. See the comments within the function for details.
 */
void *controller_thread(void *arg) 
{
  // Suppress the warning for now
 (void)arg;

  printf("The air traffic controller arrived and is beginning operations\n");

  /* Loop while waiting for aircraft to arrive. */
  while (1) 
  {
    /* TODO */
    /* Add code here to handle aircraft requests, controller breaks,      */
    /* and runway direction switches.                                     */
    /* Currently the body of the loop is empty.  There's no communication */
    /* between controller and aircraft, i.e. all aircraft are admitted    */
    /* without regard for runway capacity, aircraft type, direction,      */
    /* priorities, and whether the controller needs a break.              */
    /* You need to add all of this.                                       */
    int NorthSwitchLimit = 0;
    int SouthSwitchLimit = 0;
    int NorthSwitchEarly = 0;
    int SouthSwitchEarly = 0;
    int NorthSwitchFuel = 0;
    int SouthSwitchFuel = 0;

    sem_wait(&Mutex);
    if(aircraft_since_break >= CONTROLLER_LIMIT) //Once controller at limit
    {
      Break = 1; //Stop planes from entering

      while(aircraft_on_runway > 0) // Controller at limit so let planes on runway take off
      {
        sem_post(&Mutex);
        sleep(1);
        sem_wait(&Mutex); //Wait mutex technically starting loop
      }

      sem_post(&Mutex); //Close that mutex before break
      take_break();

      sem_wait(&Mutex);
      Break = 0; //Allow planes to start entering
      sem_post(&Mutex);
    }

    else
    {
      sem_post(&Mutex); 
      sem_wait(&Mutex);
      int Limit = (consecutive_direction >= DIRECTION_LIMIT); //Short lines 
      int NorthWaiting = (CargoWaiting == 0) && (CommercialWaiting > 0) && (CargoFuelWaiting == 0);
      int SouthWaiting = (CommercialWaiting == 0) && (CargoWaiting > 0) && (CommercialFuelWaiting == 0);
      int North = (current_direction == NORTH);
      int South = (current_direction == SOUTH);

      NorthSwitchLimit = ((Limit) && (South == 1) && (CommercialWaiting > 0)); //If limit is hit with Commercial waiting & no emergencies then switch
      SouthSwitchLimit = ((Limit) && (North == 1) && (CargoWaiting > 0)); //If limit is hit with Cargo waiting & no emergencies then switch

      NorthSwitchEarly = ((South == 1) && (NorthWaiting) && (aircraft_on_runway == 0)); //If Commercial waiting & no cargo/fuel emergency then switch
      SouthSwitchEarly = ((North == 1) && (SouthWaiting) && (aircraft_on_runway == 0)); //If Cargo waiting & no commercial/fuel emergency then switch

      NorthSwitchFuel = ((South == 1) && (CommercialFuelWaiting > 0)); //If cargo fuel emergency & no emergency then switch
      SouthSwitchFuel = ((North == 1) && (CargoFuelWaiting > 0)); //if commercial fuel emergency & no emergency then switch

      if((NorthSwitchFuel == 1) || (SouthSwitchFuel == 1))
      {
        SwitchDirection = 1; //Stop planes

        while(aircraft_on_runway > 0) // Let planes on runway take off
        {
          sem_post(&Mutex);
          sleep(1);
          sem_wait(&Mutex); //Wait mutex technically starting loop
        }
        
        sem_post(&Mutex); //Close that mutex before switch
        switch_direction();

        sem_wait(&Mutex);
        SwitchDirection = 0; //Allow planes to enter again
        sem_post(&Mutex);
      }

      else
      {
        if((NorthSwitchLimit == 1) || (SouthSwitchLimit == 1)) //If limit is met then stop planes from entering
        {
          SwitchDirection = 1; //Stop planes

          while(aircraft_on_runway > 0) // Let planes on runway take off
          {
            sem_post(&Mutex);
            sleep(1);
            sem_wait(&Mutex); //Wait mutex technically starting loop
          }
          
          sem_post(&Mutex); //Close that mutex before switch
          switch_direction();

          sem_wait(&Mutex);
          SwitchDirection = 0; //Allow planes to enter again
          sem_post(&Mutex);
        }

        else if((NorthSwitchEarly == 1) || (SouthSwitchEarly == 1)) //If switching early stop planes from going on runway
        {
          SwitchDirection = 1; //Stop planes

          while(aircraft_on_runway > 0) // Let planes on runway take off
          {
            sem_post(&Mutex);
            sleep(1);
            sem_wait(&Mutex); //Wait mutex technically starting loop
          }

          sem_post(&Mutex); //Close that mutex before switch
          switch_direction();

          sem_wait(&Mutex);
          SwitchDirection = 0; //Allow planes to enter again
          sem_post(&Mutex);
        }

        else //If nothing then continue
        {
          sem_post(&Mutex);
          /* Allow thread to be cancelled */
          pthread_testcancel();
          sleep(1); // 100ms sleep to prevent busy waiting
        }
      } 
    }
  }

  pthread_exit(NULL);
}

/* Code executed by a commercial aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void commercial_enter(aircraft_info *arg) 
{
  // Suppress the compiler warning

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (commercial prefer NORTH),       */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */
  /*  YOUR CODE HERE.
                                                        */
  int AddCommercial = 0;
  int FuelEmergency = 0;

  sem_wait(&Mutex);
  CommercialWaiting++; //Announce commercial plane waiting
  sem_post(&Mutex);

  while (AddCommercial == 0)
	{
    sem_wait(&Mutex);
    int CommercialTime = arg->arrival_timestamp; //Timestamp when commercial was made
    int TimeNow = time(NULL); //Current time
    int Fuel = arg->fuel_reserve; //Get the fuel from when commerical was made
    int TimeWaited = (TimeNow - CommercialTime); //Get time now vs when commercial was made
    int FuelLeft = (Fuel - TimeWaited); //Take out fuel for each second waited
    int EmergencyRunway = ((Break == 0) && (aircraft_on_runway < 2)); //Check if there's a runway to use 

    int NorthClear = ((current_direction == NORTH) && (cargo_on_runway == 0)); //Short lines
    int FuelLandingChecks = ((FuelLeft <= 0) && (EmergencyRunway == 1));
    int Limit = ((consecutive_direction < DIRECTION_LIMIT));
    sem_post(&Mutex);

    if((FuelLeft <= 0) && (FuelEmergency == 0)) //Announce FuelEmergency once
    {
      FuelEmergency = 1;
      CommercialFuelWaiting++; //Let controller know
      printf("Plane %d Fuel Emergency\n", arg->aircraft_id); //Terminal output
    }

    if((FuelLandingChecks == 1) && (NorthClear == 1) && (SwitchDirection == 0)) //If no fuel & can land
    {
      sem_post(&Mutex); //Close mutex before runway
      sem_wait(&RunwayCapacity); //Ask for a runway

      sem_wait(&Mutex);
      if((EmergencyRunway == 1) && (NorthClear == 1) && (SwitchDirection == 0)) //Recheck after asking for runway
      {
        aircraft_on_runway    = aircraft_on_runway + 1;
        aircraft_since_break  = aircraft_since_break + 1;
        commercial_on_runway  = commercial_on_runway + 1;
        consecutive_direction = consecutive_direction + 1;
        CommercialWaiting--; //On runway so not waiting
        CommercialFuelWaiting--; //Get rid of fuelwaiting tag
        sem_post(&Mutex);
        FuelEmergency = 0; //Reset FuelEmergency so controller can go back to normal
        AddCommercial = 1; //Commercial added to runway
      }

      else
      {
        sem_post(&Mutex); //Close open mutex
        sem_post(&RunwayCapacity); //Give back runway
      }
    }
    

    sem_wait(&Mutex);
    if ((Limit == 1) && (NorthClear == 1) && (SwitchDirection == 0) && (Break == 0)) //If no fuel/emergency problems then try to land
    {
      sem_post(&Mutex); //Close mutex before runway
      sem_wait(&RunwayCapacity); //Ask for a runway

      sem_wait(&Mutex);
      if ((Limit == 1) && (NorthClear == 1) && (SwitchDirection == 0) && (Break == 0)) //Double check after runway before letting through
      {
        aircraft_on_runway    = aircraft_on_runway + 1;
        aircraft_since_break  = aircraft_since_break + 1;
        commercial_on_runway  = commercial_on_runway + 1;
        consecutive_direction = consecutive_direction + 1;
        CommercialWaiting--; //On runway so not waiting
        sem_post(&Mutex);
        AddCommercial = 1; //Commercial added to runway
      }

      else
      {
        sem_post(&Mutex); //Close open mutex
        sem_post(&RunwayCapacity); //Give back runway
      }
    }

    else
    {
      sem_post(&Mutex); //Close mutex before sleep
      sleep(1);
    }
  }
}
/* Code executed by a cargo aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void cargo_enter(aircraft_info *ai) 
{

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (cargo prefer SOUTH),            */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */
  /*  YOUR CODE HERE.                                                      */
  int AddCargo = 0;
  int FuelEmergency = 0;

  sem_wait(&Mutex);
  CargoWaiting++; //Announce cargo plane waiting
  sem_post(&Mutex);

  while (AddCargo == 0)
	{
    sem_wait(&Mutex);
    int CargoTime = ai->arrival_timestamp; //Timestamp when cargo was made
    int TimeNow = time(NULL); //Current time
    int Fuel = ai->fuel_reserve; //Get the fuel from when cargo was made
    int TimeWaited = (TimeNow - CargoTime); //Get time now vs when cargo was made
    int FuelLeft = (Fuel - TimeWaited); //Take out fuel for each second waited
    int EmergencyRunway = ((Break == 0) && (aircraft_on_runway < 2)); //Check if there's a runway to use

    int SouthClear = ((current_direction == SOUTH) && (commercial_on_runway == 0)); //Short lines
    int FuelLandingChecks = ((FuelLeft <= 0) && (EmergencyRunway == 1));
    int Limit = ((consecutive_direction < DIRECTION_LIMIT));
    sem_post(&Mutex);

    if((FuelLeft <= 0) && (FuelEmergency == 0)) //Announce FuelEmergency once
    {
      FuelEmergency = 1; //Let controller know
      CargoFuelWaiting++; //Increase emergency waiting
      printf("Plane %d Fuel Emergency\n", ai->aircraft_id); //Terminal output
    }

    if((FuelLandingChecks == 1) && (SouthClear == 1) && (SwitchDirection == 0)) //If no fuel & can land
    {
      sem_post(&Mutex); //Close mutex before runway
      sem_wait(&RunwayCapacity); //Ask for a runway

      sem_wait(&Mutex);
      if((EmergencyRunway == 1) && (SouthClear == 1) && (SwitchDirection == 0)) //Recheck after asking for runway
      {
        aircraft_on_runway    = aircraft_on_runway + 1;
        aircraft_since_break  = aircraft_since_break + 1;
        cargo_on_runway  = cargo_on_runway + 1;
        consecutive_direction = consecutive_direction + 1;
        CargoWaiting--; //On runway so not waiting
        CargoFuelWaiting--; //Get rid of fuelwaiting tag
        sem_post(&Mutex); //Close mutex before runway
        FuelEmergency = 0; //Reset FuelEmergency so controller can go back to normal
        AddCargo = 1; //Commercial added to runway
      }

      else
      {
        sem_post(&Mutex); //Close open mutex
        sem_post(&RunwayCapacity); //Give back runway
      }
    }

    sem_wait(&Mutex);
    if ((Limit == 1) && (SouthClear == 1) && (SwitchDirection == 0) && (Break == 0)) //If no fuel/emergency problems then try to land
    {
      sem_post(&Mutex); //Close mutex before runway
      sem_wait(&RunwayCapacity); //Ask for a runway

      sem_wait(&Mutex);
      if ((Limit == 1) && (SouthClear == 1) && (SwitchDirection == 0) && (Break == 0)) //Double check after runway before letting through
      {
        aircraft_on_runway    = aircraft_on_runway + 1;
        aircraft_since_break  = aircraft_since_break + 1;
        cargo_on_runway  = cargo_on_runway + 1;
        consecutive_direction = consecutive_direction + 1;
        CargoWaiting--; //On runway so not waiting
        sem_post(&Mutex);
        AddCargo = 1; //Commercial added to runway
      }

      else
      {
        sem_post(&Mutex); //Close open mutex
        sem_post(&RunwayCapacity); //Give back runway
      }
    }

    else
    {
      sem_post(&Mutex); //Close mutex before sleep
      sleep(1);
    }
  }
}

/* Code executed by an emergency aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void emergency_enter(aircraft_info *ai) 
{

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Emergency aircraft have priority and must be admitted within 30s,     */
  /* but still respect runway capacity and controller breaks.              */
  /* Emergency aircraft can use either direction.                          */
  /*  YOUR CODE HERE.                                                      */
  int AddEmergency = 0;

  sem_wait(&Mutex);
  EmergencyWaiting++; //Announce Emergency plane waiting
  int ThisEmergency = ai->arrival_timestamp; //Current emergency timestamp

  if(EmergencyTime == 0 || ThisEmergency < EmergencyTime) //If first emergency or older than previous
  {
    EmergencyTime = ThisEmergency; //Replace the last emergency with current
  }
  sem_post(&Mutex);

  while (AddEmergency == 0)
	{
    sem_wait(&Mutex);
    int Wait = 0; // Holds back emergency
    int TimeNow = time(NULL); //Current time
    int Age = (TimeNow - ThisEmergency); //Get time now vs when emergency made

    int AnyCommercial = (CommercialWaiting + CommercialFuelWaiting); //Short lines
    int AnyCargo = (CargoWaiting + CargoFuelWaiting);
    int RunwayGood = ((SwitchDirection == 0) && (Break == 0));

    if(Age < EMERGENCY_TIMEOUT) //If time < timeout
    {
      if(((AnyCommercial) > 0) || ((AnyCargo) > 0)) //Any planes ahead of emergency
      {
        Wait = 1; //Wait and don't skip line
      }
    }

    if(Wait == 1) //If planes ahead sleep and try again
    {
      sem_post(&Mutex);
      sleep(1);
    }

    else if ((RunwayGood) && (aircraft_on_runway < 2) && (Wait == 0)) //Only cares if SwitchLimit or Break are active & open runway slot
    {
      sem_post(&Mutex); //Close mutex before runway
      sem_wait(&RunwayCapacity); //Ask for a runway

      sem_wait(&Mutex);
      if ((RunwayGood) && (aircraft_on_runway < 2)) //Double check after runway before letting through
      {
        int remaining = EMERGENCY_TIMEOUT - (int)(time(NULL) - ai->arrival_timestamp); //Debug tiemr for emergencies
        printf("Time remaining: %ds\n", remaining); //Print debug

        aircraft_on_runway = aircraft_on_runway + 1;
        aircraft_since_break = aircraft_since_break + 1;
        emergency_on_runway = emergency_on_runway + 1;
        consecutive_direction = consecutive_direction + 1;
        EmergencyWaiting--; //On runway so not waiting
        sem_post(&Mutex);
        AddEmergency = 1; //Emergency added to runway
      }

      else
      {
        sem_post(&Mutex); //Close open mutex
        sem_post(&RunwayCapacity); //Give back runway
      }
    }

    else
    {
      sem_post(&Mutex); //Close mutex before sleep
      sleep(1);
    }
  }
}

/* Code executed by an aircraft to simulate the time spent on the runway
 * You do not need to add anything here.  
 */
static void use_runway(int t) 
{
  sleep(t);
}


/* Code executed by a commercial aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void commercial_leave() 
{
  /* 
   *  TODO
   *  YOUR CODE HERE. 
   */
  sem_wait(&Mutex); //Change on_runway

  aircraft_on_runway = aircraft_on_runway - 1;
  commercial_on_runway = commercial_on_runway - 1;

  sem_post(&RunwayCapacity); //Open runway spot again
  sem_post(&Mutex); //Leave on_runway alone
}

/* Code executed by a cargo aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void cargo_leave() 
{
  /* 
   * TODO
   * YOUR CODE HERE. 
   */
  sem_wait(&Mutex); //Change on_runway

  aircraft_on_runway = aircraft_on_runway - 1;
  cargo_on_runway = cargo_on_runway - 1;

  sem_post(&RunwayCapacity); //Open runway spot again
  sem_post(&Mutex); //Leave on_runway alone
}

/* Code executed by an emergency aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void emergency_leave() 
{
  /* 
   * TODO
   * YOUR CODE HERE. 
   */
  sem_wait(&Mutex); //Change on_runway

  aircraft_on_runway = aircraft_on_runway - 1;
  emergency_on_runway = emergency_on_runway - 1;

  sem_post(&RunwayCapacity); //Open runway spot again
  sem_post(&Mutex); //Leave on_runway alone
}

/* Main code for commercial aircraft threads.  
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* commercial_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  commercial_enter(ai);

  printf("Commercial aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway == 0 ); // Commercial and cargo cannot mix
  
  /* Use runway  --- do not make changes to the 3 lines below*/
  printf("Commercial aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Commercial aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  commercial_leave();  

  printf("Commercial aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for cargo aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* cargo_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  cargo_enter(ai);

  printf("Cargo aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, 
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(commercial_on_runway == 0 ); 

  printf("Cargo aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Cargo aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  cargo_leave();        

  printf("Cargo aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", 
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for emergency aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* emergency_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel and emergency timeout tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  emergency_enter(ai);

  printf("EMERGENCY aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, 
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  printf("EMERGENCY aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("EMERGENCY aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  emergency_leave();        

  printf("EMERGENCY aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", 
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main function sets up simulation and prints report
 * at the end.
 * GUID: 355F4066-DA3E-4F74-9656-EF8097FBC985
 */
int main(int nargs, char **args) 
{
  int i;
  int result;
  int num_aircraft;
  void *status;
  pthread_t controller_tid;
  pthread_t aircraft_tid[MAX_AIRCRAFT];
  aircraft_info ai[MAX_AIRCRAFT];

  if (nargs != 2) 
  {
    printf("Usage: runway <name of inputfile>\n");
    return EINVAL;
  }

  num_aircraft = initialize(ai, args[1]);
  if (num_aircraft > MAX_AIRCRAFT || num_aircraft <= 0) 
  {
    printf("Error:  Bad number of aircraft threads. "
           "Maybe there was a problem with your input file?\n");
    return 1;
  }

  printf("Starting runway simulation with %d aircraft ...\n", num_aircraft);

  result = pthread_create(&controller_tid, NULL, controller_thread, NULL);

  if (result) 
  {
    printf("runway:  pthread_create failed for controller: %s\n", strerror(result));
    exit(1);
  }

  for (i=0; i < num_aircraft; i++) 
  {
    ai[i].aircraft_id = i;
    sleep(ai[i].arrival_time);
                
    if (ai[i].aircraft_type == COMMERCIAL)
    {
      result = pthread_create(&aircraft_tid[i], NULL, commercial_aircraft, 
                             (void *)&ai[i]);
    }
    else if (ai[i].aircraft_type == CARGO)
    {
      result = pthread_create(&aircraft_tid[i], NULL, cargo_aircraft, 
                             (void *)&ai[i]);
    }
    else 
    {
      result = pthread_create(&aircraft_tid[i], NULL, emergency_aircraft, 
                             (void *)&ai[i]);
    }

    if (result) 
    {
      printf("runway: pthread_create failed for aircraft %d: %s\n", 
            i, strerror(result));
      exit(1);
    }
  }

  /* wait for all aircraft threads to finish */
  for (i = 0; i < num_aircraft; i++) 
  {
    pthread_join(aircraft_tid[i], &status);
  }

  /* tell the controller to finish. */
  pthread_cancel(controller_tid);
  pthread_join(controller_tid, &status);

  printf("Runway simulation done.\n");

  return 0;
}
