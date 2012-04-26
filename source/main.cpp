// Panda main
#include "pandaFramework.h"
#include "pandaSystem.h"
#include "load_prc_file.h" // for changing framework configs

// Panda tasks
#include "genericAsyncTask.h"
#include "asyncTaskManager.h"

// Panda Intervals
#include "cIntervalManager.h"
#include "cLerpNodePathInterval.h"
#include "cMetaInterval.h"
 
PandaFramework framework;
// The global task manager
PT(AsyncTaskManager) taskMgr = AsyncTaskManager::get_global_ptr(); 
// The global clock
PT(ClockObject) globalClock = ClockObject::get_global_clock();
// Here's what we'll store the camera in.
NodePath camera;
double cameraHDist = 2.0;
double cameraVDist = 10.0;

// This is our task - a global or static function that has to return DoneStatus.
// The task object is passed as argument, plus a void* pointer, containing custom data.
// For more advanced usage, we can subclass AsyncTask and override the do_task method.
AsyncTask::DoneStatus spinCameraTask(GenericAsyncTask* task, void* data) 
{
   // Calculate the new position and orientation (inefficient - change me!)
   double time = globalClock->get_real_time();
   double angledegrees = time * 6.0;
   double angleradians = angledegrees * (3.14 / 180.0);
   camera.set_pos(20*sin(angleradians),-20.0*cos(angleradians),3);
   camera.set_hpr(angledegrees, 0, 0);
 
   // Tell the task manager to continue this task the next frame.
   return AsyncTask::DS_cont;
}

/* returns 1, 2, 3, or 4 for quadrants;
 * returns 5, 6, 7, 8 for xpos, ypos, xneg, yneg axes (respectively)
 * or zero if the calculation fails
 */
int getAngleQuad( double radAngle )
{
   double sinResult = sin( radAngle );
   double cosResult = cos( radAngle );
   double tanResult = tan( radAngle );

   bool sinPositive = sinResult > 0;
   bool cosPositive = cosResult > 0;
   bool tanPositive = tanResult > 0;

   if( sinPositive && cosPositive && tanPositive ) return 1;
   if( sinPositive ) return 2;
   if( tanPositive ) return 3;
   if( cosPositive ) return 4;

   /* if we get this far, it's on an axis:
    * deg   sin   cos
    * 0	   0	   1
    * 90	   1	   0
    * 180 	0	   -1
    * 270   -1	   0
    */
   if( sinResult == 0 && cosResult == 1 ) return 5;
   if( sinResult == 1 && cosResult == 0 ) return 6;
   if( sinResult == 0 && cosResult == -1 ) return 7;
   if( sinResult == -1 && cosResult == 0 ) return 8;

   return 0;
}

// Update camera pos to follow the actor (chase cam)
AsyncTask::DoneStatus updateCameraTask( GenericAsyncTask* task, void* actor ) 
{
   NodePath* actorPtr = (NodePath*)actor;
   LVecBase3f actorPos = actorPtr->get_pos();
   LVecBase3f actorHpr = actorPtr->get_hpr();

   double actorXAngleDeg = actorHpr.get_x();
   double actorXAngleRad = actorXAngleDeg * (3.14 / 180.0);
   double cameraDeltaX = cameraHDist * cos( actorXAngleRad );
   double cameraDeltaY = cameraHDist * sin( actorXAngleRad );
   double cameraAbsX, cameraAbsY, cameraTiltAngleRad, cameraTiltAngleDeg;
   
   // fix camera deltas based on actor's view angle quadrant
   int actorAngleQuad = getAngleQuad( actorXAngleRad );
   switch( actorAngleQuad )
   {
      case 1:
      case 5:
         cameraDeltaX = -1 * fabs( cameraDeltaX );
         cameraDeltaY = -1 * fabs( cameraDeltaY );
         break;
      case 2:
      case 6:
         cameraDeltaX = fabs( cameraDeltaX );
         cameraDeltaY = -1 * fabs( cameraDeltaY );
         break;
      case 3:
      case 7:
         cameraDeltaX = fabs( cameraDeltaX );
         cameraDeltaY = fabs( cameraDeltaY );
         break;
      case 4:
      case 8:
         cameraDeltaX = -1 * fabs( cameraDeltaX );
         cameraDeltaY = fabs( cameraDeltaY );
         break;
      default:
         break;
   }

   cameraAbsX = actorPos.get_x() + cameraDeltaX;
   cameraAbsY = actorPos.get_y() + cameraDeltaY;
   cameraTiltAngleRad = -1.0 * atan( cameraVDist / cameraHDist );
   cameraTiltAngleDeg = cameraTiltAngleRad / (3.14 / 180.0);

   cout << "Camera: " << cameraAbsX << " " << cameraAbsY << " " 
      << camera.get_hpr().get_x() << '\n';
   cout << "Actor: " << actorPos.get_x() << " " << actorPos.get_y() << " " 
      << actorHpr.get_x() << '\n';

   camera.set_pos( cameraAbsX, cameraAbsY, cameraVDist );
   camera.set_hpr( actorHpr.get_x(), cameraTiltAngleDeg, 0 );

   // Tell the task manager to continue this task the next frame.
   return AsyncTask::DS_cont;
}

// Task to step the interval manager
AsyncTask::DoneStatus stepIntervalManTask( GenericAsyncTask* task, void* data )
{
   CIntervalManager::get_global_ptr()->step();
   return AsyncTask::DS_cont;
}

// Setup panda model walk stuff
void initPandaPace( NodePath &pandaActor )
{
   // Create the lerp intervals needed to walk back and forth
   PT(CLerpNodePathInterval) pandaPosInterval1, pandaPosInterval2,
      pandaHprInterval1, pandaHprInterval2;
   pandaPosInterval1 = new CLerpNodePathInterval("pandaPosInterval1",
      7.75, CLerpInterval::BT_no_blend,
      true, false, pandaActor, NodePath());
   pandaPosInterval1->set_start_pos(LPoint3f(0, 10, 0));
   pandaPosInterval1->set_end_pos(LPoint3f(0, -10, 0));
 
   pandaPosInterval2 = new CLerpNodePathInterval("pandaPosInterval2",
      7.75, CLerpInterval::BT_no_blend,
      true, false, pandaActor, NodePath());
   pandaPosInterval2->set_start_pos(LPoint3f(0, -10, 0));
   pandaPosInterval2->set_end_pos(LPoint3f(0, 10, 0));
 
   pandaHprInterval1 = new CLerpNodePathInterval("pandaHprInterval1", 1.0,
       CLerpInterval::BT_no_blend,
       true, false, pandaActor, NodePath());
   pandaHprInterval1->set_start_hpr(LPoint3f(0, 0, 0));
   pandaHprInterval1->set_end_hpr(LPoint3f(180, 0, 0));
 
   pandaHprInterval2 = new CLerpNodePathInterval("pandaHprInterval2", 1.0,
       CLerpInterval::BT_no_blend,
       true, false, pandaActor, NodePath());
   pandaHprInterval2->set_start_hpr(LPoint3f(180, 0, 0));
   pandaHprInterval2->set_end_hpr(LPoint3f(0, 0, 0));
 
   // Create and play the sequence that coordinates the intervals
   PT(CMetaInterval) pandaPace;
   pandaPace = new CMetaInterval("pandaPace");
   pandaPace->add_c_interval(pandaPosInterval1, 0,
      CMetaInterval::RS_previous_end);
   pandaPace->add_c_interval(pandaHprInterval1, 0,
      CMetaInterval::RS_previous_end);
   pandaPace->add_c_interval(pandaPosInterval2, 0,
      CMetaInterval::RS_previous_end);
   pandaPace->add_c_interval(pandaHprInterval2, 0,
      CMetaInterval::RS_previous_end);
   pandaPace->loop();
}

// Add the task to move the panda
void startPandaWalk( const Event* theEvent, void* intervalStepTask )
{
   taskMgr->add( (GenericAsyncTask*)intervalStepTask );
}

// Remove the task to move the panda
void stopPandaWalk( const Event* theEvent, void* intervalStepTask )
{
   taskMgr->remove( (GenericAsyncTask*)intervalStepTask );
}

int main(int argc, char *argv[]) 
{
   // Load the window and set its title.
   framework.open_framework(argc, argv);
   framework.set_window_title("Panda3Derp Window");
   load_prc_file_data( "", "win-size 1280 720" );
   WindowFramework *window = framework.open_window();

   // Get the camera and store it in a variable.
   camera = window->get_camera_group();

   // Enable keyboard detection 
   window->enable_keyboard();

   // Load the environment model.
   NodePath environ = window->load_model(framework.get_models(), "models/environment");
   // Reparent the model to render.
   environ.reparent_to(window->get_render());
   // Apply scale and position transforms to the model.
   environ.set_scale(0.25, 0.25, 0.25);
   environ.set_pos(-8, 42, 0);

   // Load our panda
   NodePath pandaActor = window->load_model(framework.get_models(), "ralph");
   //pandaActor.set_scale(0.005);
   pandaActor.reparent_to(window->get_render());

   // Load the walk animation
   window->load_model(pandaActor, "ralph-walk");
   window->loop_animations(0);
 
   // Sets up notepaths for the panda motion
   initPandaPace( pandaActor );

   // Add our task.
   // If we specify custom data instead of NULL, it will be passed as the second argument
   // to the task function.
   //taskMgr->add(new GenericAsyncTask("SpinCameraPos", &spinCameraTask, (void*) NULL));
   //taskMgr->add( new GenericAsyncTask("UpdateCameraPos", &updateCameraTask, 
   //   (void*) &pandaActor ) );
   //taskMgr->add(new GenericAsyncTask("Step interval manager", &stepIntervalManTask, 
   //   (void*) NULL));

   PT(GenericAsyncTask) intervalStepTask = new GenericAsyncTask( "Step interval manager", 
      &stepIntervalManTask, (void*) NULL );

   LPoint3f pandaPos = pandaActor.get_pos();
   LPoint3f pandaHpr = pandaActor.get_hpr();
   camera.set_pos( pandaPos.get_x(), pandaPos.get_y() + 22.0, pandaPos.get_z() + 26.0 );
   camera.set_hpr( pandaHpr.get_x() + 180.0, -45.0, 0 );
   camera.reparent_to( pandaActor );

   // derpy input handling
   framework.define_key( "s", "start", startPandaWalk, intervalStepTask );
   framework.define_key( "d", "stop", stopPandaWalk, intervalStepTask );

   // Run the engine.
   framework.main_loop();
   // Shut down the engine when done.
   framework.close_framework();

   return (0);
}
