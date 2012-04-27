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

/*************************/
/*        GLOBALS        */
/*************************/
PandaFramework framework;
// The global task manager
PT(AsyncTaskManager) taskMgr = AsyncTaskManager::get_global_ptr(); 
// The global clock
PT(ClockObject) globalClock = ClockObject::get_global_clock();
// Here's what we'll store the camera in.
NodePath camera;

/* This is our task - a global or static function that has to return DoneStatus.
 * The task object is passed as argument, plus a void* pointer, containing custom data.
 * For more advanced usage, we can subclass AsyncTask and override the do_task method.
 */

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
   pandaActor.reparent_to(window->get_render());

   // Load the walk animation
   window->load_model(pandaActor, "ralph-walk");
   window->loop_animations(0);
 
   // Sets up notepaths for the panda motion
   initPandaPace( pandaActor );

   // Add a task to step the interval manager continuously 
   //taskMgr->add(new GenericAsyncTask("Step interval manager", &stepIntervalManTask, 
   //   (void*) NULL));

   PT(GenericAsyncTask) intervalStepTask = new GenericAsyncTask( "Step interval manager", 
      &stepIntervalManTask, (void*) NULL );

   // make camera follow the actor
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
