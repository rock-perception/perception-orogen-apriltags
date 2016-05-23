/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef APRILTAGS_TASK_TASK_HPP
#define APRILTAGS_TASK_TASK_HPP

#include "apriltags/TaskBase.hpp"
#include "frame_helper/FrameHelper.h"
#include <base/samples/RigidBodyState.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "sys/time.h"

#include "apriltags/apriltag.h"
#include "apriltags/common/image_u8.h"
#include "apriltags/tag36h11.h"
#include "apriltags/tag36h10.h"
#include "apriltags/tag36artoolkit.h"
#include "apriltags/tag25h9.h"
#include "apriltags/tag25h7.h"

#include "apriltags/common/zarray.h"
#include "apriltags/common/getopt.h"

namespace apriltags {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','apriltags::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
        friend class TaskBase;
        protected:

        apriltags::ApriltagsConfig conf;
        apriltag_family_t *tf;
        apriltag_detector_t *td;

        cv::Mat camera_k, camera_dist;
        cv::Mat rvec, tvec;
        // Maps for faster undistortion:
        cv::Mat undist_map1, undist_map2;
        double scaling;


        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> out_frame_ptr;

        std::map<int, double> apriltag_id_to_size_; //Mapping from ID to size

        void getRbs(base::samples::RigidBodyState &rbs, float markerSizeMeters, double points[][2], cv::Mat  camMatrix,cv::Mat distCoeff)throw(cv::Exception);
        void draw(cv::Mat &in, double p[][2], double c[], int id, cv::Scalar color, int lineWidth)const;
        void draw3dAxis(cv::Mat &Image, float marker_size, cv::Mat camera_matrix, cv::Mat dist_matrix);
        void draw3dCube(cv::Mat &Image,float marker_size,cv::Mat  camMatrix,cv::Mat distCoeff);
        double tic();
        std::string getMarkerFrameName(int i);
        std::string getMarkerIdentifierName(int i);

        public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "apriltags::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
         needs_configuration
         ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

