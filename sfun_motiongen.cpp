#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  sfun_motiongen

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#include <ruckig/ruckig.hpp>
using namespace ruckig;

#define NPARAMS 8

#define SAMPLE_TIME_IDX 0
#define SAMPLE_TIME_PARAM(S) ssGetSFcnParam(S,SAMPLE_TIME_IDX)

#define NUMBER_OF_DOFS_IDX 1
#define NUMBER_OF_DOFS_PARAM(S) ssGetSFcnParam(S,NUMBER_OF_DOFS_IDX)

#define LIMIT_VEL_IDX 2
#define LIMIT_VEL_PARAM(S) ssGetSFcnParam(S,LIMIT_VEL_IDX)

#define LIMIT_ACC_IDX 3
#define LIMIT_ACC_PARAM(S) ssGetSFcnParam(S,LIMIT_ACC_IDX)

#define LIMIT_JRK_IDX 4
#define LIMIT_JRK_PARAM(S) ssGetSFcnParam(S,LIMIT_JRK_IDX)

#define CONTROL_INTERFACE_IDX 5
#define CONTROL_INTERFACE_PARAM(S) ssGetSFcnParam(S,CONTROL_INTERFACE_IDX)

#define SYNCHRONIZATION_IDX 6
#define SYNCHRONIZATION_PARAM(S) ssGetSFcnParam(S,SYNCHRONIZATION_IDX)

#define DURATION_DISCRETIZATION_IDX 7
#define DURATION_DISCRETIZATION_PARAM(S) ssGetSFcnParam(S,DURATION_DISCRETIZATION_IDX)

#define MDL_CHECK_PARAMETERS
#if defined(MATLAB_MEX_FILE)
// Function: mdlCheckParameters =============================================
// Abstract:
//    mdlCheckParameters verifies new parameter settings whenever parameter
//    change or are re-evaluated during a simulation. When a simulation is
//    running, changes to S-function parameters can occur at any time during
//    the simulation loop.
static void mdlCheckParameters(SimStruct *S)
{
    if (mxGetScalar(SAMPLE_TIME_PARAM(S)) < 0.0) {
        ssSetErrorStatus(S, "Sample time must be a positive scalar.");
        return;
    }
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    if (ndofs < 0) {
        ssSetErrorStatus(S, "Number of Dof's must be a positive integer.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_VEL_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have a velocity limit defined.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_ACC_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have an acceleration limit defined.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_JRK_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have a jerk limit defined.");
        return;
    }
}
#endif


// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 6;  // number of input ports
    int_T nOutputPorts = 3;  // number of output ports
    int_T needsInput   = 1;  // direct feed through
    
    ssSetNumSFcnParams(S, NPARAMS);  // Number of expected parameters
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
        return; // Simulink will report a parameter mismatch error
    }
    #endif
    ssSetSFcnParamTunable(S, SAMPLE_TIME_IDX, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, NUMBER_OF_DOFS_IDX, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_VEL_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_ACC_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_JRK_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, CONTROL_INTERFACE_IDX, SS_PRM_NOT_TUNABLE);    
    ssSetSFcnParamTunable(S, SYNCHRONIZATION_IDX, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, DURATION_DISCRETIZATION_IDX, SS_PRM_NOT_TUNABLE);
    
    // Register the number and type of states the S-Function uses
    ssSetNumContStates(S, 0);   // number of continuous states
    ssSetNumDiscStates(S, 0);   // number of discrete states
    
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    
    // Configure the input ports. First set the number of input ports.
    if (!ssSetNumInputPorts(S, nInputPorts)) return;
    if (!ssSetInputPortVectorDimension(S, 0, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    if (!ssSetInputPortVectorDimension(S, 1, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 1, 0);
    if (!ssSetInputPortVectorDimension(S, 2, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 2, 0);
    if (!ssSetInputPortVectorDimension(S, 3, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 3, needsInput);
    if (!ssSetInputPortVectorDimension(S, 4, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 4, needsInput);
    if (!ssSetInputPortVectorDimension(S, 5, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 5, needsInput);
    
    // Configure the output ports. First set the number of output ports.
    if (!ssSetNumOutputPorts(S,nOutputPorts)) return;
    if (!ssSetOutputPortVectorDimension(S, 0, ndofs)) return;
    if (!ssSetOutputPortVectorDimension(S, 1, ndofs)) return;
    if (!ssSetOutputPortVectorDimension(S, 2, ndofs)) return;
    
    ssSetNumSampleTimes(S, 1);  // number of sample times
    
    // Set size of the work vectors.
    ssSetNumRWork(S, 0);   // number of real work vector elements
    ssSetNumIWork(S, 1);   // number of integer work vector elements
    ssSetNumPWork(S, 3);   // number of pointer work vector elements
    ssSetNumModes(S, 0);   // number of mode work vector elements
    ssSetNumNonsampledZCs(S, 0);   // number of nonsampled zero crossings
    
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    
    #if defined(MATLAB_MEX_FILE)
    ssSetOptions(S, 
                 SS_OPTION_DISCRETE_VALUED_OUTPUT);
    #else
    ssSetOptions(S,
                 SS_OPTION_DISCRETE_VALUED_OUTPUT |
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE);
    #endif
}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(SAMPLE_TIME_PARAM(S)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


#define MDL_INITIALIZE_CONDITIONS
// Function: mdlInitializeConditions ========================================
// Abstract:
//    In this function, you should initialize the continuous and discrete
//    states for your S-function block.  The initial states are placed
//    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
//    You can also perform any other initialization activities that your
//    S-function may require. Note, this routine will be called at the
//    start of simulation and if it is present in an enabled subsystem
//    configured to reset states, it will be call when the enabled
//    subsystem restarts execution to reset the states.
static void mdlInitializeConditions(SimStruct *S)
{
    // The mdlInitializeConditions method is called when the simulation
    // start and every time an enabled subsystem is re-enabled.

    // Reset the IWork flag to 1 when values need to be reinitialized.
    ssSetIWorkValue(S, 0, 1);
}


// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you have
//   states that should be initialized once, this is the place to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Create instances: the Ruckig OTG as well as input and output parameters
    const unsigned int ndofs = static_cast<unsigned int>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    auto *OTG = new Ruckig<DynamicDOFs>( ndofs, mxGetScalar(SAMPLE_TIME_PARAM(S)) );
    auto *IP  = new InputParameter<DynamicDOFs>( ndofs );
    auto *OP  = new OutputParameter<DynamicDOFs>( ndofs );

    
    const char* control = mxArrayToString(CONTROL_INTERFACE_PARAM(S));
    if (strcmp(control,"POSITION") == 0){
        IP->control_interface = ControlInterface::Position;
    }
    else if (strcmp(control,"VELOCITY") == 0){
        IP->control_interface = ControlInterface::Velocity;
    }
    else {
        IP->control_interface = ControlInterface::Position;
    }
    
    const char* sync = mxArrayToString(SYNCHRONIZATION_PARAM(S));
    if (strcmp(sync,"NO_SYNCHRONIZATION") == 0){
        IP->synchronization = Synchronization::None;
    }
    else if (strcmp(sync,"TIME_SYNCHRONIZATION_IF_NECESSARY") == 0){
        IP->synchronization = Synchronization::TimeIfNecessary;
    }
    else if (strcmp(sync,"ONLY_TIME_SYNCHRONIZATION") == 0){
        IP->synchronization = Synchronization::Time;
    }
    else if (strcmp(sync,"ONLY_PHASE_SYNCHRONIZATION") == 0){
        IP->synchronization = Synchronization::Phase;
    }
    else {
        IP->synchronization = Synchronization::Time;
    }
    
    const char* duration = mxArrayToString(DURATION_DISCRETIZATION_PARAM(S));
    if (strcmp(duration,"CONTINUOUS") == 0){
        IP->duration_discretization = DurationDiscretization::Continuous;
    }
    else if (strcmp(duration,"DISCRETE") == 0){
        IP->duration_discretization = DurationDiscretization::Discrete;
    }
    else {
        IP->duration_discretization = DurationDiscretization::Continuous;
    }
    
    // Store new C++ object in the pointers vector
    ssGetPWork(S)[0] = OTG;
    ssGetPWork(S)[1] = IP;
    ssGetPWork(S)[2] = OP;
}


// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get data addresses of Outputs
    real_T *pos = ssGetOutputPortRealSignal(S, 0);
    real_T *vel = ssGetOutputPortRealSignal(S, 1);
    real_T *acc = ssGetOutputPortRealSignal(S, 2);
    
    // Initialize values if the IWork vector flag is true. 
    if (ssGetIWorkValue(S, 0) == 1) {
        // Get data addresses of nputs
        InputRealPtrsType current_pos_ptr = ssGetInputPortRealSignalPtrs(S,3);
        InputRealPtrsType current_vel_ptr = ssGetInputPortRealSignalPtrs(S,4);
        InputRealPtrsType current_acc_ptr = ssGetInputPortRealSignalPtrs(S,5);
        
        const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
        for (int_T dof=0; dof < ndofs; dof++)
        {
            pos[dof] = *current_pos_ptr[dof];
            vel[dof] = *current_vel_ptr[dof];
            acc[dof] = *current_acc_ptr[dof];
        }
       
        ssSetIWorkValue(S, 0, 0);
    }
    else
    {
        // Retrieve C++ object from the pointers vector
        auto *OP = static_cast<OutputParameter<DynamicDOFs> *>(ssGetPWork(S)[2]);
        
        const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
        for (int_T dof=0; dof < ndofs; dof++)
        {
            pos[dof] = OP->new_position[dof];
            vel[dof] = OP->new_velocity[dof];
            acc[dof] = OP->new_acceleration[dof];
        }
    }
}

#define MDL_UPDATE
// Function: mdlUpdate ======================================================
// Abstract:
//    This function is called once for every major integration time step.
//    Discrete states are typically updated here, but this function is
//    useful for performing any tasks that should only take place once per
//    integration step.
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // Retrieve C++ object from the pointers vector
    auto *OTG   = static_cast<Ruckig<DynamicDOFs> *>(ssGetPWork(S)[0]);
    auto *IP    = static_cast<InputParameter<DynamicDOFs> *>(ssGetPWork(S)[1]);
    auto *OP    = static_cast<OutputParameter<DynamicDOFs> *>(ssGetPWork(S)[2]);
    
    // Get data addresses of I/O
    InputRealPtrsType target_pos_ptr  = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType target_vel_ptr  = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType target_acc_ptr  = ssGetInputPortRealSignalPtrs(S,2);
    InputRealPtrsType current_pos_ptr = ssGetInputPortRealSignalPtrs(S,3);
    InputRealPtrsType current_vel_ptr = ssGetInputPortRealSignalPtrs(S,4);
    InputRealPtrsType current_acc_ptr = ssGetInputPortRealSignalPtrs(S,5);
            
    // Set-up the input parameters
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    for (int_T dof=0; dof < ndofs; dof++)
    {
        IP->current_position[dof]     = *current_pos_ptr[dof];
        IP->current_velocity[dof]     = *current_vel_ptr[dof];
        IP->current_acceleration[dof] = *current_acc_ptr[dof];
        IP->max_velocity[dof]         = mxGetPr(LIMIT_VEL_PARAM(S))[dof];
        IP->max_acceleration[dof]     = mxGetPr(LIMIT_ACC_PARAM(S))[dof];
        IP->max_jerk[dof]             = mxGetPr(LIMIT_JRK_PARAM(S))[dof];
        IP->target_position[dof]      = *target_pos_ptr[dof];
        IP->target_velocity[dof]      = *target_vel_ptr[dof];
        IP->target_acceleration[dof]  = *target_acc_ptr[dof];
    }
    
    // Calling the Ruckig OTG algorithm
    int_T ResultValue = OTG->update(*IP,*OP);
    
    #if defined(MATLAB_MEX_FILE)
    if (ResultValue < 0) 
    {
        static char msg[256];
        sprintf(msg,"An error occurred with Ruckig OTG algorithm (%d).\n", ResultValue );
        ssSetErrorStatus(S,msg);
        return; 
    }
    #endif
}


// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // Retrieve and destroy C++ objects
    auto *OTG   = static_cast<Ruckig<DynamicDOFs> *>(ssGetPWork(S)[0]);
    auto *IP    = static_cast<InputParameter<DynamicDOFs> *>(ssGetPWork(S)[1]);
    auto *OP    = static_cast<OutputParameter<DynamicDOFs> *>(ssGetPWork(S)[2]);
    
    // Deleting the objects of the Ruckig Library end terminating the process
    delete  OTG;
    delete  IP;
    delete  OP;
}


#define MDL_RTW  /* Change to #undef to remove function */
#if defined(MATLAB_MEX_FILE)
// Function: mdlRTW =========================================================
// Abstract:
//
//    This function is called when the Real-Time Workshop is generating
//    the model.rtw file. In this method, you can call the following
//    functions which add fields to the model.rtw file.
static void mdlRTW(SimStruct *S)
{
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    
    // Parameter records for tunable S-function parameters.
    if (!ssWriteRTWParameters(S, 3,
            SSWRITE_VALUE_VECT,"VelLimit","",mxGetPr(LIMIT_VEL_PARAM(S)),ndofs,
            SSWRITE_VALUE_VECT,"AccLimit","",mxGetPr(LIMIT_ACC_PARAM(S)),ndofs,
            SSWRITE_VALUE_VECT,"JrkLimit","",mxGetPr(LIMIT_JRK_PARAM(S)),ndofs) )
    {
        return;
    }
    
    // Parameter records for non-tunable S-function parameters.
    if (!ssWriteRTWParamSettings(S, 5,
            SSWRITE_VALUE_NUM,"SampleTime",mxGetScalar(SAMPLE_TIME_PARAM(S)),
            SSWRITE_VALUE_NUM,"NumberOfDoFs",mxGetScalar(NUMBER_OF_DOFS_PARAM(S)),
            SSWRITE_VALUE_STR,"ControlInterface",mxArrayToString(CONTROL_INTERFACE_PARAM(S)),
            SSWRITE_VALUE_STR,"Synchronization",mxArrayToString(SYNCHRONIZATION_PARAM(S)),
            SSWRITE_VALUE_STR,"DurationDiscretization",mxArrayToString(DURATION_DISCRETIZATION_PARAM(S)) ))
    {
        return;
    }
    
    // Work vector records for S-functions
    if (!ssWriteRTWWorkVect(S, "IWork", 1,
            "BlockInit", 1) )
    {
        return;
    }
    
    // Work vector records for S-functions
    if (!ssWriteRTWWorkVect(S, "PWork", 3,
            "Ruckig", 1,
            "InputParameter", 1,
            "OutputParameter", 1) )
    {
        return;
    }
}
#endif /* MDL_RTW */


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism
#else
#include "cg_sfun.h"       // Code generation registration function
#endif
