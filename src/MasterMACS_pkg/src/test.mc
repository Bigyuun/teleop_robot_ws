/*********************************************************************
**
**    This sample provides a prototype of an Aposs state machine
** program.  The state machine consists of two substates.  A timer
** is used to toggle between the substates.
*/

#include <SysDef.mh>
#include "..\include\SDK\SDK_ApossC.mc"
/*********************************************************************
** Application Defines
*********************************************************************/
enum K
{
	A=0,
	B
};

#define TIMER_DURATION  1000        // Timer duration in milliseconds.

/*********************************************************************
** Private Data Array Indices
*********************************************************************/

#define SAMPLE_MACHINE   1          // ID number of the sample state machine.
#define SAMPLE_DATA      2          // Length of private data for the sample state machine.

#define DATA_COUNTA      0          // Number of times substate A has been entered.
#define DATA_COUNTB      0          // Number of times substate B has been entered.

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {  0,          // Runtime flags.
                   10,          // Event pool size.
                    1,          // Maximum number of timers.
                    0,          // Subscribe pool size.
                    0,          // Parameter pool size.
                    0 }         // Position pool size.

/*********************************************************************
** Event Definitions
*********************************************************************/

SmEvent SIG_TICK { }    // The timer has timed out.

/*********************************************************************
** State Definitions
*********************************************************************/

SmState RootState
    {
    SIG_INIT = SmTrans(->SubStateA);

    SmState SubStateA
        {
        SIG_ENTRY = {
                    data[DATA_COUNTA]++;
                    print("Entering SubState A: cnt = ",data[DATA_COUNTA]);
                    }
        SIG_TICK  = SmTrans(SubStateB);
        }

    SmState SubStateB
        {
        SIG_ENTRY = EnterB;
        SIG_TICK  = SmTrans(SubStateA);
        }
    }

/*********************************************************************
** State Machine Definitions
*********************************************************************/

SmMachine Sample { SAMPLE_MACHINE, // State machine ID.
                   RootInit,       // Initialization function.
                   RootState,      // Top-most state.
                   5,              // Event queue size.
                   SAMPLE_DATA }   // Private data array size.

/*********************************************************************
** State Machine Initialization Functions
*********************************************************************/

// This routine is automatically called to initialize the state machine
// before it starts running.  If initialization is not necessary, then
// this routine can be deleted (and the reference in the "SmMachine"
// statement (above) changed to *).

long RootInit(long id, long data[])
{
    /*
    ** Initialize the state machine's private data values.  These
    ** variables can be used for any purpose required by the user.
    */

    data[DATA_COUNTA] = 0;
    data[DATA_COUNTB] = 0;

    /*
    ** Start a 1 second timer.
    */

    SmPeriod(TIMER_DURATION, id, SIG_TICK);

    return(0);
}

/*********************************************************************
** State Machine Event Functions
*********************************************************************/

// This is an example of an event function.  Explicitly defining
// an event function in this way allows the function to be shared
// by multiple states.  If this is not required, then the event
// function may also be defined within the SmState statement (above).
// Note that defining the function in the SmState statement will
// implicitly declare the function calling sequence to be identical
// to that shown here.

long EnterB(long id, long signal, long event[], long data[])
{
    data[DATA_COUNTB]++;
    print("Entering SubState B: cnt = ",data[DATA_COUNTB]);

    return(SmHandled);
}

/*********************************************************************
** Aposs Main Program
*********************************************************************/

long main(void)
{
    /*
    ** Start the state machine.
    */


    SmRun(Sample);

    return(0);
}
