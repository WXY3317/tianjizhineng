/**
 * @file example_basic_LinkSystem.cpp
 * @brief Example demonstrating how to establish and terminate communication
 *        with the L1 robot controller system.
 *
 * This example shows how to:
 * - Query the SDK version
 * - Establish a communication link with the robot controller
 * - Check the communication link state
 * - Retrieve the controller firmware version
 * - Safely disconnect from the controller
 *
 * This is the most fundamental example and should be executed before
 * any other robot control examples.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

void CheckLinkState()
{
    int link_state = FX_L1_System_GetLinkState();
    if (link_state == 0)
    {
        printf("Link is not established\n");
    }
    else if (link_state == -1)
    {
        printf("Link is established, but no data arrived in 100ms\n");
    }
    else if (link_state == 1)
    {
        printf("Link is established\n");
    }
    else
    {
        printf("Link state is unknown(%d)\n", link_state);
    }
}

/**
 * @brief Entry point of the system link example.
 *
 * Workflow overview:
 * 1. Retrieve the SDK version
 * 2. Establish communication with the robot controller
 * 3. Check the communication link state
 * 4. Retrieve the controller firmware version
 * 5. Disconnect from the controller
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version

    /* Get SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with the controller */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }
    
    /* Get link state */
    printf("Press any key to check link state...\n");
    getchar();
    CheckLinkState();

    /* Get controller version */
    printf("Press any key to get controller version...\n");
    getchar();
    controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /* Disconnect from the controller */
    printf("Press any key to unlink system...\n");
    getchar();
    FX_L1_System_Unlink();
    printf("Unlink system\n");

    printf("Press any key to check link state...\n");
    getchar();
    CheckLinkState();

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}