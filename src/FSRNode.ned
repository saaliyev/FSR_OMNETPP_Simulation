//
// FSRNode.ned
// Defines the OMNeT++ module for a single Fisheye State Routing node.
// This should be placed in the `src` directory (or a suitable sub-directory)
// within your OMNeT++ project.
//
package fsr.src.src;

// Define the FSRNode module.
simple FSRNode
{
    parameters:
        int myId = default(-1);
        double transmissionRange @unit(m) = default(250m);
        double maxSpeed @unit(mps) = default(20mps);
        double minSpeed @unit(mps) = default(10mps);
        double pauseTime @unit(s) = default(1s);
        double playgroundX @unit(m); // Still no default here, as it's passed from parent
        double playgroundY @unit(m); // Still no default here, as it's passed from parent

        double fsrUpdateInterval @unit(s) = default(1.0s); // Interval for local scope updates
        double fsrGlobalUpdateInterval @unit(s) = default(5.0s); // NEW: Interval for global scope updates (less frequent)
        int numFisheyeScopes = default(2); // Parameter exists, now used for conceptual 2-level FSR

        // Traffic generation parameters (with defaults now)
        double trafficInterarrivalTime @unit(s) = default(2.5s);
        int numTrafficSessions; // Parameter received from parent network
        int dataPacketSize @unit(B) = default(512B);

    gates:
        inout radio;
}
