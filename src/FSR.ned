//
// FSR.ned
// Defines the top-level network for the Fisheye State Routing simulation.
// This should be placed in the `src` directory (or a suitable sub-directory)
// within your OMNeT++ project.
//
package fsr.src.src;

// Import the FSRNode module.
import fsr.src.src.FSRNode;

network FSR
{
    parameters:
        // Number of nodes in the network. You will vary this in omnetpp.ini.
        int numNodes = default(25);
        // Simulation area dimensions (e.g., 1000x1000 meters as in the paper).
        double playgroundX @unit(m) = default(1000m);
        double playgroundY @unit(m) = default(1000m);
        // NEW: numTrafficSessions is a parameter of the network
        int numTrafficSessions = default(5); // Default value at network level

    submodules:
        // Define an array of FSRNode modules.
        node[numNodes]: FSRNode {
            // Pass playground dimensions from the network module to each node.
            playgroundX = parent.playgroundX;
            playgroundY = parent.playgroundY;
            // NEW: Pass numTrafficSessions down to the nodes
            numTrafficSessions = parent.numTrafficSessions;
            @display("p=500,500;is=s");
        }
    connections allowunconnected:
        // All nodes are connected to a single broadcast medium (like a wireless channel).
        // In OMNeT++, this is typically modeled by having all nodes connect to a "radio" or "nic" module
        // which then connects to a "channel" or "medium" module.
        // For simplicity here, we'll assume a direct connection for message passing within the simulation
        // and let the FSRNode handle the concept of 'neighbors' based on transmission range.
        // A more realistic setup would involve a WirelessInterface module and an IdealRadioChannel.
        // For this basic setup, messages can be sent between any two nodes.
        // The FSR logic itself will determine "neighbor" based on proximity/transmission range.

        // In a realistic wireless simulation, you'd define a physical layer and MAC layer.
        // For a conceptual FSR implementation, nodes can directly send messages to others,
        // and the FSR protocol itself will decide who is a "neighbor" based on a virtual range.
        // For now, we'll keep it simple and assume a "fully connected" logical view,
        // with the FSRNode's C++ code determining neighbor relationships.

        // Example: If you were using a wireless medium:
        // channel: IdealRadioChannel;
        // for i=0..numNodes-1 {
        //     node[i].radio <--> channel;
        // }
}
