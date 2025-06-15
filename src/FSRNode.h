//
// FSRNode.h
// Header file for the C++ implementation of the FSRNode module.
// This should be placed in the `src` directory.
//
#ifndef FSRNODE_H_
#define FSRNODE_H_

#include <omnetpp.h>
#include <map>
#include <vector>
#include <set>
#include <string>
#include <queue> // For Dijkstra's or similar pathfinding
#include <cmath> // For sqrt, pow
#include <limits> // For std::numeric_limits

using namespace omnetpp;

// Define a simple structure for a link state entry (from the paper)
struct LinkStateEntry {
    int sourceId;         // ID of the node originating this link state
    int neighborId;       // ID of the neighbor it has a link with
    int sequenceNumber;   // Sequence number (timestamp) of this entry
    // Add other link state information if needed, e.g., link cost/weight
};

// Define a structure to represent a route (for the routing table/next-hop table)
struct RouteEntry {
    int destinationId;
    int nextHopId;
    int distance;       // e.g., hop count
    int sequenceNumber; // Sequence number of the route, for freshness
};

// Define a structure for a packet (control or data)
class FSRPacket : public cPacket { // Inherits from cPacket
  public:
    enum PacketType {
        // Original FSR_UPDATE type can be general, or we can specialize.
        // Let's specialize for clear distinction in FSR 2-level
        DATA_PACKET = 1,
        FSR_LOCAL_UPDATE = 2,  // For frequent, local scope (direct neighbors) updates
        FSR_GLOBAL_UPDATE = 3  // For less frequent, global scope (full topology) updates
    };

    PacketType type;
    int sourceAddress;
    int destinationAddress;
    int sequenceNum; // Sequence number for data packets to track delivery and duplicates (per source)

    std::vector<LinkStateEntry> linkStates; // List of link state entries being sent for FSR_UPDATE

    // Constructor for FSRPacket
    FSRPacket(const char *name = nullptr, PacketType type = DATA_PACKET, short kind = 0)
        : cPacket(name, kind), type(type), sequenceNum(0) {} // Initialize sequenceNum

    // Custom copy constructor for duplicating messages
    FSRPacket(const FSRPacket& other) : cPacket(other) {
        type = other.type;
        sourceAddress = other.sourceAddress;
        destinationAddress = other.destinationAddress;
        sequenceNum = other.sequenceNum; // Copy sequenceNum
        linkStates = other.linkStates;   // Copy vector contents
    }

    // Override dup() for polymorphic copying
    virtual FSRPacket *dup() const override { return new FSRPacket(*this); }
};


class FSRNode : public cSimpleModule
{
  private:
    int myId;                           // This node's ID
    double transmissionRange;           // Communication range
    double playgroundX, playgroundY;    // Simulation area dimensions

    // Mobility parameters
    double maxSpeed;
    double minSpeed;
    double pauseTime;
    cMessage *mobilityTimer;            // Pointer to the scheduled mobility timer message
    double currentX, currentY;          // Current position
    double destX, destY;                // Current destination
    double currentSpeed;                // Current speed towards destination
    double lastMoveTime;                // Time of last position update

    // FSR-specific members
    cMessage *fsrUpdateTimer;           // Pointer to the scheduled FSR update timer message
    double fsrUpdateInterval;           // Interval for sending FSR updates (for local scope)
    // NEW: Global update timer and interval
    cMessage *fsrGlobalUpdateTimer;
    double fsrGlobalUpdateInterval;

    int numFisheyeScopes;               // Number of fisheye scopes (parameter exists, now directly used for decision)

    // Traffic generation members
    cMessage *trafficTimer;             // Timer for sending data packets
    double trafficInterarrivalTime;     // Time between sending data packets (e.g., for Poisson traffic)
    int numTrafficSessions;             // Number of nodes that will act as sources
    int myTrafficSequenceNum;           // Sequence number for data packets originating from this node
    int dataPacketSize;                 // Size of data packets in bytes

    // Data structures for FSR
    std::set<int> neighbors;            // Set of direct neighbors
    int currentSequenceNumber;          // Monotonically increasing sequence number for this node's LS updates

    // Topology Table (TT_i): Stores LinkStateEntry from all known nodes
    // Map: sourceNodeId -> map<neighborNodeId, LinkStateEntry>
    std::map<int, std::map<int, LinkStateEntry>> topologyTable;

    // Next Hop Table (NEXT_i): Stores next hop for each destination
    // Map: destinationId -> RouteEntry
    std::map<int, RouteEntry> nextHopTable;

    // Distance Table (D_i): Stores shortest distance (hop count) to each destination
    // Map: destinationId -> distance
    std::map<int, int> distanceTable;

    // Data for performance metrics
    long packetsSentControl;
    long packetsReceivedControl;
    long packetsSentData;       // Count of data packets originated by this node
    long packetsReceivedData;   // Count of data packets received by this node as destination
    long packetsForwardedData;  // NEW: Count of data packets forwarded by this node
    long dataBytesSentTotal;    // Total bytes of data sent (originating + forwarded)
    long dataBytesReceivedTotal; // Total bytes of data received (at this node as destination)
    long controlBytesSentTotal;
    long controlBytesReceivedTotal;

    // For Packet Delivery Ratio and End-to-End Delay, track sent data packets
    // Map: "sourceId_sequenceNum" -> {sendTime, destinationId}
    std::map<std::string, std::pair<simtime_t, int>> sentDataPacketsInfo;


    // Signals for recording statistics (emitted during simulation)
    simsignal_t endToEndDelaySignal;        // End-to-end delay for data packets
    simsignal_t throughputSignal;           // Throughput (bytes/sec) - derived from dataBytesReceivedTotal
    simsignal_t packetDeliveryRatioSignal;  // PDR - derived from sent/received counts
    simsignal_t dataOverheadSignal;         // Data overhead ratio
    simsignal_t controlOverheadSignal;      // Control overhead ratio

    // NEW: Raw count signals to enable flexible network-wide aggregation in analysis editor
    simsignal_t packetsSentDataAsSourceSignal;
    simsignal_t packetsReceivedDataAsDestinationSignal;
    simsignal_t dataBytesSentTotalSignal;
    simsignal_t dataBytesReceivedTotalSignal;
    simsignal_t controlBytesSentTotalSignal;
    simsignal_t controlBytesReceivedTotalSignal;


    // OMNeT++ display string (not directly used as an object, but set via getDisplayString())
    cXMLElement* displayString;


  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void finish() override; // For emitting final scalar results

    // FSR specific functions
    void updateFSR();
    // Renamed sendLinkStateUpdate to be specific for LOCAL scope updates
    void sendLocalScopeUpdate();
    // NEW: Function to send a full topology update for global scope
    void sendGlobalScopeUpdate();
    void processLinkStateUpdate(FSRPacket *pkt);
    void computeShortestPaths();
    void updateNeighborList();
    void sendDataPacket(int destination); // Called by traffic generator

    // Traffic generation function
    void generateDataTraffic(); // Function to generate and send data packets

    // Mobility functions
    void scheduleMobilityUpdate();
    void updatePosition();
    void setNewDestination();
    double calculateDistance(double x1, double y1, double x2, double y2);

    // Helper function for logging (declared here)
    void log(const std::string& message) const;

  public:
    // Public getter methods for other nodes to query position for neighbor discovery
    double getX() const { return currentX; }
    double getY() const { return currentY; }

    // Virtual destructor for proper cleanup of scheduled events/messages
    virtual ~FSRNode();
};

#endif /* FSRNODE_H_ */
