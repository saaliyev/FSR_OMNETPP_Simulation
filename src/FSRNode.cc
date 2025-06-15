//
// FSRNode.cc
// Source file for the C++ implementation of the FSRNode module.
// This should be placed in the `src` directory.
//
#include "FSRNode.h"
// Standard C++ Libraries for math and containers
#include <vector>
#include <algorithm> // For std::find, std::min
#include <limits>    // For std::numeric_limits
#include <cmath>     // For sqrt, pow
#include <string>    // For std::to_string
#include <queue>     // For std::priority_queue

// Register the module with OMNeT++.
Define_Module(FSRNode);

// Helper function for logging (now properly declared in .h)
void FSRNode::log(const std::string& message) const {
    EV_INFO << "Node " << myId << ": " << message << endl;
}

FSRNode::~FSRNode() {
    // Correct cleanup for self-messages (timers).
    if (fsrUpdateTimer != nullptr) {
        if (fsrUpdateTimer->isScheduled()) {
            cancelEvent(fsrUpdateTimer);
        }
        delete fsrUpdateTimer;
        fsrUpdateTimer = nullptr;
    }

    if (mobilityTimer != nullptr) {
        if (mobilityTimer->isScheduled()) {
            cancelEvent(mobilityTimer);
        }
        delete mobilityTimer;
        mobilityTimer = nullptr;
    }

    // Cleanup for trafficTimer
    if (trafficTimer != nullptr) {
        if (trafficTimer->isScheduled()) {
            cancelEvent(trafficTimer);
        }
        delete trafficTimer;
        trafficTimer = nullptr;
    }

    // NEW: Cleanup for fsrGlobalUpdateTimer
    if (fsrGlobalUpdateTimer != nullptr) {
        if (fsrGlobalUpdateTimer->isScheduled()) {
            cancelEvent(fsrGlobalUpdateTimer);
        }
        delete fsrGlobalUpdateTimer;
        fsrGlobalUpdateTimer = nullptr;
    }
}

void FSRNode::initialize()
{
    // Get parameters from NED file
    myId = getIndex();
    transmissionRange = par("transmissionRange");
    playgroundX = par("playgroundX");
    playgroundY = par("playgroundY");
    maxSpeed = par("maxSpeed");
    minSpeed = par("minSpeed");
    pauseTime = par("pauseTime");
    fsrUpdateInterval = par("fsrUpdateInterval");
    // NEW: Get global update interval
    fsrGlobalUpdateInterval = par("fsrGlobalUpdateInterval");
    numFisheyeScopes = par("numFisheyeScopes"); // Parameter to be used for logic now

    // Get traffic parameters
    trafficInterarrivalTime = par("trafficInterarrivalTime");
    numTrafficSessions = par("numTrafficSessions");
    dataPacketSize = par("dataPacketSize");

    displayString = nullptr;

    // Initialize mobility
    currentX = uniform(0, playgroundX);
    currentY = uniform(0, playgroundY);
    setNewDestination();

    mobilityTimer = new cMessage("mobilityTimer");
    fsrUpdateTimer = new cMessage("fsrUpdateTimer");
    trafficTimer = new cMessage("trafficTimer");
    fsrGlobalUpdateTimer = new cMessage("fsrGlobalUpdateTimer"); // NEW timer

    // Schedule initial events
    scheduleAt(simTime() + uniform(0, pauseTime), mobilityTimer);
    scheduleAt(simTime() + fsrUpdateInterval, fsrUpdateTimer); // Schedule local updates
    // NEW: Schedule global updates (less frequent)
    scheduleAt(simTime() + fsrGlobalUpdateInterval, fsrGlobalUpdateTimer);
    lastMoveTime = simTime().dbl();

    // Initialize FSR-specific data structures
    currentSequenceNumber = 0;

    // Initialize traffic sequence number
    myTrafficSequenceNum = 0;

    // Initialize performance metrics
    packetsSentControl = 0;
    packetsReceivedControl = 0;
    packetsSentData = 0; // Packets sent by this node as a SOURCE
    packetsReceivedData = 0; // Packets received by this node as a DESTINATION
    packetsForwardedData = 0; // Track packets forwarded by this node
    dataBytesSentTotal = 0; // Bytes sent by this node (source + forwarded)
    dataBytesReceivedTotal = 0; // Bytes received by this node (destination)
    controlBytesSentTotal = 0;
    controlBytesReceivedTotal = 0;

    // Register signals for recording statistics
    endToEndDelaySignal = registerSignal("endToEndDelay");
    throughputSignal = registerSignal("throughput");
    packetDeliveryRatioSignal = registerSignal("packetDeliveryRatio");
    dataOverheadSignal = registerSignal("dataOverhead");
    controlOverheadSignal = registerSignal("controlOverhead");
    packetsSentDataAsSourceSignal = registerSignal("packetsSentDataAsSource");
    packetsReceivedDataAsDestinationSignal = registerSignal("packetsReceivedDataAsDestination");
    dataBytesSentTotalSignal = registerSignal("dataBytesSentTotal");
    dataBytesReceivedTotalSignal = registerSignal("dataBytesReceivedTotal");
    controlBytesSentTotalSignal = registerSignal("controlBytesSentTotal");
    controlBytesReceivedTotalSignal = registerSignal("controlBytesReceivedTotal");


    log("Initialized at (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");

    // Initial neighbor discovery and link state update
    updateNeighborList(); // This also triggers sendLocalScopeUpdate if neighbors change
    computeShortestPaths();

    // Start traffic generation for some nodes based on numTrafficSessions
    if (myId < numTrafficSessions) {
        scheduleAt(simTime() + uniform(0, trafficInterarrivalTime), trafficTimer);
        log("Node is a traffic source, scheduling first data packet.");
    }
}

void FSRNode::handleMessage(cMessage *msg)
{
    if (msg == fsrUpdateTimer) {
        // This timer triggers local scope updates (frequent)
        updateNeighborList(); // This will call sendLocalScopeUpdate if neighbors changed
        sendLocalScopeUpdate(); // Also send it regularly, even if neighbors don't change
        scheduleAt(simTime() + fsrUpdateInterval, fsrUpdateTimer);
    } else if (msg == fsrGlobalUpdateTimer) {
        // This timer triggers global scope updates (less frequent)
        sendGlobalScopeUpdate();
        scheduleAt(simTime() + fsrGlobalUpdateInterval, fsrGlobalUpdateTimer);
    }
    else if (msg == mobilityTimer) {
        updatePosition();
        scheduleMobilityUpdate();
    } else if (msg == trafficTimer) {
        generateDataTraffic();
        scheduleAt(simTime() + exponential(trafficInterarrivalTime), trafficTimer);
    } else {
        FSRPacket *pkt = dynamic_cast<FSRPacket*>(msg);
        if (!pkt) {
            log("Received unknown or non-FSRPacket message type.");
            delete msg;
            return;
        }

        if (msg->arrivedOn("radio$i")) {
            // Check if packet is for this node (destinationAddress is myId or broadcast -1)
            // Updated to handle new FSR packet types
            if (pkt->destinationAddress == myId || pkt->destinationAddress == -1) {
                if (pkt->type == FSRPacket::FSR_LOCAL_UPDATE || pkt->type == FSRPacket::FSR_GLOBAL_UPDATE) {
                    processLinkStateUpdate(pkt);
                } else if (pkt->type == FSRPacket::DATA_PACKET) {
                    if (pkt->destinationAddress == myId) {
                        packetsReceivedData++;
                        dataBytesReceivedTotal += pkt->getByteLength();
                        log("Received DATA_PACKET (seq=" + std::to_string(pkt->sequenceNum) + ") from " + std::to_string(pkt->sourceAddress) + " for self.");

                        std::string packetId = std::to_string(pkt->sourceAddress) + "_" + std::to_string(pkt->sequenceNum);
                        auto it = sentDataPacketsInfo.find(packetId);
                        if (it != sentDataPacketsInfo.end()) {
                            simtime_t originalSendTime = it->second.first;
                            emit(endToEndDelaySignal, simTime() - originalSendTime);
                        } else {
                            log("Warning: Received data packet " + packetId + " but no matching sent record found.");
                        }
                        emit(packetsReceivedDataAsDestinationSignal, 1);
                        emit(dataBytesReceivedTotalSignal, pkt->getByteLength());

                    } else {
                        log("Forwarding DATA_PACKET (seq=" + std::to_string(pkt->sequenceNum) + ") for " + std::to_string(pkt->destinationAddress));
                        auto it = nextHopTable.find(pkt->destinationAddress);
                        if (it != nextHopTable.end()) {
                            int nextHop = it->second.nextHopId;
                            FSRPacket *forwardPkt = pkt->dup();

                            cModule *targetModule = getParentModule()->getSubmodule("node", nextHop);
                            if (targetModule) {
                                cGate *targetGate = targetModule->gate("radio$i");
                                sendDirect(forwardPkt, targetGate);
                                packetsForwardedData++;
                                dataBytesSentTotal += forwardPkt->getByteLength();
                                emit(dataBytesSentTotalSignal, forwardPkt->getByteLength());
                            } else {
                                log("Error: Target module " + std::to_string(nextHop) + " not found for forwarding. Dropping packet.");
                                delete forwardPkt;
                            }
                        } else {
                            log("No route found for destination " + std::to_string(pkt->destinationAddress) + ". Dropping packet.");
                        }
                    }
                }
            } else {
                log("Received packet not for me or broadcast, ignoring. Dest: " + std::to_string(pkt->destinationAddress));
            }
        }
        delete pkt;
    }
}

void FSRNode::refreshDisplay() const {
    std::string pos_str = "p=" + std::to_string((int)currentX) + "," + std::to_string((int)currentY);
    getDisplayString().set(pos_str.c_str());
}

void FSRNode::finish() {
    EV << "Node " << myId << " final statistics:" << endl;
    EV << "  Data packets sent (as source): " << packetsSentData << endl;
    EV << "  Data packets forwarded: " << packetsForwardedData << endl;
    EV << "  Data bytes sent (as source/forwarder): " << dataBytesSentTotal << endl;
    EV << "  Data packets received (as destination): " << packetsReceivedData << endl;
    EV << "  Data bytes received (as destination): " << dataBytesReceivedTotal << endl;
    EV << "  Control packets sent: " << packetsSentControl << endl;
    EV << "  Control bytes sent: " << controlBytesSentTotal << endl;
    EV << "  Control packets received: " << packetsReceivedControl << endl;
    EV << "  Control bytes received: " << controlBytesReceivedTotal << endl;

    emit(packetsSentDataAsSourceSignal, (double)packetsSentData);
    emit(packetsReceivedDataAsDestinationSignal, (double)packetsReceivedData);
    emit(dataBytesSentTotalSignal, (double)dataBytesSentTotal);
    emit(dataBytesReceivedTotalSignal, (double)dataBytesReceivedTotal);
    emit(controlBytesSentTotalSignal, (double)controlBytesSentTotal);
    emit(controlBytesReceivedTotalSignal, (double)controlBytesReceivedTotal);
}


void FSRNode::updateFSR()
{
    log("Initiating FSR update cycle.");
    // Neighbor list update will trigger sendLocalScopeUpdate if neighbors change.
    updateNeighborList();
    computeShortestPaths();
}

void FSRNode::sendLocalScopeUpdate()
{
    log("Sending LOCAL Scope FSR Update (direct neighbors only).");
    FSRPacket *updatePkt = new FSRPacket("FSR_LOCAL_UPDATE", FSRPacket::FSR_LOCAL_UPDATE);
    updatePkt->sourceAddress = myId;
    updatePkt->destinationAddress = -1; // Broadcast
    updatePkt->setBitLength(1000); // Example size
    currentSequenceNumber++; // Increment sequence for fresh local info

    // Include only direct neighbors (1-hop scope)
    for (int neighborId : neighbors) {
        LinkStateEntry entry;
        entry.sourceId = myId;
        entry.neighborId = neighborId;
        entry.sequenceNumber = currentSequenceNumber;
        updatePkt->linkStates.push_back(entry);
    }

    int totalNodes = getParentModule()->par("numNodes");
    for (int i = 0; i < totalNodes; ++i) {
        if (i == myId) continue;
        cModule *targetModule = getParentModule()->getSubmodule("node", i);
        if (targetModule) {
            cGate *targetGate = targetModule->gate("radio$i");
            FSRPacket *pktCopy = updatePkt->dup();
            sendDirect(pktCopy, targetGate);
        } else {
            log("Error: Target module " + std::to_string(i) + " not found for local update broadcast.");
        }
    }
    packetsSentControl++;
    controlBytesSentTotal += updatePkt->getByteLength();
    emit(controlBytesSentTotalSignal, updatePkt->getByteLength());
    delete updatePkt;
}

void FSRNode::sendGlobalScopeUpdate()
{
    log("Sending GLOBAL Scope FSR Update (all known topology info).");
    FSRPacket *updatePkt = new FSRPacket("FSR_GLOBAL_UPDATE", FSRPacket::FSR_GLOBAL_UPDATE);
    updatePkt->sourceAddress = myId;
    updatePkt->destinationAddress = -1; // Broadcast
    updatePkt->setBitLength(2000); // Larger example size for global updates (more info)

    // Include all known link states from the topologyTable
    // The sequence numbers carried will be those from the original source nodes,
    // reflecting the freshness (or staleness) of that information.
    for (const auto& sourceEntry : topologyTable) {
        for (const auto& neighborEntry : sourceEntry.second) {
            updatePkt->linkStates.push_back(neighborEntry.second);
        }
    }
    // Also include own direct links, ensuring they are fresh if not already covered
    // (though sendLocalScopeUpdate handles this frequently).
    // This ensures that even global updates carry the latest about self.
    // To avoid massive duplication and maintain consistency with sequence numbers,
    // it's better to ensure only the latest versions from topologyTable are included,
    // and rely on processLinkStateUpdate to handle freshness.
    // For extreme simplicity, we can just flood all entries in topologyTable.
    // If the node itself is the source of a link, its sequence number will be higher (fresher)
    // than any relayed info from other nodes for its own direct links.
    // The processLinkStateUpdate logic ensures only newer entries are stored.

    int totalNodes = getParentModule()->par("numNodes");
    for (int i = 0; i < totalNodes; ++i) {
        if (i == myId) continue;
        cModule *targetModule = getParentModule()->getSubmodule("node", i);
        if (targetModule) {
            cGate *targetGate = targetModule->gate("radio$i");
            FSRPacket *pktCopy = updatePkt->dup();
            sendDirect(pktCopy, targetGate);
        } else {
            log("Error: Target module " + std::to_string(i) + " not found for global update broadcast.");
        }
    }
    packetsSentControl++; // Count as a control packet sent
    controlBytesSentTotal += updatePkt->getByteLength(); // Add to control bytes
    emit(controlBytesSentTotalSignal, updatePkt->getByteLength());
    delete updatePkt;
}


void FSRNode::processLinkStateUpdate(FSRPacket *pkt)
{
    log("Processing Link State Update from " + std::to_string(pkt->sourceAddress) + " (Type: " + std::to_string(pkt->type) + ").");
    controlBytesReceivedTotal += pkt->getByteLength();
    packetsReceivedControl++;
    emit(controlBytesReceivedTotalSignal, pkt->getByteLength());

    bool topologyChanged = false;
    for (const auto& receivedEntry : pkt->linkStates) {
        bool foundExisting = false;
        if (topologyTable.count(receivedEntry.sourceId) && topologyTable[receivedEntry.sourceId].count(receivedEntry.neighborId)) {
            const LinkStateEntry& existingEntry = topologyTable[receivedEntry.sourceId][receivedEntry.neighborId];
            if (receivedEntry.sequenceNumber > existingEntry.sequenceNumber) {
                topologyTable[receivedEntry.sourceId][receivedEntry.neighborId] = receivedEntry;
                log("Updated LS for (" + std::to_string(receivedEntry.sourceId) + ", " + std::to_string(receivedEntry.neighborId) + ") with newer sequence " + std::to_string(receivedEntry.sequenceNumber));
                topologyChanged = true;
            }
            foundExisting = true;
        }

        if (!foundExisting) {
            topologyTable[receivedEntry.sourceId][receivedEntry.neighborId] = receivedEntry;
            log("Added new LS for (" + std::to_string(receivedEntry.sourceId) + ", " + std::to_string(receivedEntry.neighborId) + ") sequence " + std::to_string(receivedEntry.sequenceNumber));
            topologyChanged = true;
        }
    }
    if (topologyChanged) {
        computeShortestPaths();
    }
}

void FSRNode::computeShortestPaths()
{
    log("Computing shortest paths (Dijkstra's algorithm).");
    distanceTable.clear();
    nextHopTable.clear();

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    int totalNodes = getParentModule()->par("numNodes");
    for (int i = 0; i < totalNodes; ++i) {
        distanceTable[i] = std::numeric_limits<int>::max();
    }

    distanceTable[myId] = 0;
    pq.push({0, myId});

    std::map<int, int> predecessor;

    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > distanceTable[u]) {
            continue;
        }

        std::set<int> knownNeighborsOfU;

        if (topologyTable.count(u)) {
            for (auto const& [v_neighbor, linkState] : topologyTable[u]) {
                knownNeighborsOfU.insert(v_neighbor);
            }
        }
        for (auto const& [srcNode, links] : topologyTable) {
            if (links.count(u)) {
                knownNeighborsOfU.insert(srcNode);
            }
        }

        for (int v : knownNeighborsOfU) {
            if (v == u) continue;

            int newDist = distanceTable[u] + 1;

            if (newDist < distanceTable[v]) {
                distanceTable[v] = newDist;
                predecessor[v] = u;
                pq.push({newDist, v});
            }
        }
    }

    for (int destId = 0; destId < totalNodes; ++destId) {
        if (destId == myId) continue;

        if (distanceTable[destId] == std::numeric_limits<int>::max()) {
            continue;
        }

        int current = destId;
        int nextHop = -1;

        while (predecessor.count(current) && predecessor[current] != myId) {
            current = predecessor[current];
        }

        if (predecessor.count(destId) && predecessor[destId] == myId) {
            nextHop = destId;
        } else if (predecessor.count(current) && predecessor[current] == myId) {
            nextHop = current;
        } else {
            // log("Warning: Could not determine next hop for " + std::to_string(destId) + " from predecessor chain.");
        }


        if (nextHop != -1 && distanceTable[destId] > 0) {
            RouteEntry entry;
            entry.destinationId = destId;
            entry.nextHopId = nextHop;
            entry.distance = distanceTable[destId];
            entry.sequenceNumber = currentSequenceNumber;
            nextHopTable[destId] = entry;
            log("Route to " + std::to_string(destId) + ": next hop " + std::to_string(nextHop) + ", distance " + std::to_string(distanceTable[destId]));
        }
    }
}

// Function to generate and send data packets
void FSRNode::generateDataTraffic() {
    // Choose a random destination that is not this node
    int totalNodes = getParentModule()->par("numNodes");
    int destinationId = -1;
    if (totalNodes <= 1) { // Cannot send traffic if only one node
        log("Cannot generate traffic: not enough nodes in the network.");
        return;
    }
    do {
        destinationId = intuniform(0, totalNodes - 1);
    } while (destinationId == myId); // Ensure destination is not self

    myTrafficSequenceNum++; // Increment sequence number for this node's data packets

    log("Generating DATA_PACKET (seq=" + std::to_string(myTrafficSequenceNum) + ") for destination " + std::to_string(destinationId));
    sendDataPacket(destinationId);
}


void FSRNode::updateNeighborList()
{
    log("Updating neighbor list.");
    std::set<int> newNeighbors;
    int totalNodes = getParentModule()->par("numNodes");

    for (int i = 0; i < totalNodes; ++i) {
        if (i == myId) continue; // Skip self

        // Get the other node module and its position
        FSRNode *otherNode = check_and_cast<FSRNode*>(getParentModule()->getSubmodule("node", i));
        double otherX = otherNode->getX();
        double otherY = otherNode->getY();

        // Calculate distance between this node and the other node
        double dist = calculateDistance(currentX, currentY, otherX, otherY);

        // If within transmission range, add to new neighbors
        if (dist <= transmissionRange) {
            newNeighbors.insert(i);
        }
    }

    // Check if the neighbor list has changed since the last update
    if (newNeighbors != neighbors) {
        log("Neighbor list changed. Old size: " + std::to_string(neighbors.size()) + ", New size: " + std::to_string(newNeighbors.size()));
        neighbors = newNeighbors; // Update the neighbors set
        // Trigger a local scope update because direct neighbors changed
        sendLocalScopeUpdate();
    } else {
        log("Neighbor list unchanged. Size: " + std::to_string(neighbors.size()));
    }
}

void FSRNode::sendDataPacket(int destination)
{
    if (destination == myId) {
        log("Cannot send data to self. Dropping packet.");
        return;
    }

    // Find the next hop to the destination
    auto it = nextHopTable.find(destination);
    if (it != nextHopTable.end()) {
        int nextHop = it->second.nextHopId;
        log("Sending DATA_PACKET (seq=" + std::to_string(myTrafficSequenceNum) + ") to " + std::to_string(destination) + " via next hop " + std::to_string(nextHop));

        FSRPacket *dataPkt = new FSRPacket("DATA_PACKET", FSRPacket::DATA_PACKET);
        dataPkt->sourceAddress = myId;
        dataPkt->destinationAddress = destination;
        dataPkt->sequenceNum = myTrafficSequenceNum; // Unique sequence number for this source's data packets
        dataPkt->setBitLength(dataPacketSize * 8); // Set packet size in bits
        dataPkt->encapsulate(new cPacket("AppDataPacket")); // Simulate application data content

        // Store information about the sent packet for end-to-end delay/PDR tracking
        std::string packetId = std::to_string(myId) + "_" + std::to_string(myTrafficSequenceNum);
        sentDataPacketsInfo[packetId] = {simTime(), destination}; // Store send time and destination

        cModule *targetModule = getParentModule()->getSubmodule("node", nextHop);
        if (targetModule) {
            cGate *targetGate = targetModule->gate("radio$i"); // Send to the next hop's radio gate
            sendDirect(dataPkt, targetGate);
            packetsSentData++; // Increment count of data packets originated by this node
            dataBytesSentTotal += dataPkt->getByteLength(); // Accumulate total bytes sent (source)
            emit(packetsSentDataAsSourceSignal, 1); // Emit 1 for each packet originated
            emit(dataBytesSentTotalSignal, dataPkt->getByteLength()); // Emit bytes for overhead calculation
        } else {
            log("Error: Target module " + std::to_string(nextHop) + " not found for data forwarding (dropping packet).");
            delete dataPkt;
            // Optionally, mark as dropped in sentDataPacketsInfo if you track drops globally
        }
    } else {
        log("No route to destination " + std::to_string(destination) + " found for data packet (dropping packet).");
        // If packet is dropped due to no route, you might want to record this for PDR analysis
        // For simplicity, we just log and drop for now.
    }
}

// *** Mobility Functions ***
void FSRNode::scheduleMobilityUpdate() {
    double delay;
    double remainingDistance = calculateDistance(currentX, currentY, destX, destY);
    if (remainingDistance > 0.01) { // If still far from destination, continue moving
        delay = remainingDistance / currentSpeed;
    } else { // Destination reached, pause and set new destination
        delay = pauseTime;
        setNewDestination();
    }
    if (delay <= 0) delay = 0.001; // Ensure delay is always positive
    scheduleAt(simTime() + delay, mobilityTimer);
}

void FSRNode::updatePosition() {
    double timeSinceLastMove = simTime().dbl() - lastMoveTime;
    if (timeSinceLastMove < 0) timeSinceLastMove = 0; // Should not be negative

    double dx = destX - currentX;
    double dy = destY - currentY;
    double distanceToDest = std::sqrt(dx*dx + dy*dy);

    if (distanceToDest < 0.01) { // If very close to destination, snap to it
        currentX = destX;
        currentY = destY;
        log("Reached destination (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");
        updateNeighborList(); // Check for neighbor changes upon reaching destination
        return;
    }

    // Calculate ratio of distance covered in timeSinceLastMove
    double ratio = currentSpeed * timeSinceLastMove / distanceToDest;
    if (ratio >= 1.0) { // If overshot or reached
        currentX = destX;
        currentY = destY;
    } else { // Move partially towards destination
        currentX += dx * ratio;
        currentY += dy * ratio;
    }
    lastMoveTime = simTime().dbl(); // Update last move time

    log("Moving to (" + std::to_string(currentX) + ", " + std::to_string(currentY) + ")");
    refreshDisplay(); // Update graphical display
    updateNeighborList(); // Check for neighbor changes during movement
}

void FSRNode::setNewDestination() {
    destX = uniform(0, playgroundX); // Random X destination
    destY = uniform(0, playgroundY); // Random Y destination
    currentSpeed = uniform(minSpeed, maxSpeed); // Random speed within range
    lastMoveTime = simTime().dbl(); // Reset last move time for new segment
    log("Set new destination: (" + std::to_string(destX) + ", " + std::to_string(destY) + ") with speed " + std::to_string(currentSpeed) + " m/s");
}

double FSRNode::calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
