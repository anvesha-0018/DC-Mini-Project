#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/udp-echo-helper.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <filesystem>

using namespace ns3;
namespace fs = std::filesystem;

NS_LOG_COMPONENT_DEFINE("HcMacMobilityCsma");

struct WaypointData {
  double time;
  double x;
  double y;
};

std::vector<WaypointData> ReadTraceFile(const std::string& filename) {
  std::vector<WaypointData> waypoints;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    NS_FATAL_ERROR("Could not open trace file: " << filename);
    return waypoints;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    WaypointData wp;
    if (iss >> wp.time >> wp.x >> wp.y) {
      waypoints.push_back(wp);
    } else {
      NS_LOG_WARN("Malformed line in trace file: " << line);
    }
  }
  return waypoints;
}

int main(int argc, char *argv[]) {
  // Enable logging
  LogComponentEnable("HcMacMobilityCsma", LOG_LEVEL_INFO);
  LogComponentEnable("WaypointMobilityModel", LOG_LEVEL_DEBUG);

  // Add command line parameters
  double simTime = 20.0;
  uint32_t packetSize = 1024;
  double interval = 0.1;
  std::string traceDir = "scratch/vehicle_traces";
  
  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue("packetSize", "Size of UDP packets", packetSize);
  cmd.AddValue("interval", "Packet interval time", interval);
  cmd.AddValue("traceDir", "Directory containing vehicle traces", traceDir);
  cmd.Parse(argc, argv);

  // Trace file directory handling with error checking
  if (!fs::exists(traceDir)) {
    NS_FATAL_ERROR("Trace directory " << traceDir << " does not exist");
    return -1;
  }

  std::vector<std::string> files;
  try {
    for (const auto& entry : fs::directory_iterator(traceDir)) {
      if (entry.path().extension() == ".txt") {
        files.push_back(entry.path().string());
        NS_LOG_INFO("Found trace file: " << entry.path().string());
      }
    }
  } catch (const fs::filesystem_error& err) {
    NS_FATAL_ERROR("Filesystem error: " << err.what());
    return -1;
  }

  if (files.empty()) {
    NS_FATAL_ERROR("No trace files found in directory " << traceDir);
    return -1;
  }

  uint32_t numVehicles = files.size();
  NS_LOG_INFO("Creating simulation for " << numVehicles << " vehicles");

  NodeContainer nodes;
  nodes.Create(numVehicles);

  // Mobility setup
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::WaypointMobilityModel");
  mobility.Install(nodes);

  for (uint32_t i = 0; i < numVehicles; ++i) {
    Ptr<Node> node = nodes.Get(i);
    Ptr<WaypointMobilityModel> model = node->GetObject<WaypointMobilityModel>();
    std::vector<WaypointData> waypoints = ReadTraceFile(files[i]);
    
    if (waypoints.empty()) {
      NS_LOG_WARN("No valid waypoints found for vehicle " << i);
      continue;
    }

    for (const auto& wp : waypoints) {
      Vector pos(wp.x, wp.y, 1.5);  // 1.5m height for vehicle antenna
      model->AddWaypoint(Waypoint(Seconds(wp.time), pos));
    }
    NS_LOG_DEBUG("Added " << waypoints.size() << " waypoints for vehicle " << i);
  }

  // ==============================================
  // Enhanced CSMA Network Setup
  // ==============================================
  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
  csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));
  // Enable flow control and set more realistic parameters
  csma.SetQueue("ns3::DropTailQueue", "MaxSize", StringValue("50p"));

  NetDeviceContainer devices = csma.Install(nodes);
  // ==============================================

  InternetStackHelper internet;
  internet.Install(nodes);

  // Fix for IP address overflow with large number of vehicles
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.0.0");  // Using /16 subnet for >254 vehicles
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  // Install applications with configurable parameters
  UdpEchoServerHelper echoServer(9);
  ApplicationContainer serverApps;
  for (uint32_t i = 1; i < numVehicles; i++) {
    serverApps.Add(echoServer.Install(nodes.Get(i)));
  }
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(simTime));

  ApplicationContainer clientApps;
  for (uint32_t i = 0; i < numVehicles - 1; i++) {
    UdpEchoClientHelper echoClient(interfaces.GetAddress(i + 1), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(interval)));
    echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
    clientApps.Add(echoClient.Install(nodes.Get(i)));
  }
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(simTime));

  // Enhanced flow monitoring
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // Enable PCAP tracing for first 10 nodes only (to avoid too many files)
  for (uint32_t i = 0; i < std::min(numVehicles, 10u); i++) {
    csma.EnablePcap("hc_mac_csma", devices.Get(i), true);
  }

  NS_LOG_INFO("Starting simulation for " << simTime << " seconds");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  // Enhanced statistics collection
  monitor->CheckForLostPackets();
  monitor->SerializeToXmlFile("hc_mac_csma_results.flowmon", true, true);

  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  double totalThroughput = 0.0;
  double totalDelay = 0.0;
  uint32_t totalPackets = 0;
  uint32_t totalLostPackets = 0;
  uint32_t flowCount = 0;

  // Detailed statistics output
  std::ofstream statsFile("hc_mac_csma_stats.csv");
  statsFile << "FlowID,Throughput(kbps),AvgDelay(ms),PacketDeliveryRatio(%),LostPackets\n";

  for (auto const& flow : stats) {
    if (flow.second.rxPackets == 0) continue;
    
    flowCount++;
    auto rxBytes = flow.second.rxBytes;
    auto delay = flow.second.delaySum.GetSeconds();
    auto txPackets = flow.second.txPackets;
    auto rxPackets = flow.second.rxPackets;

    double throughput = (rxBytes * 8.0) /
        (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024.0;
    double avgDelay = (delay / rxPackets) * 1000; // Convert to milliseconds
    double pdr = (rxPackets * 100.0 / txPackets);

    // Write to CSV file
    statsFile << flow.first << "," << throughput << "," << avgDelay << "," 
              << pdr << "," << (txPackets - rxPackets) << "\n";

    // Console output
    std::cout << "\nFlow " << flow.first << " Statistics:\n"
              << "  Throughput: " << throughput << " kbps\n"
              << "  Avg Delay: " << avgDelay << " ms\n"
              << "  Packet Delivery Ratio: " << pdr << "%\n"
              << "  Lost Packets: " << (txPackets - rxPackets) << "\n";

    totalThroughput += throughput;
    totalDelay += avgDelay;
    totalPackets += txPackets;
    totalLostPackets += (txPackets - rxPackets);
  }
  statsFile.close();

  if (flowCount > 0) {
    std::cout << "\nGlobal Statistics:\n"
              << "  Avg Throughput: " << totalThroughput/flowCount << " kbps\n"
              << "  Avg End-to-End Delay: " << totalDelay/flowCount << " ms\n"
              << "  Total Packet Loss: " << totalLostPackets << " (" 
              << (totalLostPackets * 100.0 / totalPackets) << "%)\n";
  } else {
    std::cout << "\nNo valid flow statistics to report\n";
  }

  Simulator::Destroy();
  NS_LOG_INFO("Simulation completed");
  return 0;
}
