#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace ns3;
using namespace ns3::energy;

// ============================================================
// ENERGY-EFFICIENT COMMUNICATION IN WIRELESS SENSOR NETWORKS
// Using ns-3.48
//
// Scenario 1: Baseline WSN
// Scenario 2: Energy-efficient WSN using duty-cycled
//             Wi-Fi sleep/wake scheduling
// ============================================================

// ---------------------- PROJECT PARAMETERS ------------------

static const uint32_t NUM_SENSORS = 30;

static const double SIMULATION_TIME = 60.0;

static const double SENSOR_INITIAL_ENERGY = 10.0;   // Joules
static const double SINK_INITIAL_ENERGY = 100.0;    // Joules

static const double PACKET_INTERVAL = 1.0;           // seconds
static const uint32_t PACKET_SIZE = 256;             // bytes

static const double APPLICATION_START = 2.0;
static const double APPLICATION_STOP = 58.0;

// Energy-efficient duty cycle:
// sensor sleeps after transmission and wakes before next packet.
static const double SLEEP_START_OFFSET = 0.15;
static const double WAKE_OFFSET = 0.85;

// ------------------------------------------------------------


// ============================================================
// NETWORK LIFETIME TRACKER
// ============================================================

class NetworkLifetimeTracker
{
public:

    NetworkLifetimeTracker(const EnergySourceContainer& sources,
                           uint32_t sensorCount,
                           double simulationTime)
        : m_sources(sources),
          m_sensorCount(sensorCount),
          m_simulationTime(simulationTime),
          m_lifetimeRecorded(false),
          m_lifetime(simulationTime)
    {
    }

    void Check()
    {
        if (!m_lifetimeRecorded)
        {
            for (uint32_t i = 0; i < m_sensorCount; ++i)
            {
                Ptr<energy::BasicEnergySource> source =
                    DynamicCast<energy::BasicEnergySource>(
                        m_sources.Get(i));

                if (source != nullptr &&
                    source->GetRemainingEnergy() <= 0.0)
                {
                    m_lifetimeRecorded = true;
                    m_lifetime = Simulator::Now().GetSeconds();

                    std::cout
                        << "\n*** First sensor depleted at "
                        << std::fixed
                        << std::setprecision(3)
                        << m_lifetime
                        << " seconds ***\n";

                    return;
                }
            }
        }

        if (!m_lifetimeRecorded &&
            Simulator::Now().GetSeconds() + 0.1 < m_simulationTime)
        {
            Simulator::Schedule(
                MilliSeconds(100),
                &NetworkLifetimeTracker::Check,
                this);
        }
    }

    double GetLifetime() const
    {
        return m_lifetime;
    }

    bool WasDepleted() const
    {
        return m_lifetimeRecorded;
    }

private:

    EnergySourceContainer m_sources;

    uint32_t m_sensorCount;

    double m_simulationTime;

    bool m_lifetimeRecorded;

    double m_lifetime;
};


// ============================================================
// PUT A SENSOR RADIO INTO SLEEP MODE
// ============================================================

void
PutNodeToSleep(Ptr<WifiNetDevice> wifiDevice)
{
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();

    if (!phy->IsStateSleep() && !phy->IsStateOff())
    {
        phy->SetSleepMode();
    }
}


// ============================================================
// WAKE A SENSOR RADIO
// ============================================================

void
WakeNode(Ptr<WifiNetDevice> wifiDevice)
{
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();

    if (phy->IsStateSleep())
    {
        phy->ResumeFromSleep();
    }
}


// ============================================================
// CONFIGURE DUTY-CYCLED OPERATION
// ============================================================

void
ConfigureDutyCycling(NetDeviceContainer devices)
{
    /*
     * Each sensor sends one packet every second.
     *
     * Example:
     *
     * 2.00 s  -> packet
     * 2.15 s  -> sleep
     * 2.85 s  -> wake
     * 3.00 s  -> packet
     * 3.15 s  -> sleep
     * 3.85 s  -> wake
     *
     * This keeps the same traffic pattern while reducing
     * radio idle time.
     */

    for (uint32_t node = 0; node < NUM_SENSORS; ++node)
    {
        Ptr<WifiNetDevice> wifiDevice =
            DynamicCast<WifiNetDevice>(
                devices.Get(node));

        for (double t = APPLICATION_START;
             t < APPLICATION_STOP;
             t += PACKET_INTERVAL)
        {
            double sleepTime = t + SLEEP_START_OFFSET;
            double wakeTime = t + WAKE_OFFSET;

            if (sleepTime < APPLICATION_STOP)
            {
                Simulator::Schedule(
                    Seconds(sleepTime),
                    &PutNodeToSleep,
                    wifiDevice);
            }

            if (wakeTime < APPLICATION_STOP)
            {
                Simulator::Schedule(
                    Seconds(wakeTime),
                    &WakeNode,
                    wifiDevice);
            }
        }
    }
}


// ============================================================
// PRINT RESULTS
// ============================================================

void
PrintAndSaveResults(
    const std::string& mode,
    const std::string& csvFile,
    FlowMonitorHelper& flowHelper,
    Ptr<FlowMonitor> monitor,
    EnergySourceContainer sources,
    DeviceEnergyModelContainer deviceModels,
    NetworkLifetimeTracker& lifetimeTracker)
{
    monitor->CheckForLostPackets();

    std::map<FlowId, FlowMonitor::FlowStats> stats =
        monitor->GetFlowStats();

    uint64_t totalTxPackets = 0;
    uint64_t totalRxPackets = 0;

    uint64_t totalTxBytes = 0;
    uint64_t totalRxBytes = 0;

    uint64_t totalLostPackets = 0;

    double totalDelay = 0.0;

    // --------------------------------------------------------
    // Aggregate FlowMonitor results
    // --------------------------------------------------------

    for (const auto& entry : stats)
    {
        const FlowMonitor::FlowStats& flow = entry.second;

        totalTxPackets += flow.txPackets;
        totalRxPackets += flow.rxPackets;

        totalTxBytes += flow.txBytes;
        totalRxBytes += flow.rxBytes;

        totalLostPackets += flow.lostPackets;

        totalDelay += flow.delaySum.GetSeconds();
    }

    // --------------------------------------------------------
    // Network performance calculations
    // --------------------------------------------------------

    double pdr = 0.0;

    if (totalTxPackets > 0)
    {
        pdr =
            (static_cast<double>(totalRxPackets) /
             static_cast<double>(totalTxPackets)) * 100.0;
    }

    double averageDelay = 0.0;

    if (totalRxPackets > 0)
    {
        averageDelay =
            totalDelay /
            static_cast<double>(totalRxPackets);
    }

    double throughputKbps =
        (static_cast<double>(totalRxBytes) * 8.0) /
        (SIMULATION_TIME * 1000.0);

    // --------------------------------------------------------
    // Energy calculations
    // --------------------------------------------------------

    double totalInitialEnergy = 0.0;
    double totalRemainingEnergy = 0.0;

    double sensorInitialEnergy = 0.0;
    double sensorRemainingEnergy = 0.0;

    for (uint32_t i = 0; i < sources.GetN(); ++i)
    {
        Ptr<energy::BasicEnergySource> source =
            DynamicCast<energy::BasicEnergySource>(
                sources.Get(i));

        if (source != nullptr)
        {
            double initial = source->GetInitialEnergy();
            double remaining = source->GetRemainingEnergy();

            totalInitialEnergy += initial;
            totalRemainingEnergy += remaining;

            if (i < NUM_SENSORS)
            {
                sensorInitialEnergy += initial;
                sensorRemainingEnergy += remaining;
            }
        }
    }

    double totalEnergyConsumed =
        totalInitialEnergy - totalRemainingEnergy;
	// Save baseline energy consumption for comparison
	if (mode == "baseline")
	{
    		std::ofstream energyFile("baseline-energy.txt");
    		energyFile << std::setprecision(10) << totalEnergyConsumed;
    		energyFile.close();
	}
	// Compare efficient mode against baseline
	if (mode == "efficient")
	{
    		std::ifstream energyFile("baseline-energy.txt");

    		if (energyFile.is_open())
    		{
        		double baselineEnergyConsumed = 0.0;
        		energyFile >> baselineEnergyConsumed;
        		energyFile.close();

        		double energySaved =
            			baselineEnergyConsumed - totalEnergyConsumed;

        		double energySavingPercent =
            			(energySaved / baselineEnergyConsumed) * 100.0;

        		std::cout << "\n";
        		std::cout << "==================================================\n";
        		std::cout << "ENERGY EFFICIENCY COMPARISON\n";
       			std::cout << "==================================================\n";
       		 	std::cout << std::fixed << std::setprecision(4);

        		std::cout << "Baseline energy consumed"
                  		<< " : " << baselineEnergyConsumed << " J\n";

        		std::cout << "Efficient energy consumed"
                  		<< " : " << totalEnergyConsumed << " J\n";

        		std::cout << "Energy saved"
                  		<< " : " << energySaved << " J\n";

        		std::cout << "Energy saving"
                  		<< " : " << energySavingPercent << " %\n";

        		std::cout << "==================================================\n";
    		}
    		else
    		{
        		std::cout << "\nCould not find baseline-energy.txt.\n";
        		std::cout << "Run the baseline simulation first.\n";
    		}
}

    double sensorEnergyConsumed =
        sensorInitialEnergy - sensorRemainingEnergy;

    // --------------------------------------------------------
    // Print results
    // --------------------------------------------------------

    std::cout << "\n";
    std::cout << "====================================================\n";
    std::cout << " WSN SIMULATION RESULTS\n";
    std::cout << "====================================================\n";

    std::cout << "Mode                    : "
              << mode << "\n";

    std::cout << "Number of sensors       : "
              << NUM_SENSORS << "\n";

    std::cout << "Simulation time         : "
              << SIMULATION_TIME << " s\n";

    std::cout << "Initial network energy  : "
              << totalInitialEnergy << " J\n";

    std::cout << "Remaining network energy: "
              << totalRemainingEnergy << " J\n";

    std::cout << "Total energy consumed   : "
              << totalEnergyConsumed << " J\n";

    std::cout << "Sensor energy consumed  : "
              << sensorEnergyConsumed << " J\n";

    std::cout << "Packets sent            : "
              << totalTxPackets << "\n";

    std::cout << "Packets received        : "
              << totalRxPackets << "\n";

    std::cout << "Packets lost            : "
              << totalLostPackets << "\n";

    std::cout << "Packet Delivery Ratio   : "
              << pdr << " %\n";

    std::cout << "Throughput              : "
              << throughputKbps << " kbps\n";

    std::cout << "Average end-to-end delay: "
              << averageDelay * 1000.0 << " ms\n";

    if (lifetimeTracker.WasDepleted())
    {
        std::cout << "Network lifetime        : "
                  << lifetimeTracker.GetLifetime()
                  << " s\n";
    }
    else
    {
        std::cout << "Network lifetime        : >= "
                  << SIMULATION_TIME
                  << " s\n";
    }

    std::cout << "====================================================\n";

    // --------------------------------------------------------
    // Save CSV
    // --------------------------------------------------------

    std::ofstream output(csvFile);

    output << "Mode,InitialEnergyJ,RemainingEnergyJ,"
              "TotalEnergyConsumedJ,SensorEnergyConsumedJ,"
              "PacketsSent,PacketsReceived,PacketsLost,"
              "PDRPercent,ThroughputKbps,AverageDelayMs,"
              "NetworkLifetimeSeconds\n";

    output << mode << ","
           << totalInitialEnergy << ","
           << totalRemainingEnergy << ","
           << totalEnergyConsumed << ","
           << sensorEnergyConsumed << ","
           << totalTxPackets << ","
           << totalRxPackets << ","
           << totalLostPackets << ","
           << pdr << ","
           << throughputKbps << ","
           << averageDelay * 1000.0 << ",";

    if (lifetimeTracker.WasDepleted())
    {
        output << lifetimeTracker.GetLifetime();
    }
    else
    {
        output << SIMULATION_TIME;
    }

    output << "\n";

    output.close();

    // --------------------------------------------------------
    // Save FlowMonitor XML
    // --------------------------------------------------------

    flowHelper.SerializeToXmlFile(
        mode + "-flowmon.xml",
        true,
        true);

    std::cout
        << "\nResults saved to: "
        << csvFile
        << "\n";

    std::cout
        << "FlowMonitor saved to: "
        << mode
        << "-flowmon.xml\n";
}


// ============================================================
// MAIN
// ============================================================

int
main(int argc, char* argv[])
{
    // --------------------------------------------------------
    // Command-line mode
    // --------------------------------------------------------

    std::string mode = "baseline";

    CommandLine cmd;

    cmd.AddValue(
        "mode",
        "Simulation mode: baseline or efficient",
        mode);

    cmd.Parse(argc, argv);

    if (mode != "baseline" &&
        mode != "efficient")
    {
        std::cerr
            << "Invalid mode. Use baseline or efficient.\n";

        return 1;
    }

    bool energyEfficient =
        (mode == "efficient");

    // --------------------------------------------------------
    // Create nodes
    // --------------------------------------------------------

    NodeContainer sensorNodes;

    sensorNodes.Create(NUM_SENSORS);

    NodeContainer sinkNode;

    sinkNode.Create(1);

    NodeContainer allNodes;

    allNodes.Add(sensorNodes);
    allNodes.Add(sinkNode);

    // --------------------------------------------------------
    // Wi-Fi configuration
    // --------------------------------------------------------

    WifiHelper wifi;

    wifi.SetStandard(WIFI_STANDARD_80211b);

    WifiMacHelper wifiMac;

    wifiMac.SetType(
        "ns3::AdhocWifiMac");

    YansWifiChannelHelper channel =
        YansWifiChannelHelper::Default();

    YansWifiPhyHelper phy;

    phy.SetChannel(
        channel.Create());

    NetDeviceContainer devices =
        wifi.Install(
            phy,
            wifiMac,
            allNodes);

    // --------------------------------------------------------
    // Mobility / positions
    // --------------------------------------------------------

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> positionAllocator =
        CreateObject<ListPositionAllocator>();

    // Sensor positions

    positionAllocator->Add(
        Vector(10.0, 10.0, 0.0));

    positionAllocator->Add(
        Vector(20.0, 10.0, 0.0));

    positionAllocator->Add(
        Vector(30.0, 10.0, 0.0));

    positionAllocator->Add(
        Vector(10.0, 20.0, 0.0));

    positionAllocator->Add(
        Vector(30.0, 20.0, 0.0));

    positionAllocator->Add(
        Vector(10.0, 30.0, 0.0));

    positionAllocator->Add(
        Vector(20.0, 30.0, 0.0));

    positionAllocator->Add(
        Vector(30.0, 30.0, 0.0));

    // Sink at the center

    positionAllocator->Add(
        Vector(20.0, 20.0, 0.0));

    mobility.SetPositionAllocator(
        positionAllocator);

    mobility.SetMobilityModel(
        "ns3::ConstantPositionMobilityModel");

    mobility.Install(allNodes);

    // --------------------------------------------------------
    // Energy model
    // --------------------------------------------------------

    BasicEnergySourceHelper energySourceHelper;

    energySourceHelper.Set(
        "BasicEnergySourceInitialEnergyJ",
        DoubleValue(SENSOR_INITIAL_ENERGY));

    energySourceHelper.Set(
        "BasicEnergySupplyVoltageV",
        DoubleValue(3.0));

    energySourceHelper.Set(
        "PeriodicEnergyUpdateInterval",
        TimeValue(MilliSeconds(100)));

    EnergySourceContainer sources =
        energySourceHelper.Install(allNodes);

    // Give the sink a larger energy reserve.

    Ptr<energy::BasicEnergySource> sinkSource =
        DynamicCast<energy::BasicEnergySource>(
            sources.Get(NUM_SENSORS));

    sinkSource->SetInitialEnergy(
        SINK_INITIAL_ENERGY);

    // --------------------------------------------------------
    // Wi-Fi radio energy model
    // --------------------------------------------------------

    WifiRadioEnergyModelHelper radioEnergyHelper;

    /*
     * These values are intentionally explicit so the
     * experiment uses the same radio-energy parameters
     * in both scenarios.
     */

    radioEnergyHelper.Set(
        "IdleCurrentA",
        DoubleValue(0.273));

    radioEnergyHelper.Set(
        "CcaBusyCurrentA",
        DoubleValue(0.273));

    radioEnergyHelper.Set(
        "TxCurrentA",
        DoubleValue(0.380));

    radioEnergyHelper.Set(
        "RxCurrentA",
        DoubleValue(0.313));

    radioEnergyHelper.Set(
        "SleepCurrentA",
        DoubleValue(0.033));

    DeviceEnergyModelContainer deviceModels =
        radioEnergyHelper.Install(
            devices,
            sources);

    // --------------------------------------------------------
    // Internet stack
    // --------------------------------------------------------

    InternetStackHelper internet;

    internet.Install(allNodes);

    // --------------------------------------------------------
    // IP addressing
    // --------------------------------------------------------

    Ipv4AddressHelper address;

    address.SetBase(
        "10.1.1.0",
        "255.255.255.0");

    Ipv4InterfaceContainer interfaces =
        address.Assign(devices);

    // --------------------------------------------------------
    // UDP server at sink
    // --------------------------------------------------------

    uint16_t port = 9000;

    UdpServerHelper server(port);

    ApplicationContainer serverApp =
        server.Install(
            sinkNode.Get(0));

    serverApp.Start(
        Seconds(0.0));

    serverApp.Stop(
        Seconds(SIMULATION_TIME));

    // --------------------------------------------------------
    // UDP clients on sensor nodes
    // --------------------------------------------------------

    for (uint32_t i = 0;
         i < NUM_SENSORS;
         ++i)
    {
        UdpClientHelper client(
            interfaces.GetAddress(NUM_SENSORS),
            port);

        client.SetAttribute(
            "MaxPackets",
            UintegerValue(100000));

        client.SetAttribute(
            "Interval",
            TimeValue(
                Seconds(PACKET_INTERVAL)));

        client.SetAttribute(
            "PacketSize",
            UintegerValue(PACKET_SIZE));

        ApplicationContainer clientApp =
            client.Install(
                sensorNodes.Get(i));

        clientApp.Start(
            Seconds(APPLICATION_START));

        clientApp.Stop(
            Seconds(APPLICATION_STOP));
    }

    // --------------------------------------------------------
    // Energy-efficient mechanism
    // --------------------------------------------------------

    if (energyEfficient)
    {
        ConfigureDutyCycling(devices);
    }

    // --------------------------------------------------------
    // FlowMonitor
    // --------------------------------------------------------

    FlowMonitorHelper flowHelper;

    Ptr<FlowMonitor> monitor =
        flowHelper.InstallAll();

    // --------------------------------------------------------
    // Network lifetime monitoring
    // --------------------------------------------------------

    NetworkLifetimeTracker lifetimeTracker(
        sources,
        NUM_SENSORS,
        SIMULATION_TIME);

    Simulator::Schedule(
        MilliSeconds(100),
        &NetworkLifetimeTracker::Check,
        &lifetimeTracker);

    // --------------------------------------------------------
    // Optional packet capture
    // --------------------------------------------------------

    phy.EnablePcap(
        mode + "-sensor0",
        devices.Get(0));

    phy.EnablePcap(
        mode + "-sink",
        devices.Get(NUM_SENSORS));

    // --------------------------------------------------------
    // Run simulation
    // --------------------------------------------------------

    std::cout
        << "\nStarting "
        << mode
        << " WSN simulation...\n";

    Simulator::Stop(
        Seconds(SIMULATION_TIME));

    Simulator::Run();

    // --------------------------------------------------------
    // Results
    // --------------------------------------------------------

    PrintAndSaveResults(
        mode,
        mode + "-results.csv",
        flowHelper,
        monitor,
        sources,
        deviceModels,
        lifetimeTracker);

    Simulator::Destroy();

    return 0;
}
