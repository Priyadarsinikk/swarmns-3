/*
 * SwarmNet-IND -- Preliminary study  (version 11: C2 over two paths; N1 v3 window + C2 expiry is the final N1)
 *
 * What this simulates:
 *   - nUav drones flying in 3D (Gauss-Markov mobility, 30-120 km/h, 50-150 m altitude)
 *   - one ground station at the centre of the area
 *   - an ad-hoc Wi-Fi mesh (802.11a, CSMA-CA medium access) -- "today's approach"
 *   - multi-hop routing with OLSR (or AODV, selectable)
 *   - nCam "camera" drones streaming constant-rate UDP traffic to the ground
 *
 * What it measures:
 *   - delivery, mean / 95th / 99th-percentile / maximum delay   (FlowMonitor)
 *   - NEW: WHY packets are lost:
 *       * no route at the source drone
 *       * dropped at the IP layer along the path (no route, TTL, queue, route error)
 *       * Wi-Fi gave up after all retries, split into:
 *           - receiver still within radio range  -> collision / interference  (N1's target)
 *           - receiver beyond radio range        -> link broken by movement  (N2's target)
 *       * Wi-Fi queue full / packet waited too long
 *   - NEW option --arpPrefill=1: fills every neighbour table at the start, to test
 *     how much of the 2-3 s worst-case delay comes from neighbour discovery (ARP)
 *
 * Run from the ns-3-dev folder:
 *   ./ns3 run "swarm-baseline --nUav=20 --run=1"
 *   ./ns3 run "swarm-baseline --nUav=20 --run=1 --arpPrefill=1"
 *   ./ns3 run "swarm-baseline --nUav=20 --run=1 --arpPrefill=1 --mac=N1"            (N1, 3 bands)
 *   ./ns3 run "swarm-baseline --nUav=20 --run=1 --arpPrefill=1 --mac=N1 --cols=2"   (3 bands x 2 columns)
 *   ./ns3 run "swarm-baseline --nUav=20 --run=1 --arpPrefill=1 --routing=N2"        (N2)
 */

#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SwarmBaseline");


// =====================================================================================
//  N1 PROTOTYPE: altitude-banded TDMA gate
//
//  Every superframe is split into slots. Each drone computes its GROUP from its current
//  position:  group = altitude band (B bands between minAlt and maxAlt)
//                     x horizontal column (C columns across the area).
//  The ground station has its own slot. A drone hands a packet to its Wi-Fi radio only
//  (a) during its group's slot, (b) when the radio has nothing else waiting, and
//  (c) when enough of the slot remains to finish sending it. Outside its slot it holds
//  packets here. Inside a slot, members of the same group share it with normal CSMA.
//
//  It is implemented as a "queue discipline": the layer that decides when packets go
//  from IP down to the radio. This lets us prototype TDMA on top of the existing Wi-Fi
//  radio without writing a new radio model.
// =====================================================================================
namespace ns3
{

class TdmaGateQueueDisc : public QueueDisc
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::TdmaGateQueueDisc")
                .SetParent<QueueDisc>()
                .SetGroupName("TrafficControl")
                .AddConstructor<TdmaGateQueueDisc>()
                .AddAttribute("MaxSize",
                              "Maximum number of packets held while waiting for the slot",
                              QueueSizeValue(QueueSize("1000p")),
                              MakeQueueSizeAccessor(&QueueDisc::SetMaxSize, &QueueDisc::GetMaxSize),
                              MakeQueueSizeChecker());
        return tid;
    }

    TdmaGateQueueDisc()
        : QueueDisc(QueueDiscSizePolicy::SINGLE_INTERNAL_QUEUE)
    {
    }

    // Called once after installation
    void Configure(Ptr<Node> node, Ptr<WifiMacQueue> macQueue, bool isGround, uint32_t bands,
                   uint32_t cols, double minAlt, double maxAlt, double areaSize, Time slot,
                   Time txTime, Time jitter)
    {
        m_jitter = jitter;
        m_rng = CreateObject<UniformRandomVariable>();
        m_node = node;
        m_macQueue = macQueue;
        m_isGround = isGround;
        m_bands = bands;
        m_cols = cols;
        m_minAlt = minAlt;
        m_maxAlt = maxAlt;
        m_areaSize = areaSize;
        m_slot = slot;
        m_txTime = txTime;
    }

    uint32_t NumSlots() const
    {
        return m_bands * m_cols + 1; // +1 for the ground station
    }

    Time Superframe() const
    {
        return m_slot * NumSlots();
    }

  private:
    // Which group (= which slot) is this node in right now?
    uint32_t MyGroup() const
    {
        if (m_isGround)
        {
            return m_bands * m_cols; // last slot
        }
        Vector p = m_node->GetObject<MobilityModel>()->GetPosition();
        double bandH = (m_maxAlt - m_minAlt) / m_bands;
        int band = static_cast<int>(std::floor((p.z - m_minAlt) / bandH));
        band = std::max(0, std::min(static_cast<int>(m_bands) - 1, band));
        double colW = m_areaSize / m_cols;
        int col = static_cast<int>(std::floor(p.x / colW));
        col = std::max(0, std::min(static_cast<int>(m_cols) - 1, col));
        return static_cast<uint32_t>(band) * m_cols + static_cast<uint32_t>(col);
    }

    void WakeLater(Time delay)
    {
        if (!m_wake.IsPending())
        {
            m_wake = Simulator::Schedule(delay, &TdmaGateQueueDisc::Wake, this);
        }
    }

    void Wake()
    {
        Run(); // ask the queue discipline to try sending again
    }

    bool DoEnqueue(Ptr<QueueDiscItem> item) override
    {
        if (GetCurrentSize() + item > GetMaxSize())
        {
            DropBeforeEnqueue(item, "Queue full");
            return false;
        }
        return GetInternalQueue(0)->Enqueue(item);
    }

    Ptr<QueueDiscItem> DoDequeue() override
    {
        if (GetInternalQueue(0)->GetNPackets() == 0)
        {
            return nullptr;
        }
        int64_t now = Simulator::Now().GetNanoSeconds();
        int64_t sf = Superframe().GetNanoSeconds();
        int64_t slot = m_slot.GetNanoSeconds();
        int64_t offset = now % sf;
        uint32_t g = MyGroup();
        int64_t myStart = static_cast<int64_t>(g) * slot;
        int64_t myEnd = myStart + slot;

        if (offset >= myStart && offset < myEnd)
        {
            int64_t remaining = myEnd - offset;
            if (remaining < m_txTime.GetNanoSeconds())
            {
                // Too little of the slot left to finish a packet: wait for the next superframe
                WakeLater(NanoSeconds(sf - offset + myStart));
                return nullptr;
            }
            if (m_macQueue && m_macQueue->GetNPackets() > 0)
            {
                // Radio still busy with an earlier packet: check again shortly
                WakeLater(MicroSeconds(100));
                return nullptr;
            }
            Ptr<QueueDiscItem> item = GetInternalQueue(0)->Dequeue();
            if (GetInternalQueue(0)->GetNPackets() > 0)
            {
                WakeLater(MicroSeconds(100)); // more waiting: try again within this slot
            }
            return item;
        }

        // Not my slot: sleep until my slot starts (group is recomputed then), plus a small
        // random delay so that all members of the group do not try to transmit at the same instant
        int64_t untilStart = (offset < myStart) ? (myStart - offset) : (sf - offset + myStart);
        int64_t jit = static_cast<int64_t>(m_rng->GetValue(0.0, m_jitter.GetNanoSeconds()));
        WakeLater(NanoSeconds(untilStart + jit));
        return nullptr;
    }

    bool CheckConfig() override
    {
        if (GetNQueueDiscClasses() > 0 || GetNPacketFilters() > 0)
        {
            return false;
        }
        if (GetNInternalQueues() == 0)
        {
            AddInternalQueue(CreateObjectWithAttributes<DropTailQueue<QueueDiscItem>>(
                "MaxSize", QueueSizeValue(GetMaxSize())));
        }
        return GetNInternalQueues() == 1;
    }

    void InitializeParams() override
    {
    }

    void DoDispose() override
    {
        m_wake.Cancel();
        m_node = nullptr;
        m_macQueue = nullptr;
        QueueDisc::DoDispose();
    }

    Ptr<Node> m_node;
    Ptr<WifiMacQueue> m_macQueue;
    bool m_isGround{false};
    uint32_t m_bands{3};
    uint32_t m_cols{1};
    double m_minAlt{50};
    double m_maxAlt{150};
    double m_areaSize{1000};
    Time m_slot{MilliSeconds(3)};
    Time m_txTime{MicroSeconds(1000)};
    Time m_jitter{MicroSeconds(300)};
    Ptr<UniformRandomVariable> m_rng;
    EventId m_wake;
};

NS_OBJECT_ENSURE_REGISTERED(TdmaGateQueueDisc);

} // namespace ns3

// ---------------- Diagnostic counters ----------------
static uint64_t g_appSent = 0;           // packets the camera apps handed to the socket
static uint64_t g_macRetryNear = 0;      // Wi-Fi gave up, receiver within range  -> collision/interference
static uint64_t g_macRetryFar = 0;       // Wi-Fi gave up, receiver out of range  -> link broken by movement
static uint64_t g_dropBin[4] = {0, 0, 0, 0}; // Wi-Fi gave up, by receiver distance: <300, 300-500, 500-650, >650 m
static uint64_t g_macQueueFull = 0;      // Wi-Fi queue full, packet refused
static uint64_t g_macExpired = 0;        // packet waited in the Wi-Fi queue too long
static uint64_t g_arpQueueDrop = 0;      // ARP waiting queue full (default holds only 3 packets per neighbour)
static uint64_t g_arpTimeoutDrop = 0;    // ARP gave up: neighbour never answered
static double g_linkRange = 365.0;       // metres; real range: -82 dBm detection limit at 16 dBm, 5.18 GHz (Friis)
static std::map<Mac48Address, Ptr<Node>> g_macToNode;
// C2-only loss counters (C2 frames are 130-499 bytes on air; video is >= 500, beacons smaller)
static uint64_t g_c2RetryNear = 0, g_c2RetryFar = 0, g_c2MacExpired = 0, g_c2MacFull = 0;
static uint64_t g_c2Bin[4] = {0, 0, 0, 0};
static uint64_t g_c2SrcFail = 0; // C2 the source could not send (no route at all)

static void
AppTxCounter(Ptr<const Packet> /*p*/)
{
    ++g_appSent;
}

// Called whenever a Wi-Fi device throws a frame away. We only count data frames
// (large ones), not the small routing-control messages.
static void
MacDrop(std::string context, WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
{
    if (mpdu->GetPacketSize() >= 130 && mpdu->GetPacketSize() < 500)
    {
        // a C2 message
        if (reason == WIFI_MAC_DROP_REACHED_RETRY_LIMIT)
        {
            std::size_t a = context.find("/NodeList/") + 10;
            std::size_t b = context.find('/', a);
            Ptr<Node> sender = NodeList::GetNode(std::atoi(context.substr(a, b - a).c_str()));
            auto it = g_macToNode.find(mpdu->GetHeader().GetAddr1());
            double d = 0.0;
            if (it != g_macToNode.end())
                d = CalculateDistance(sender->GetObject<MobilityModel>()->GetPosition(),
                                      it->second->GetObject<MobilityModel>()->GetPosition());
            ++g_c2Bin[d < 300 ? 0 : d < 500 ? 1 : d < 650 ? 2 : 3];
            if (d > g_linkRange)
                ++g_c2RetryFar;
            else
                ++g_c2RetryNear;
        }
        else if (reason == WIFI_MAC_DROP_EXPIRED_LIFETIME)
            ++g_c2MacExpired;
        else if (reason == WIFI_MAC_DROP_FAILED_ENQUEUE)
            ++g_c2MacFull;
        return;
    }
    if (mpdu->GetPacketSize() < 500)
    {
        return; // routing / control frame, not camera data
    }
    switch (reason)
    {
    case WIFI_MAC_DROP_REACHED_RETRY_LIMIT: {
        // Which node was sending? The context string looks like "/NodeList/12/DeviceList/0/..."
        std::size_t a = context.find("/NodeList/") + 10;
        std::size_t b = context.find('/', a);
        uint32_t senderId = std::atoi(context.substr(a, b - a).c_str());
        Ptr<Node> sender = NodeList::GetNode(senderId);
        auto it = g_macToNode.find(mpdu->GetHeader().GetAddr1());
        if (it == g_macToNode.end())
        {
            ++g_macRetryNear; // unknown receiver: count as near (conservative)
            break;
        }
        double d = CalculateDistance(sender->GetObject<MobilityModel>()->GetPosition(),
                                     it->second->GetObject<MobilityModel>()->GetPosition());
        if (d < 300)
            ++g_dropBin[0];
        else if (d < 500)
            ++g_dropBin[1];
        else if (d < 650)
            ++g_dropBin[2];
        else
            ++g_dropBin[3];
        if (d > g_linkRange)
        {
            ++g_macRetryFar;
        }
        else
        {
            ++g_macRetryNear;
        }
        break;
    }
    case WIFI_MAC_DROP_FAILED_ENQUEUE:
        ++g_macQueueFull;
        break;
    case WIFI_MAC_DROP_EXPIRED_LIFETIME:
        ++g_macExpired;
        break;
    default:
        break;
    }
}

// ARP drops: only count data-sized packets
static void
ArpQueueDrop(Ptr<const Packet> p)
{
    if (p->GetSize() >= 500)
    {
        ++g_arpQueueDrop;
    }
}

static void
ArpTimeoutDrop(Ptr<const Packet> p)
{
    if (p->GetSize() >= 500)
    {
        ++g_arpTimeoutDrop;
    }
}


// =====================================================================================
//  N2 PROTOTYPE: trajectory-aware, make-before-break forwarding  (--routing=N2)
//
//  * Every drone broadcasts a 40-byte beacon every beaconMs (its position + velocity).
//    The beacons are really transmitted, so their airtime is charged against N2.
//  * Every beaconMs, each drone re-chooses its next hop toward the ground station:
//      - a neighbour is "safe" only if it is within safeFrac x hearRange of us NOW and
//        is PREDICTED to still be within that distance predMs from now
//        (prediction = current position + velocity x time);
//      - among safe neighbours that are closer to the ground than we are, pick the one
//        whose predicted position is closest to the ground (greedy geographic forwarding;
//        strict progress means packets cannot loop);
//      - if no safe neighbour exists, fall back to the best neighbour within hearing range.
//  * The choice is written into each drone's routing table as a host route to the ground.
//  This is "make-before-break": a relay that is about to drift away is replaced BEFORE
//  the link fails, instead of after OLSR notices missing hellos.
// =====================================================================================
static NodeContainer g_n2Nodes;          // all nodes; the ground station is the last one
static Ipv4InterfaceContainer g_n2Ifaces;
static Ipv4Address g_n2Ground;
static double g_n2Hear = 365.0;          // metres: real radio range (preamble detection limit)
static double g_n2SafeFrac = 0.85;       // use links only within this fraction of the range
static double g_n2PredS = 0.5;           // seconds ahead to predict
static uint64_t g_n2Switches = 0;        // times a drone changed its next hop
static uint64_t g_n2Fallback = 0;        // times no SAFE neighbour existed
static uint64_t g_n2NoNeighbour = 0;     // times no neighbour at all was usable
static std::vector<int> g_n2NextHop;
// ---- version 11: C2 sent along two paths ----
// Each drone also keeps a SECOND relay toward the ground (different from its first choice).
// The second copy of every C2 message carries a mark in the IP header (TOS field); every
// drone on the way forwards marked packets to its second relay, so the two copies travel
// along largely separate paths. The ground station keeps whichever copy arrives first.
static std::vector<int> g_n2NextHop2;
static const uint8_t g_c2DupTos = 0x20;
static uint32_t g_c2Route = 0;          // 1 = C2 uses its own relay chain with shorter, safer links
static double g_c2SafeFrac = 0.6;        // ... links no longer than this share of the radio range
static uint32_t g_c2Copies = 0;          // 0 = old C2 apps (one copy), 1 = own C2 app one copy, 2 = two copies
struct C2Rec
{
    std::map<uint64_t, double> firstDelay; // (source, sequence) -> delay of the first copy, seconds
    uint64_t dupes{0};
};
static C2Rec g_c2Rec;

// =====================================================================================
//  Version 12: TWO-WAY C2 (uplink drone -> ground AND downlink ground -> drone)
// =====================================================================================
//  Everything up to version 11b measured C2 in one direction only: ten drones reporting
//  to the ground station. Real C2 is two-way, and the 3GPP TS 22.125 direct-steering
//  profile this traffic is taken from is actually a COMMAND from the controller to the
//  UAV. With --c2Downlink=1 the ground station also sends a 120-byte command every 40 ms
//  to each C2 drone, with the same 40 ms deadline, through the same protected N1 window
//  and (with --c2Route=1) under the same short-link rule.
//
//  Routing toward a MOVING destination needs to know where the destination is. Uplink
//  does not need this: the ground station never moves, so "closer to the ground" is a
//  quantity every drone can compute on its own. Downlink has no such shortcut. In a real
//  system the uplink report carries the drone's own GPS fix, so the ground station knows
//  where each C2 drone was when it last reported, and can put that fix in the command it
//  sends back. That is exactly what is modelled here:
//
//    * g_c2FixInFlight[(k,seq)] is the fix drone k stamped on its uplink report seq.
//    * g_c2DestBelief[k] is updated ONLY when that report actually ARRIVES at the ground
//      (so a lost or late report leaves the ground with an older fix).
//    * every relay extrapolates from that one stale fix, which is what it would do with a
//      fix carried inside the packet.
//
//  --c2DlPos=ideal replaces the fix with the destination's true current position. That is
//  not implementable, but it brackets the result: the gap between "tracked" and "ideal"
//  is the cost of not knowing exactly where the destination is.
struct C2Fix
{
    Vector p;
    Vector v;
    double t{-1.0};
};
static Vector Noisy(Vector p); // defined further down, with the other N2 helpers
static bool g_n2Real = false;  // true = beliefs from beacons; false = exact (moved up: the
                               // C2 apps stamp their own fix and need to know which mode)

static bool g_c2Downlink = false;
static bool g_c2DlIdealPos = false;        // true = exact destination position (upper bound)
static double g_c2DlMaxAgeS = 1.0;         // a destination fix older than this is unusable
static std::vector<uint32_t> g_c2DestNode; // C2 drone k -> node index
static std::vector<Ipv4Address> g_c2DestAddr;
static std::map<uint32_t, uint32_t> g_ipToDl;  // destination IP -> k
static std::map<uint32_t, uint32_t> g_nodeToDl; // node index -> k
static std::vector<C2Fix> g_c2DestBelief;      // ground's last known fix of each C2 drone
static std::map<uint64_t, C2Fix> g_c2FixInFlight;
static std::vector<std::vector<int>> g_n2NextHopDl; // [node][k] -> next hop, -1 = none
static uint32_t g_groundIdx = 0;
static C2Rec g_c2RecDl;
static double g_c2DlExpected = 0.0;
static uint64_t g_c2DlSwitches = 0;
static uint64_t g_c2DlNoPos = 0;    // no usable destination fix, so no downlink route
static uint64_t g_c2DlNoRelay = 0;  // fix known, but no safe relay toward it
static uint64_t g_c2DlSrcFail = 0;  // the ground station could not send at all
static uint64_t g_c2StaleDl = 0;    // downlink C2 discarded as too old

// =====================================================================================
//  Version 13: PRED, a predictive-routing BASELINE (--routing=PRED)
// =====================================================================================
//  N2 is compared against OLSR and AODV, and neither of those predicts anything. That
//  comparison cannot show what N2's prediction is worth, because any predictive scheme
//  would beat them. Published predictive routing for mobile ad-hoc and UAV networks goes
//  back to the link-expiration-time (LET) rule of Su, Lee and Gerla, and to P-OLSR for
//  fixed-wing UAVs. PRED is that idea, implemented here as a baseline:
//
//    * LET for each neighbour is computed from relative position and relative velocity:
//      the time until the distance between the two drones would exceed the radio range.
//      A link is usable while its LET is above --letThreshS.
//    * The route is written into the IP table at each update and then LEFT ALONE, so the
//      next hop is whatever the table said when the packet was enqueued.
//    * When no neighbour qualifies, the PREVIOUS route is kept. Packets go into a link
//      that is already failing, which is what a routing table does.
//    * C2 follows the same route as video: no separate relay chain.
//
//  It uses the same beacons, the same beacon loss, the same GPS error, the same radio and
//  the same N1 window as N2, so the only differences left are the three mechanisms this
//  proposal actually claims: late binding of the next hop at the moment of transmission,
//  holding data when no relay qualifies, and a traffic-class-specific relay chain.
//
//  PRED is NOT a reimplementation of any published protocol and must not be reported as
//  one. It is the LET prediction rule on this project's own routing substrate, which is
//  what makes the ablation honest: every step of the ladder differs by one mechanism.
static bool g_predMode = false;     // true = PRED baseline instead of N2
static bool g_predHold = false;     // PRED + hold-and-deadline (ladder step 3)
static double g_letThreshS = 1.0;   // a link is usable while its LET exceeds this
static bool g_baseQueue = false;    // give OLSR/AODV N2's queue discipline, not its routing
static uint64_t g_predStaleKeep = 0; // times PRED kept a route no neighbour justified

// Link expiration time: how long until these two drones drift out of radio range,
// from their current relative position and velocity. Su, Lee and Gerla's rule.
// Returns a large number when the relative motion keeps them in range indefinitely.
static double
LinkExpiry(const Vector& pi, const Vector& vi, const Vector& pj, const Vector& vj, double range)
{
    Vector d(pi.x - pj.x, pi.y - pj.y, pi.z - pj.z);
    Vector w(vi.x - vj.x, vi.y - vj.y, vi.z - vj.z);
    double a = w.x * w.x + w.y * w.y + w.z * w.z;
    double c = d.x * d.x + d.y * d.y + d.z * d.z - range * range;
    if (c > 0.0)
        return 0.0; // already out of range
    if (a < 1e-9)
        return 1e9; // no relative motion: the link does not expire
    double b = 2.0 * (d.x * w.x + d.y * w.y + d.z * w.z);
    double disc = b * b - 4.0 * a * c;
    if (disc <= 0.0)
        return 1e9;
    double t = (-b + std::sqrt(disc)) / (2.0 * a);
    return t > 0.0 ? t : 0.0;
}

static void
C2Receive(Ptr<Socket> s)
{
    Address from;
    Ptr<Packet> pk;
    while ((pk = s->RecvFrom(from)))
    {
        SeqTsHeader h;
        pk->RemoveHeader(h);
        uint64_t src = InetSocketAddress::ConvertFrom(from).GetIpv4().Get();
        uint64_t key = (src << 32) | h.GetSeq();
        double d = (Simulator::Now() - h.GetTs()).GetSeconds();
        if (g_c2Rec.firstDelay.count(key))
            ++g_c2Rec.dupes;
        else
            g_c2Rec.firstDelay[key] = d;

        // The report has arrived, so the ground station now knows where this drone was
        // when it sent it. This is the only way the ground learns the position of a drone
        // that is several hops away, and it is what the downlink route is computed from.
        if (g_c2Downlink)
        {
            auto ni = g_ipToDl.find(static_cast<uint32_t>(src));
            if (ni != g_ipToDl.end())
            {
                uint64_t fkey = (static_cast<uint64_t>(ni->second) << 32) | h.GetSeq();
                auto fi = g_c2FixInFlight.find(fkey);
                if (fi != g_c2FixInFlight.end())
                {
                    if (fi->second.t > g_c2DestBelief[ni->second].t)
                        g_c2DestBelief[ni->second] = fi->second;
                    g_c2FixInFlight.erase(fi);
                }
            }
            // drop fixes whose report will never arrive, so the map stays small
            if (g_c2FixInFlight.size() > 8 * g_c2DestBelief.size() + 16)
            {
                double cut = Simulator::Now().GetSeconds() - 2.0;
                for (auto it = g_c2FixInFlight.begin(); it != g_c2FixInFlight.end();)
                    it = (it->second.t < cut) ? g_c2FixInFlight.erase(it) : std::next(it);
            }
        }
    }
}

// Downlink command received at drone k. One callback per drone, so k is known.
static void
C2ReceiveDl(uint32_t k, Ptr<Socket> s)
{
    Address from;
    Ptr<Packet> pk;
    while ((pk = s->RecvFrom(from)))
    {
        SeqTsHeader h;
        pk->RemoveHeader(h);
        uint64_t key = (static_cast<uint64_t>(k) << 32) | h.GetSeq();
        double d = (Simulator::Now() - h.GetTs()).GetSeconds();
        if (g_c2RecDl.firstDelay.count(key))
            ++g_c2RecDl.dupes;
        else
            g_c2RecDl.firstDelay[key] = d;
    }
}

static void
C2SendTick(uint32_t srcIdx, Ptr<Socket> s1, Ptr<Socket> s2, uint32_t seq, uint32_t bytes,
           Time interval, Time stop)
{
    if (Simulator::Now() >= stop)
        return;
    // Stamp this report with my own GPS fix, as a real telemetry/C2 report would carry it.
    if (g_c2Downlink && srcIdx < g_n2Nodes.GetN())
    {
        auto ki = g_nodeToDl.find(srcIdx);
        if (ki != g_nodeToDl.end())
        {
            Ptr<MobilityModel> m = g_n2Nodes.Get(srcIdx)->GetObject<MobilityModel>();
            C2Fix f;
            f.p = g_n2Real ? Noisy(m->GetPosition()) : m->GetPosition();
            f.v = m->GetVelocity();
            f.t = Simulator::Now().GetSeconds();
            g_c2FixInFlight[(static_cast<uint64_t>(ki->second) << 32) | seq] = f;
        }
    }
    for (Ptr<Socket> s : {s1, s2})
    {
        if (!s)
            continue;
        SeqTsHeader h;
        h.SetSeq(seq);
        Ptr<Packet> pk = Create<Packet>(bytes > 12 ? bytes - 12 : 0);
        pk->AddHeader(h);
        if (s->Send(pk) < 0)
            ++g_c2SrcFail;
    }
    Simulator::Schedule(interval, &C2SendTick, srcIdx, s1, s2, seq + 1, bytes, interval, stop);
}

// The ground station's command to drone k: same size and period as the uplink report.
static void
C2SendTickDl(uint32_t k, Ptr<Socket> s, uint32_t seq, uint32_t bytes, Time interval, Time stop)
{
    if (Simulator::Now() >= stop)
        return;
    SeqTsHeader h;
    h.SetSeq(seq);
    Ptr<Packet> pk = Create<Packet>(bytes > 12 ? bytes - 12 : 0);
    pk->AddHeader(h);
    if (s->Send(pk) < 0)
        ++g_c2DlSrcFail;
    Simulator::Schedule(interval, &C2SendTickDl, k, s, seq + 1, bytes, interval, stop);
}

// ---------------- N2 version 6 additions ----------------
// (a) What each drone BELIEVES about its neighbours, learned only from beacons it actually
//     received: position (with GPS error), velocity, and when the beacon was sent.
//     --n2Info=ideal uses exact current positions instead (the earlier, optimistic prototype).
// (b) Hold-and-deadline: a drone with no usable relay HOLDS data instead of transmitting into
//     empty air; data older than --deadlineMs is discarded. The next hop is chosen at the
//     moment a packet leaves the drone ("late binding"), so held packets go to the newest relay.
// (c) Physical connectivity logging: is each camera drone connected to the ground at all?
struct N2Belief
{
    Vector p;
    Vector v;
    double t{-1.0};
};
static std::vector<std::vector<N2Belief>> g_n2Seen; // g_n2Seen[i][j] = what i believes about j
// (g_n2Real is declared further up, with the version-12 globals, because the C2 apps need it)
static double g_n2BeaconLoss = 0.0;                 // probability a beacon is lost at a receiver
static double g_n2GpsErr = 0.0;                     // GPS error, metres (standard deviation per axis)
static double g_n2BeaconS = 0.1;                    // beacon interval, seconds
static Ptr<NormalRandomVariable> g_n2Gps;
static Ptr<UniformRandomVariable> g_n2Uni;
static std::vector<Address> g_n2Macs;               // hardware address of each node
static std::vector<Ptr<QueueDisc>> g_n2Hold;        // hold queue of each node
static uint64_t g_n2DeadlineDrops = 0;              // data discarded because it was too old

// N1 v2: funnel-zone TDMA (works together with N2). Every superframe has two phases:
//   Phase A (first aFrac of the superframe): only the "funnel zone" drones -- those whose
//            next hop is the ground station -- may send data, each in its OWN slot,
//            so transmissions into the ground station never collide. Everyone else is silent.
//   Phase B (the rest): all other drones forward data with normal CSMA; zone drones are
//            silent and receive. The slot map follows from the positions in N2 beacons,
//            so no extra signalling is needed.
static bool g_n1Zone = false;
static Time g_n1Sf = MilliSeconds(20);    // superframe length
static double g_n1AFrac = 0.5;            // share of the superframe reserved for the funnel zone
static Time g_n1Slot = MicroSeconds(1100); // one slot = one data packet + ACK + guard
static std::vector<uint32_t> g_n1Demand;  // backlog each drone reported in its last beacon

// N1 v3: protected command-and-control (C2) window.
// Each superframe (g_n1Sf) starts with a short window in which ONLY C2 messages may be sent;
// video and beacons wait until it ends. Outside the window, C2 has priority over video, and
// video is handed to the radio one packet at a time so it cannot spill into the next window.
static bool g_c2Prio = true;              // C2 queued ahead of video (false = same queue as video)
static uint32_t g_n1C2 = 0;               // 0 = off, 1 = shared C2 window (v3), 2 = own C2 slots (v4)
static Time g_c2Win = MilliSeconds(3);    // window length at the start of each superframe
static Time g_c2Tx = MicroSeconds(400);   // airtime for one C2 message incl. ACK
static Time g_c2Deadline = MilliSeconds(40); // C2 older than this is useless and is discarded
static double g_c2MaxFrac = 0.5;          // v4: at most this share of a superframe for C2 slots
static std::vector<uint32_t> g_c2Count;   // C2 messages each drone queued since the last beacon
static std::vector<uint32_t> g_c2Activity;// ... as reported in its last beacon
static uint64_t g_c2Stale = 0;            // C2 discarded as too old
// v4 slot plan for the current superframe (same for every drone: built from beacon data)
static int64_t g_planSf = -1;
static std::vector<int64_t> g_planFirst, g_planCount;
static int64_t g_planWinNs = 0;

// v4: build the C2 slot plan for superframe 'sfIndex'. Drones that carried C2 recently get
// their own slots, ordered from the farthest from the ground to the nearest, so a message
// can be relayed several hops toward the ground inside ONE window.
static void
C2Plan(int64_t sfIndex)
{
    if (sfIndex == g_planSf)
        return;
    g_planSf = sfIndex;
    uint32_t n = g_n2NextHop.size();
    g_planFirst.assign(n, -1);
    g_planCount.assign(n, 0);
    g_planWinNs = 0;
    if (g_c2Activity.size() < n)
        return;
    Vector gpos = g_n2Nodes.Get(n)->GetObject<MobilityModel>()->GetPosition();
    std::vector<std::pair<double, uint32_t>> act;
    for (uint32_t i = 0; i < n; ++i)
    {
        if (g_c2Activity[i] > 0)
        {
            double d = CalculateDistance(g_n2Nodes.Get(i)->GetObject<MobilityModel>()->GetPosition(), gpos);
            act.push_back({-d, i}); // farthest first
        }
    }
    std::sort(act.begin(), act.end());
    int64_t sfNs = g_n1Sf.GetNanoSeconds();
    int64_t slotNs = g_c2Tx.GetNanoSeconds();
    int64_t maxSlots = static_cast<int64_t>(sfNs * g_c2MaxFrac) / slotNs;
    double sfSec = g_n1Sf.GetSeconds();
    int64_t next = 0;
    for (auto& a : act)
    {
        uint32_t i = a.second;
        int64_t k = static_cast<int64_t>(std::ceil(g_c2Activity[i] * sfSec / 0.1));
        k = std::max<int64_t>(1, k);
        k = std::min(k, maxSlots - next);
        if (k <= 0)
            break;
        g_planFirst[i] = next;
        g_planCount[i] = k;
        next += k;
    }
    g_planWinNs = next * slotNs;
}
static Time g_n1Tx = MicroSeconds(1000);  // airtime needed for one data packet incl. ACK
static Time g_n1Jit = MicroSeconds(300);  // random start offset in phase B

static Vector
Noisy(Vector p)
{
    if (g_n2GpsErr <= 0)
        return p;
    return Vector(p.x + g_n2Gps->GetValue(), p.y + g_n2Gps->GetValue(), p.z + g_n2Gps->GetValue());
}

// Where does drone i think drone j is now, and where will j be 'ahead' seconds from now?
// Returns false if i has no fresh beacon from j.
static bool
N2Believe(uint32_t i, uint32_t j, double ahead, Vector& now, Vector& later)
{
    Ptr<MobilityModel> mj = g_n2Nodes.Get(j)->GetObject<MobilityModel>();
    if (!g_n2Real)
    {
        Vector p = mj->GetPosition(), v = mj->GetVelocity();
        now = p;
        later = Vector(p.x + v.x * ahead, p.y + v.y * ahead, p.z + v.z * ahead);
        return true;
    }
    const N2Belief& b = g_n2Seen[i][j];
    if (b.t < 0)
        return false;
    double age = Simulator::Now().GetSeconds() - b.t;
    if (age > 3.0 * g_n2BeaconS)
        return false; // too old: treat the neighbour as unknown
    now = Vector(b.p.x + b.v.x * age, b.p.y + b.v.y * age, b.p.z + b.v.z * age);
    double a2 = age + ahead;
    later = Vector(b.p.x + b.v.x * a2, b.p.y + b.v.y * a2, b.p.z + b.v.z * a2);
    return true;
}

// Hold queue: control packets (small) always pass; data waits while there is no relay,
// is discarded when older than the deadline, and is addressed to the CURRENT relay on release.
namespace ns3
{
class N2HoldQueueDisc : public QueueDisc
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::N2HoldQueueDisc")
                                .SetParent<QueueDisc>()
                                .SetGroupName("TrafficControl")
                                .AddConstructor<N2HoldQueueDisc>();
        return tid;
    }

    N2HoldQueueDisc()
        : QueueDisc(QueueDiscSizePolicy::NO_LIMITS)
    {
    }

    uint32_t DataBacklog()
    {
        return GetNInternalQueues() > 1 ? GetInternalQueue(1)->GetNPackets() : 0;
    }

    void Configure(uint32_t idx, Time deadline, Ptr<WifiMacQueue> macQueue)
    {
        m_idx = idx;
        m_deadline = deadline;
        m_macQueue = macQueue;
        m_rng = CreateObject<UniformRandomVariable>();
    }

  private:
    // queue 0 = beacons (tiny), 1 = data/video (and C2 when C2 has no priority), 2 = C2
    uint32_t Class(Ptr<const QueueDiscItem> item) const
    {
        uint32_t s = item->GetSize();
        if (s < 100)
            return 0;
        if (s >= 500)
            return 1;
        return g_c2Prio ? 2 : 1;
    }

    bool DoEnqueue(Ptr<QueueDiscItem> item) override
    {
        uint32_t c = Class(item);
        if (c == 2 && m_idx < g_c2Count.size())
            ++g_c2Count[m_idx];
        return GetInternalQueue(c)->Enqueue(item);
    }

    Ptr<QueueDiscItem> Bind(Ptr<QueueDiscItem> item, bool isUav, int nh)
    {
        if (g_predMode)
        {
            return item; // PRED baseline: the next hop is whatever the routing table said
        }
        Ptr<Ipv4QueueDiscItem> ip = DynamicCast<Ipv4QueueDiscItem>(item);
        if (isUav && nh >= 0 && ip && ip->GetHeader().GetDestination() == g_n2Ground)
        {
            // late binding: send to whoever is the best relay right now
            int use = nh;
            // second copy of a C2 message: send it to my SECOND relay
            if (ip->GetHeader().GetTos() == g_c2DupTos && m_idx < g_n2NextHop2.size() &&
                g_n2NextHop2[m_idx] >= 0)
                use = g_n2NextHop2[m_idx];
            return Create<Ipv4QueueDiscItem>(ip->GetPacket(), g_n2Macs[use], ip->GetProtocol(),
                                             ip->GetHeader());
        }
        // Downlink C2: late binding toward a MOVING destination. Works the same way at the
        // ground station (first hop) and at every relay on the way.
        if (g_c2Downlink && ip && m_idx < g_n2NextHopDl.size())
        {
            auto it = g_ipToDl.find(ip->GetHeader().GetDestination().Get());
            if (it != g_ipToDl.end())
            {
                int use = g_n2NextHopDl[m_idx][it->second];
                if (use >= 0)
                    return Create<Ipv4QueueDiscItem>(ip->GetPacket(), g_n2Macs[use],
                                                     ip->GetProtocol(), ip->GetHeader());
            }
        }
        return item;
    }

    // Is this a downlink command (ground -> drone) rather than an uplink report?
    static bool IsDownlink(Ptr<QueueDiscItem> item)
    {
        if (!g_c2Downlink)
            return false;
        Ptr<Ipv4QueueDiscItem> ip = DynamicCast<Ipv4QueueDiscItem>(item);
        return ip && g_ipToDl.count(ip->GetHeader().GetDestination().Get()) > 0;
    }

    Ptr<QueueDiscItem> DoDequeue() override
    {
        bool isUav = m_idx < g_n2NextHop.size();
        int nh = isUav ? g_n2NextHop[m_idx] : 0;
        auto bq = GetInternalQueue(0);
        auto dq = GetInternalQueue(1);
        auto cq = GetInternalQueue(2);
        // Two separate things must not be confused here.
        //
        //  (a) Discarding data past its deadline is ordinary queue management. Any sane
        //      implementation does it, and OLSR and AODV get it from the radio's own
        //      MaxDelay. So the PRED baseline KEEPS it. Taking it away would leave PRED
        //      with a worse queue than OLSR has and the comparison would be worthless:
        //      its queues fill with packets that are already useless, and the measured
        //      delay runs into seconds for reasons that have nothing to do with routing.
        //
        //  (b) Refusing to transmit at all while no relay qualifies -- holding the packet
        //      in the hope that a relay appears -- is a routing decision, and it is one of
        //      the mechanisms this proposal claims. PRED does without it unless
        //      --predHold=1, which is step 3 of the ladder.
        bool holdOn = !g_predMode || g_predHold;
        // discard data that is already too old to be useful (baseline keeps this)
        while (dq->GetNPackets() > 0 && m_deadline.IsStrictlyPositive() &&
               Simulator::Now() - dq->Peek()->GetTimeStamp() > m_deadline)
        {
            Ptr<QueueDiscItem> old = dq->Dequeue();
            DropAfterDequeue(old, "Deadline");
            ++g_n2DeadlineDrops;
        }
        // discard C2 too old to be useful
        while (cq->GetNPackets() > 0 && g_n1C2 > 0 &&
               Simulator::Now() - cq->Peek()->GetTimeStamp() > g_c2Deadline)
        {
            Ptr<QueueDiscItem> old = cq->Dequeue();
            DropAfterDequeue(old, "C2 deadline");
            if (IsDownlink(old))
                ++g_c2StaleDl;
            else
                ++g_c2Stale;
        }
        bool noRelay = isUav && nh < 0 && holdOn;
        bool radioBusy = m_macQueue && m_macQueue->GetNPackets() > 0;
        // When the ground station also sends C2, it must respect the protected window too,
        // otherwise its commands would collide with the uplink reports inside the window.
        bool dlGround = g_c2Downlink && !isUav;
        bool winOn = g_n1C2 > 0 && (isUav || dlGround);
        int64_t now = Simulator::Now().GetNanoSeconds();
        int64_t sf = g_n1Sf.GetNanoSeconds();
        int64_t off = now % sf;
        int64_t win = g_c2Win.GetNanoSeconds();
        if (g_n1C2 == 2)
        {
            C2Plan(now / sf);
            win = g_planWinNs;
        }

        if (winOn && isUav && g_n1C2 == 2 && off < win)
        {
            // v4: only in MY slots, one C2 message per slot
            int64_t slotNs = g_c2Tx.GetNanoSeconds();
            int64_t first = g_planFirst[m_idx], count = g_planCount[m_idx];
            if (count > 0)
            {
                int64_t st = first * slotNs, en = st + count * slotNs;
                if (off < st)
                {
                    WakeLater(NanoSeconds(st - off));
                    return nullptr;
                }
                if (off < en && cq->GetNPackets() > 0 && !noRelay && en - off >= slotNs - 50000)
                {
                    if (radioBusy)
                    {
                        WakeLater(MicroSeconds(30));
                        return nullptr;
                    }
                    Ptr<QueueDiscItem> c2 = cq->Dequeue();
                    // next message (if any) waits for my next slot boundary
                    int64_t nextSlot = st + ((off - st) / slotNs + 1) * slotNs;
                    if (cq->GetNPackets() > 0 && nextSlot < en)
                        WakeLater(NanoSeconds(nextSlot - off));
                    return Bind(c2, isUav, nh);
                }
            }
            WakeLater(NanoSeconds(win - off + 20000));
            return nullptr;
        }

        if (winOn && g_n1C2 == 1 && off < win)
        {
            // inside the protected window: only C2, and only if it can finish before the window ends
            if (cq->GetNPackets() > 0 && !noRelay && win - off >= g_c2Tx.GetNanoSeconds())
            {
                if (radioBusy)
                {
                    WakeLater(MicroSeconds(50));
                    return nullptr;
                }
                Ptr<QueueDiscItem> c2 = cq->Dequeue();
                if (cq->GetNPackets() > 0)
                    WakeLater(MicroSeconds(50));
                return Bind(c2, isUav, nh);
            }
            int64_t jit = static_cast<int64_t>(m_rng->GetValue(0.0, 200000.0)); // 0-200 us
            WakeLater(NanoSeconds(win - off + jit));
            return nullptr;
        }

        // outside the window (or no window): beacons, then C2, then data
        Ptr<QueueDiscItem> b = bq->Dequeue();
        if (b)
            return b;
        if (cq->GetNPackets() > 0 && !noRelay)
        {
            return Bind(cq->Dequeue(), isUav, nh);
        }
        if (dq->GetNPackets() == 0 || noRelay)
        {
            return nullptr; // nothing to send, or no relay: hold (woken when a relay appears)
        }
        if (g_n1Zone && isUav && !N1MaySend(nh))
        {
            return nullptr; // not this drone's turn (a wake-up has been scheduled)
        }
        if (winOn)
        {
            int64_t toWin = sf - off; // time until the next window opens
            if (toWin < g_n1Tx.GetNanoSeconds())
            {
                int64_t jit = static_cast<int64_t>(m_rng->GetValue(0.0, 200000.0));
                WakeLater(NanoSeconds(toWin + win + jit));
                return nullptr;
            }
            if (radioBusy)
            {
                WakeLater(MicroSeconds(100));
                return nullptr;
            }
        }
        Ptr<QueueDiscItem> item = dq->Dequeue();
        if ((g_n1Zone || winOn) && isUav && dq->GetNPackets() > 0)
        {
            WakeLater(MicroSeconds(100)); // more data waiting: check again shortly
        }
        return Bind(item, isUav, nh);
    }

    bool CheckConfig() override
    {
        if (GetNQueueDiscClasses() > 0 || GetNPacketFilters() > 0)
            return false;
        if (GetNInternalQueues() == 0)
        {
            for (int k = 0; k < 3; ++k)
            {
                AddInternalQueue(CreateObjectWithAttributes<DropTailQueue<QueueDiscItem>>(
                    "MaxSize", QueueSizeValue(QueueSize("1000p"))));
            }
        }
        return GetNInternalQueues() == 3;
    }

    void InitializeParams() override
    {
    }

    void WakeLater(Time delay)
    {
        if (!m_wake.IsPending())
        {
            m_wake = Simulator::Schedule(delay, &N2HoldQueueDisc::Wake, this);
        }
    }

    void Wake()
    {
        Run();
    }

    // N1 v2: the ground station's slot plan for superframe 'sfIndex' (S slots in phase A).
    // Zone drones that reported backlog get slots in proportion to it; leftover slots rotate
    // among the other zone drones. Returns this drone's first slot and number of slots.
    void N1Allocation(int64_t sfIndex, int64_t S, int groundIdx, int64_t& first, int64_t& count)
    {
        first = 0;
        count = 0;
        std::vector<uint32_t> active, idle;
        uint64_t sumD = 0;
        for (uint32_t j = 0; j < g_n2NextHop.size(); ++j)
        {
            if (g_n2NextHop[j] != groundIdx)
                continue;
            if (j < g_n1Demand.size() && g_n1Demand[j] > 0)
            {
                active.push_back(j);
                sumD += g_n1Demand[j];
            }
            else
            {
                idle.push_back(j);
            }
        }
        if (S <= 0)
            return;
        // rotate both lists each superframe for fairness
        if (!active.empty())
            std::rotate(active.begin(), active.begin() + (sfIndex % active.size()), active.end());
        if (!idle.empty())
            std::rotate(idle.begin(), idle.begin() + (sfIndex % idle.size()), idle.end());
        int64_t next = 0;
        for (uint32_t j : active)
        {
            if (next >= S)
                break;
            int64_t n = std::max<int64_t>(1, (S * g_n1Demand[j]) / static_cast<int64_t>(sumD));
            n = std::min(n, S - next);
            if (j == m_idx)
            {
                first = next;
                count = n;
            }
            next += n;
        }
        for (uint32_t j : idle)
        {
            if (next >= S)
                break;
            if (j == m_idx)
            {
                first = next;
                count = 1;
            }
            next += 1;
        }
    }

    // N1 v2: may this drone hand a data packet to the radio right now?
    bool N1MaySend(int nh)
    {
        int64_t now = Simulator::Now().GetNanoSeconds();
        int64_t sf = g_n1Sf.GetNanoSeconds();
        int64_t off = now % sf;
        int64_t aEnd = static_cast<int64_t>(sf * g_n1AFrac);
        int64_t tx = g_n1Tx.GetNanoSeconds();
        bool radioBusy = m_macQueue && m_macQueue->GetNPackets() > 0;
        int groundIdx = static_cast<int>(g_n2NextHop.size());

        if (nh == groundIdx)
        {
            // funnel-zone drone: the slots the ground station gave me in phase A
            int64_t slotNs = g_n1Slot.GetNanoSeconds();
            int64_t first = 0, count = 0;
            N1Allocation(now / sf, aEnd / slotNs, groundIdx, first, count);
            int64_t start = first * slotNs, end = start + count * slotNs;
            if (count > 0 && off >= start && off < end)
            {
                if (end - off < tx)
                {
                    WakeLater(NanoSeconds(sf - off));
                    return false;
                }
                if (radioBusy)
                {
                    WakeLater(MicroSeconds(100));
                    return false;
                }
                return true;
            }
            if (count > 0 && off < start)
                WakeLater(NanoSeconds(start - off));
            else
                WakeLater(NanoSeconds(sf - off)); // allocation is recomputed next superframe
            return false;
        }

        // everyone else: phase B only, with a small random start offset
        if (off < aEnd)
        {
            int64_t jit = static_cast<int64_t>(m_rng->GetValue(0.0, g_n1Jit.GetNanoSeconds()));
            WakeLater(NanoSeconds(aEnd - off + jit));
            return false;
        }
        if (sf - off < tx)
        {
            WakeLater(NanoSeconds(sf - off + aEnd));
            return false;
        }
        if (radioBusy)
        {
            WakeLater(MicroSeconds(100));
            return false;
        }
        return true;
    }

    void DoDispose() override
    {
        m_wake.Cancel();
        m_macQueue = nullptr;
        QueueDisc::DoDispose();
    }

    uint32_t m_idx{0};
    Time m_deadline{Seconds(0)};
    Ptr<WifiMacQueue> m_macQueue;
    Ptr<UniformRandomVariable> m_rng;
    EventId m_wake;
};

NS_OBJECT_ENSURE_REGISTERED(N2HoldQueueDisc);
} // namespace ns3

static void
N2SetRoute(uint32_t i, int nextHop)
{
    Ptr<Ipv4> ipv4 = g_n2Nodes.Get(i)->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper srh;
    Ptr<Ipv4StaticRouting> sr = srh.GetStaticRouting(ipv4);
    for (int k = static_cast<int>(sr->GetNRoutes()) - 1; k >= 0; --k)
    {
        Ipv4RoutingTableEntry e = sr->GetRoute(k);
        if (e.IsHost() && e.GetDest() == g_n2Ground)
        {
            sr->RemoveRoute(k);
        }
    }
    if (nextHop >= 0)
    {
        sr->AddHostRouteTo(g_n2Ground, g_n2Ifaces.GetAddress(nextHop), 1);
    }
    if (i < g_n2Hold.size() && g_n2Hold[i])
    {
        g_n2Hold[i]->Run(); // a relay may have appeared: release held packets
    }
}

// Downlink: write (or clear) node i's host route to C2 drone k.
static void
N2SetRouteDl(uint32_t i, uint32_t k, int nextHop)
{
    Ptr<Ipv4> ipv4 = g_n2Nodes.Get(i)->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper srh;
    Ptr<Ipv4StaticRouting> sr = srh.GetStaticRouting(ipv4);
    Ipv4Address dst = g_c2DestAddr[k];
    for (int r = static_cast<int>(sr->GetNRoutes()) - 1; r >= 0; --r)
    {
        Ipv4RoutingTableEntry e = sr->GetRoute(r);
        if (e.IsHost() && e.GetDest() == dst)
        {
            sr->RemoveRoute(r);
        }
    }
    if (nextHop >= 0)
    {
        sr->AddHostRouteTo(dst, g_n2Ifaces.GetAddress(nextHop), 1);
    }
    if (i < g_n2Hold.size() && g_n2Hold[i])
    {
        g_n2Hold[i]->Run(); // a relay may have appeared: release held commands
    }
}

// Downlink route selection, the mirror image of the uplink rule in N2UpdateRoutes.
// Uplink asks "is this relay closer to the ground station?", which every drone can answer
// because the ground station does not move. Downlink asks "is this relay closer to the
// DESTINATION DRONE?", which needs the destination's position: the fix the destination put
// in its last uplink report, which the ground station received and passes down the chain.
// Progress is always measured against that one shared fix, so the chain cannot loop within
// an update cycle. A relay that has its own fresh beacon from the destination may deliver
// to it directly.
static void
N2UpdateDownlink()
{
    uint32_t nDrones = g_groundIdx;
    double now = Simulator::Now().GetSeconds();
    double limit = g_n2Hear * (g_c2Route == 1 ? g_c2SafeFrac : g_n2SafeFrac);

    for (uint32_t k = 0; k < g_c2DestNode.size(); ++k)
    {
        uint32_t dn = g_c2DestNode[k];
        Vector dp, dpP;
        bool havePos = false;
        if (g_c2DlIdealPos)
        {
            Ptr<MobilityModel> md = g_n2Nodes.Get(dn)->GetObject<MobilityModel>();
            Vector p = md->GetPosition(), v = md->GetVelocity();
            dp = p;
            dpP = Vector(p.x + v.x * g_n2PredS, p.y + v.y * g_n2PredS, p.z + v.z * g_n2PredS);
            havePos = true;
        }
        else
        {
            const C2Fix& f = g_c2DestBelief[k];
            if (f.t >= 0.0 && now - f.t <= g_c2DlMaxAgeS)
            {
                double age = now - f.t;
                double a2 = age + g_n2PredS;
                dp = Vector(f.p.x + f.v.x * age, f.p.y + f.v.y * age, f.p.z + f.v.z * age);
                dpP = Vector(f.p.x + f.v.x * a2, f.p.y + f.v.y * a2, f.p.z + f.v.z * a2);
                havePos = true;
            }
        }

        if (!havePos)
        {
            // Nobody knows where this drone is, so no command can be routed to it.
            ++g_c2DlNoPos;
            for (uint32_t i = 0; i <= g_groundIdx; ++i)
            {
                if (i == dn)
                    continue;
                if (g_n2NextHopDl[i][k] != -1)
                {
                    g_n2NextHopDl[i][k] = -1;
                    N2SetRouteDl(i, k, -1);
                }
            }
            continue;
        }

        for (uint32_t i = 0; i <= g_groundIdx; ++i)
        {
            if (i == dn)
                continue;
            Ptr<MobilityModel> mi = g_n2Nodes.Get(i)->GetObject<MobilityModel>();
            Vector vi = mi->GetVelocity();
            bool isGround = (i == g_groundIdx);
            Vector pi = (g_n2Real && !isGround) ? Noisy(mi->GetPosition()) : mi->GetPosition();
            Vector piP(pi.x + vi.x * g_n2PredS, pi.y + vi.y * g_n2PredS, pi.z + vi.z * g_n2PredS);
            double myDist = CalculateDistance(pi, dp);
            int best = -1;

            // Am I next to the destination? My own beacon from it is fresher than the fix.
            Vector ldp, ldpP;
            bool local = N2Believe(i, dn, g_n2PredS, ldp, ldpP);
            if (local)
            {
                if (CalculateDistance(pi, ldp) <= limit && CalculateDistance(piP, ldpP) <= limit)
                    best = static_cast<int>(dn);
            }
            else if (myDist <= limit && CalculateDistance(piP, dpP) <= limit)
            {
                best = static_cast<int>(dn);
            }

            if (best < 0)
            {
                double bs = 1e18;
                for (uint32_t j = 0; j < nDrones; ++j)
                {
                    if (j == i || j == dn)
                        continue;
                    Vector pj, pjP;
                    if (!N2Believe(i, j, g_n2PredS, pj, pjP))
                        continue; // no fresh beacon from j
                    if (CalculateDistance(pi, pj) > limit || CalculateDistance(piP, pjP) > limit)
                        continue; // not safe now, or predicted to drift away
                    double score = CalculateDistance(pjP, dpP);
                    if (score >= myDist - 1.0)
                        continue; // must get closer to the destination (no loops)
                    if (score < bs)
                    {
                        bs = score;
                        best = static_cast<int>(j);
                    }
                }
            }

            if (best < 0)
            {
                // Fallback: no safe relay; use the best neighbour merely within hearing range.
                ++g_c2DlNoRelay;
                if (myDist < g_n2Hear)
                {
                    best = static_cast<int>(dn);
                }
                else
                {
                    double bestD = myDist - 1.0;
                    for (uint32_t j = 0; j < nDrones; ++j)
                    {
                        if (j == i || j == dn)
                            continue;
                        Vector pj, pjP;
                        if (!N2Believe(i, j, 0.0, pj, pjP))
                            continue;
                        double dd = CalculateDistance(pj, dp);
                        if (CalculateDistance(pi, pj) < g_n2Hear && dd < bestD)
                        {
                            bestD = dd;
                            best = static_cast<int>(j);
                        }
                    }
                }
            }

            if (best != g_n2NextHopDl[i][k])
            {
                if (g_n2NextHopDl[i][k] != -2)
                    ++g_c2DlSwitches;
                g_n2NextHopDl[i][k] = best;
                N2SetRouteDl(i, k, best);
            }
        }
    }
}

static void
N2UpdateRoutes(double intervalS)
{
    uint32_t groundIdx = g_n2Nodes.GetN() - 1;
    double safe = g_n2Hear * g_n2SafeFrac;
    Vector gpos = g_n2Nodes.Get(groundIdx)->GetObject<MobilityModel>()->GetPosition();

    // N1 v4: C2 activity, as it would be reported in each drone's beacon
    if (g_n1C2 > 0)
    {
        if (g_c2Count.size() < groundIdx)
            g_c2Count.assign(groundIdx, 0);
        g_c2Activity = g_c2Count;
        std::fill(g_c2Count.begin(), g_c2Count.end(), 0);
    }

    // N1 v2: each drone's backlog, as it would be reported in its beacon
    if (g_n1Zone)
    {
        g_n1Demand.assign(groundIdx, 0);
        for (uint32_t i = 0; i < groundIdx && i < g_n2Hold.size(); ++i)
        {
            Ptr<N2HoldQueueDisc> h = DynamicCast<N2HoldQueueDisc>(g_n2Hold[i]);
            if (h)
                g_n1Demand[i] = h->DataBacklog();
        }
    }

    for (uint32_t i = 0; i < groundIdx; ++i)
    {
        Ptr<MobilityModel> mi = g_n2Nodes.Get(i)->GetObject<MobilityModel>();
        Vector vi = mi->GetVelocity();
        Vector pi = g_n2Real ? Noisy(mi->GetPosition()) : mi->GetPosition(); // own GPS fix
        Vector piP(pi.x + vi.x * g_n2PredS, pi.y + vi.y * g_n2PredS, pi.z + vi.z * g_n2PredS);
        double myDist = CalculateDistance(pi, gpos);
        int best = -1;

        // Safe neighbours that make progress toward the ground: best and second best
        int relay1 = -1, relay2 = -1;
        {
            double s1 = 1e18, s2 = 1e18;
            for (uint32_t j = 0; j < groundIdx; ++j)
            {
                if (j == i)
                    continue;
                Vector pj, pjP;
                if (!N2Believe(i, j, g_n2PredS, pj, pjP))
                    continue; // no fresh beacon from j
                if (g_predMode)
                {
                    // PRED baseline: usable while the link expiration time is long enough
                    Vector vj((pjP.x - pj.x) / g_n2PredS, (pjP.y - pj.y) / g_n2PredS,
                              (pjP.z - pj.z) / g_n2PredS);
                    if (LinkExpiry(pi, vi, pj, vj, g_n2Hear) < g_letThreshS)
                        continue;
                }
                else if (CalculateDistance(pi, pj) > safe || CalculateDistance(piP, pjP) > safe)
                {
                    continue; // not safe now, or predicted to drift away
                }
                if (CalculateDistance(pj, gpos) >= myDist - 1.0)
                    continue; // must get closer to the ground (no loops)
                double score = CalculateDistance(pjP, gpos);
                if (score < s1)
                {
                    s2 = s1;
                    relay2 = relay1;
                    s1 = score;
                    relay1 = static_cast<int>(j);
                }
                else if (score < s2)
                {
                    s2 = score;
                    relay2 = static_cast<int>(j);
                }
            }
        }
        int second = -1;
        int strictC2 = -1; // v11b: C2 relay using only short (high-margin) links
        if (g_c2Route == 1)
        {
            double sc = g_n2Hear * g_c2SafeFrac;
            if (myDist < sc && CalculateDistance(piP, gpos) < sc)
                strictC2 = static_cast<int>(groundIdx);
            else
            {
                double bs = 1e18;
                for (uint32_t j = 0; j < groundIdx; ++j)
                {
                    if (j == i)
                        continue;
                    Vector pj, pjP;
                    if (!N2Believe(i, j, g_n2PredS, pj, pjP))
                        continue;
                    if (CalculateDistance(pi, pj) > sc || CalculateDistance(piP, pjP) > sc)
                        continue;
                    if (CalculateDistance(pj, gpos) >= myDist - 1.0)
                        continue;
                    double score = CalculateDistance(pjP, gpos);
                    if (score < bs)
                    {
                        bs = score;
                        strictC2 = static_cast<int>(j);
                    }
                }
            }
        }

        // 1. Ground station directly reachable and predicted to stay reachable?
        bool groundOk = g_predMode
                            ? LinkExpiry(pi, vi, gpos, Vector(0, 0, 0), g_n2Hear) >= g_letThreshS
                            : (myDist < safe && CalculateDistance(piP, gpos) < safe);
        if (groundOk)
        {
            best = static_cast<int>(groundIdx);
            second = relay1; // second copy goes via a relay (or also direct if none)
        }
        else
        {
            // 2. Best SAFE neighbour that makes progress toward the ground
            best = relay1;
            second = relay2;
        }

        if (best < 0)
        {
            // 3. Fallback: no safe neighbour; best neighbour within hearing range
            ++g_n2Fallback;
            if (myDist < g_n2Hear)
            {
                best = static_cast<int>(groundIdx);
            }
            else
            {
                double bestD = myDist - 1.0;
                for (uint32_t j = 0; j < groundIdx; ++j)
                {
                    if (j == i)
                        continue;
                    Vector pj, pjP;
                    if (!N2Believe(i, j, 0.0, pj, pjP))
                        continue;
                    double dg = CalculateDistance(pj, gpos);
                    if (CalculateDistance(pi, pj) < g_n2Hear && dg < bestD)
                    {
                        bestD = dg;
                        best = static_cast<int>(j);
                    }
                }
            }
            if (best < 0)
            {
                ++g_n2NoNeighbour; // nobody to send to: data will be HELD
            }
        }

        // PRED baseline: when nothing qualifies, a routing table keeps the entry it has.
        // Packets then go into a link that is already failing, which is the behaviour this
        // proposal claims to avoid by holding instead.
        if (best < 0 && g_predMode && !g_predHold && g_n2NextHop[i] >= 0)
        {
            ++g_predStaleKeep;
            continue;
        }

        if (g_n2NextHop2.size() > i)
            g_n2NextHop2[i] = g_c2Route == 1 ? (strictC2 >= 0 ? strictC2 : best)
                                             : (second >= 0 ? second : best);
        if (best != g_n2NextHop[i])
        {
            if (g_n2NextHop[i] != -2)
                ++g_n2Switches;
            g_n2NextHop[i] = best;
            N2SetRoute(i, best);
        }
    }
    if (g_c2Downlink && !g_c2DestNode.empty())
    {
        N2UpdateDownlink();
    }
    Simulator::Schedule(Seconds(intervalS), &N2UpdateRoutes, intervalS);
}

// Beacon: a real 40-byte broadcast (its airtime is simulated). In "real" mode the
// contents (position with GPS error + velocity) reach only drones within radio range,
// and each receiver loses it with probability beaconLoss.
static void
N2SendBeacon(uint32_t j, Ptr<Socket> s, double intervalS, Ptr<UniformRandomVariable> rng)
{
    s->Send(Create<Packet>(40));
    if (g_n2Real && j + 1 < g_n2Nodes.GetN())
    {
        Ptr<MobilityModel> mj = g_n2Nodes.Get(j)->GetObject<MobilityModel>();
        N2Belief b;
        b.p = Noisy(mj->GetPosition());
        b.v = mj->GetVelocity();
        b.t = Simulator::Now().GetSeconds();
        Vector truePos = mj->GetPosition();
        // With downlink C2 the ground station also listens to beacons: it needs to know
        // which drones are within its own radio range to choose a first relay.
        uint32_t nRx = g_c2Downlink ? g_n2Nodes.GetN() : g_n2Nodes.GetN() - 1;
        for (uint32_t i = 0; i < nRx; ++i)
        {
            if (i == j)
                continue;
            double d = CalculateDistance(truePos, g_n2Nodes.Get(i)->GetObject<MobilityModel>()->GetPosition());
            if (d <= g_n2Hear && g_n2Uni->GetValue() >= g_n2BeaconLoss)
            {
                g_n2Seen[i][j] = b;
            }
        }
    }
    double next = intervalS * (0.9 + 0.2 * rng->GetValue()); // +-10% jitter
    Simulator::Schedule(Seconds(next), &N2SendBeacon, j, s, intervalS, rng);
}

// Physical connectivity: fraction of time each camera drone has ANY chain of links
// (each no longer than the real radio range) to the ground station. Same flights, any routing.
static NodeContainer g_connNodes;
static uint32_t g_connCams = 0;
static uint64_t g_connSamples = 0;
static uint64_t g_connHits = 0;

static void
LogConnectivity(double intervalS, double rangeM)
{
    uint32_t n = g_connNodes.GetN();
    std::vector<Vector> pos(n);
    for (uint32_t k = 0; k < n; ++k)
        pos[k] = g_connNodes.Get(k)->GetObject<MobilityModel>()->GetPosition();
    std::vector<bool> seen(n, false);
    std::vector<uint32_t> stack{n - 1}; // start from the ground station (last node)
    seen[n - 1] = true;
    while (!stack.empty())
    {
        uint32_t u = stack.back();
        stack.pop_back();
        for (uint32_t v = 0; v < n; ++v)
        {
            if (!seen[v] && CalculateDistance(pos[u], pos[v]) <= rangeM)
            {
                seen[v] = true;
                stack.push_back(v);
            }
        }
    }
    for (uint32_t c = 0; c < g_connCams; ++c)
    {
        ++g_connSamples;
        if (seen[c])
            ++g_connHits;
    }
    Simulator::Schedule(Seconds(intervalS), &LogConnectivity, intervalS, rangeM);
}

// Prints the simulated time every few seconds so you can see progress
static void
PrintProgress(double interval)
{
    std::cerr << "  ...simulated time: " << Simulator::Now().GetSeconds() << " s" << std::endl;
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval);
}

int
main(int argc, char* argv[])
{
    // ---------------- 1. Parameters (all changeable from the command line) ----------------
    uint32_t nUav = 20;          // number of drones
    uint32_t nCam = 5;           // how many drones stream video-like traffic
    double simTime = 100.0;      // seconds
    double areaSize = 1000.0;    // metres (square area) -- a swarm flies fairly close together
    double minAlt = 50.0;        // metres
    double maxAlt = 150.0;       // metres
    double txPowerDbm = 16.0;    // transmit power
    std::string flowRate = "300kbps"; // per camera drone
    std::string routing = "OLSR";     // OLSR or AODV
    uint32_t run = 1;            // random-run number (repeat with 1..10 for statistics)
    bool arpPrefill = false;     // true = pre-fill neighbour (ARP) tables
    std::string macMode = "CSMA";     // CSMA (baseline) or N1 (altitude-banded TDMA gate)
    uint32_t bands = 3;               // N1: number of altitude bands
    uint32_t cols = 1;                // N1: number of horizontal columns
    double slotMs = 3.0;              // N1: slot length in milliseconds
    double txTimeUs = 1000.0;         // N1: time to send one 1000-byte packet incl. ACK (~0.9-1.0 ms at 12 Mbps)
    double jitterUs = 300.0;          // N1: random delay at slot start, so group members don't all start at once
    double beaconMs = 100.0;          // N2: beacon / route-update interval
    double predMs = 500.0;            // N2: how far ahead to predict (ms)
    std::string n2Info = "ideal";     // N2: ideal (exact positions) or real (from received beacons)
    std::string c2DlPos = "tracked";  // downlink: tracked (from uplink reports) or ideal
    double deadlineMs = 0.0;          // >0: discard data older than this (all modes; Wi-Fi queue too)
    bool n2Hold = true;               // N2: hold data when no relay exists (instead of sending into the void)

    CommandLine cmd(__FILE__);
    cmd.AddValue("nUav", "Number of UAVs", nUav);
    cmd.AddValue("nCam", "Number of camera UAVs sending to ground", nCam);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("areaSize", "Side of square area (m)", areaSize);
    cmd.AddValue("txPower", "Transmit power (dBm)", txPowerDbm);
    cmd.AddValue("flowRate", "Data rate per camera UAV", flowRate);
    cmd.AddValue("routing",
                 "OLSR, AODV, N2 (trajectory-aware make-before-break), or PRED "
                 "(link-expiration-prediction baseline: prediction only, no late binding, "
                 "no hold, no C2 relay chain)",
                 routing);
    cmd.AddValue("letThreshS", "PRED: a link is usable while its expiration time exceeds this (s)",
                 g_letThreshS);
    cmd.AddValue("predHold",
                 "PRED: when no neighbour qualifies, drop the route and hold instead of "
                 "keeping a stale one (ladder step 3)",
                 g_predHold);
    cmd.AddValue("baseQueue",
                 "OLSR/AODV: use N2's queue discipline (priority classes, deadline discard, "
                 "N1 window) with their own routing, to separate queueing from routing",
                 g_baseQueue);
    cmd.AddValue("run", "Random run number", run);
    cmd.AddValue("arpPrefill", "Pre-fill ARP neighbour tables (0/1)", arpPrefill);
    cmd.AddValue("mac", "CSMA (baseline) or N1 (altitude-banded TDMA)", macMode);
    cmd.AddValue("bands", "N1: number of altitude bands", bands);
    cmd.AddValue("cols", "N1: number of horizontal columns", cols);
    cmd.AddValue("slotMs", "N1: slot length (ms)", slotMs);
    cmd.AddValue("beaconMs", "N2: beacon and route-update interval (ms)", beaconMs);
    cmd.AddValue("predMs", "N2: prediction horizon (ms)", predMs);
    cmd.AddValue("n2Info", "N2: ideal or real neighbour information", n2Info);
    cmd.AddValue("beaconLoss", "N2 real: probability a beacon is lost", g_n2BeaconLoss);
    cmd.AddValue("gpsErrM", "N2 real: GPS error, metres", g_n2GpsErr);
    cmd.AddValue("deadlineMs", "Discard data older than this (0 = off)", deadlineMs);
    cmd.AddValue("n2Hold", "N2: hold data when no relay exists (0/1)", n2Hold);
    double n1SfMs = 20.0;
    cmd.AddValue("n1zone", "N1 v2: funnel-zone TDMA on top of N2 (0/1)", g_n1Zone);
    cmd.AddValue("n1SfMs", "N1 v2: superframe length (ms)", n1SfMs);
    cmd.AddValue("n1AFrac", "N1 v2: share of superframe for the funnel zone", g_n1AFrac);
    uint32_t nC2 = 0;
    double c2IntervalMs = 40.0;
    uint32_t c2Bytes = 120;
    double c2WinMs = 3.0;
    cmd.AddValue("nC2", "Number of drones sending C2 telemetry to the ground", nC2);
    cmd.AddValue("c2IntervalMs", "C2 message interval (ms); 3GPP direct steering = 40", c2IntervalMs);
    cmd.AddValue("c2Bytes", "C2 message size (bytes); 3GPP: 84-140", c2Bytes);
    cmd.AddValue("c2Prio", "C2 queued ahead of video (0/1)", g_c2Prio);
    cmd.AddValue("n1c2", "N1 C2 mode: 0 off, 1 shared window (v3), 2 own slots far-to-near (v4)", g_n1C2);
    double c2DeadlineMs = 40.0;
    cmd.AddValue("c2DeadlineMs", "Discard C2 older than this (ms)", c2DeadlineMs);
    cmd.AddValue("c2MaxFrac", "v4: max share of superframe for C2 slots", g_c2MaxFrac);
    cmd.AddValue("c2WinMs", "N1 v3: C2 window length (ms)", c2WinMs);
    cmd.AddValue("c2Route", "C2 relay chain: 0 same as video, 1 short safe links only (needs c2Copies=1)", g_c2Route);
    cmd.AddValue("c2SafeFrac", "c2Route=1: max link length as a share of radio range", g_c2SafeFrac);
    cmd.AddValue("c2Copies", "C2 copies: 0 old app, 1 own app one path, 2 two paths (needs N2)", g_c2Copies);
    cmd.AddValue("c2Downlink", "Also send C2 from the ground station to each C2 drone (0/1)",
                 g_c2Downlink);
    cmd.AddValue("c2DlPos", "Downlink destination position: tracked (from uplink reports) or ideal",
                 c2DlPos);
    cmd.AddValue("c2DlMaxAgeS", "Downlink: ignore a destination fix older than this (s)",
                 g_c2DlMaxAgeS);
    cmd.AddValue("hearRange", "N2: real radio range (m)", g_n2Hear);
    cmd.AddValue("safeFrac", "N2: use links only within this fraction of range", g_n2SafeFrac);
    cmd.AddValue("jitterUs", "N1: random delay at slot start (us)", jitterUs);
    cmd.AddValue("txTimeUs", "N1: airtime reserved per packet (us)", txTimeUs);
    cmd.AddValue("linkRange", "Approximate radio range used to classify drops (m)", g_linkRange);
    cmd.Parse(argc, argv);

    if (nCam > nUav)
    {
        nCam = nUav;
    }
    if (nC2 > nUav)
    {
        nC2 = nUav; // each C2 drone must be a distinct drone
    }
    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(run);

    if (deadlineMs > 0)
    {
        // Wi-Fi also discards frames that have waited longer than the deadline (default is 500 ms)
        Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(static_cast<int64_t>(deadlineMs))));
    }
    g_n2Real = (n2Info == "real");
    g_c2DlIdealPos = (c2DlPos == "ideal");
    g_predMode = (routing == "PRED");
    if (g_predMode && g_c2Route != 0)
    {
        std::cerr << "  (PRED baseline: c2Route forced to 0. A traffic-class-specific relay\n"
                     "   chain is one of the mechanisms being measured, so the baseline cannot\n"
                     "   have it. Run --routing=N2 to include it.)"
                  << std::endl;
        g_c2Route = 0;
    }
    g_n1Sf = MicroSeconds(static_cast<int64_t>(n1SfMs * 1000));
    g_c2Win = MicroSeconds(static_cast<int64_t>(c2WinMs * 1000));
    g_c2Deadline = MicroSeconds(static_cast<int64_t>(c2DeadlineMs * 1000));
    g_n2BeaconS = beaconMs / 1000.0;

    // ---------------- 2. Nodes ----------------
    NodeContainer uavs;
    uavs.Create(nUav);
    NodeContainer ground;
    ground.Create(1);
    NodeContainer all(uavs, ground);

    // ---------------- 3. Radio: 802.11a ad-hoc (CSMA-CA) ----------------
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate12Mbps"),
                                 "ControlMode", StringValue("OfdmRate6Mbps"));

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // Air-to-air links are mostly line-of-sight, so free-space (Friis) loss is a fair first model
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                               "Frequency", DoubleValue(5.18e9));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(txPowerDbm));
    phy.Set("TxPowerEnd", DoubleValue(txPowerDbm));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, all);

    // Map each Wi-Fi hardware address to its node (used to find the receiver of a dropped frame)
    for (uint32_t i = 0; i < devices.GetN(); ++i)
    {
        g_macToNode[Mac48Address::ConvertFrom(devices.Get(i)->GetAddress())] = devices.Get(i)->GetNode();
    }

    // ---------------- 4. Mobility ----------------
    // Drones: 3D Gauss-Markov, speeds 8-33 m/s (about 30-120 km/h)
    MobilityHelper uavMobility;
    std::ostringstream xs, ys, zs;
    xs << "ns3::UniformRandomVariable[Min=0|Max=" << areaSize << "]";
    ys << "ns3::UniformRandomVariable[Min=0|Max=" << areaSize << "]";
    zs << "ns3::UniformRandomVariable[Min=" << minAlt << "|Max=" << maxAlt << "]";
    uavMobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                     "X", StringValue(xs.str()),
                                     "Y", StringValue(ys.str()),
                                     "Z", StringValue(zs.str()));
    uavMobility.SetMobilityModel(
        "ns3::GaussMarkovMobilityModel",
        "Bounds", BoxValue(Box(0, areaSize, 0, areaSize, minAlt, maxAlt)),
        "TimeStep", TimeValue(Seconds(0.5)),
        "Alpha", DoubleValue(0.85),
        "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=8|Max=33]"),
        "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
        "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
        "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
        "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
        "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));
    uavMobility.Install(uavs);

    // Ground station: fixed at the centre, 1.5 m above ground
    MobilityHelper groundMobility;
    Ptr<ListPositionAllocator> gpos = CreateObject<ListPositionAllocator>();
    gpos->Add(Vector(areaSize / 2, areaSize / 2, 1.5));
    groundMobility.SetPositionAllocator(gpos);
    groundMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    groundMobility.Install(ground);

    // ---------------- 5. Internet stack + multi-hop routing ----------------
    InternetStackHelper internet;
    OlsrHelper olsr;
    AodvHelper aodv;
    Ipv4StaticRoutingHelper staticRouting;
    if (routing == "AODV")
    {
        internet.SetRoutingHelper(aodv);
    }
    else if (routing == "N2" || routing == "PRED")
    {
        internet.SetRoutingHelper(staticRouting); // routes are written by N2UpdateRoutes
    }
    else
    {
        internet.SetRoutingHelper(olsr);
    }
    internet.Install(all);

    // N1: install the TDMA gate on every radio (must happen before IP addresses are assigned,
    // otherwise ns-3 installs its default queue discipline instead)
    Time superframe = Seconds(0);
    if (macMode == "N1")
    {
        TrafficControlHelper tch;
        tch.SetRootQueueDisc("ns3::TdmaGateQueueDisc");
        QueueDiscContainer qdiscs = tch.Install(devices);
        for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
            Ptr<WifiNetDevice> wd = DynamicCast<WifiNetDevice>(devices.Get(i));
            Ptr<WifiMacQueue> mq = wd->GetMac()->GetTxop()->GetWifiMacQueue();
            Ptr<TdmaGateQueueDisc> gate = DynamicCast<TdmaGateQueueDisc>(qdiscs.Get(i));
            bool isGround = (i == nUav); // the ground station is the last device
            gate->Configure(devices.Get(i)->GetNode(), mq, isGround, bands, cols, minAlt, maxAlt,
                            areaSize, MicroSeconds(static_cast<int64_t>(slotMs * 1000)),
                            MicroSeconds(static_cast<int64_t>(txTimeUs)),
                            MicroSeconds(static_cast<int64_t>(jitterUs)));
            superframe = gate->Superframe();
        }
        std::cerr << "  N1 on: " << bands << " bands x " << cols << " columns + ground = "
                  << (bands * cols + 1) << " slots of " << slotMs << " ms; superframe = "
                  << superframe.GetMilliSeconds() << " ms" << std::endl;
    }

    // --baseQueue=1 gives OLSR or AODV the SAME queue discipline that N2 uses (three
    // classes, deadline discard, and the N1 C2 window) while leaving their own routing
    // alone. Without it, part of any N2-versus-OLSR gap is the queue, not the routing,
    // and there is no way to tell how much. This separates the two.
    if ((((routing == "N2" || routing == "PRED") && n2Hold) || g_baseQueue) && macMode != "N1")
    {
        TrafficControlHelper tchHold;
        tchHold.SetRootQueueDisc("ns3::N2HoldQueueDisc");
        QueueDiscContainer hq = tchHold.Install(devices);
        for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
            Ptr<WifiMacQueue> mq =
                DynamicCast<WifiNetDevice>(devices.Get(i))->GetMac()->GetTxop()->GetWifiMacQueue();
            DynamicCast<N2HoldQueueDisc>(hq.Get(i))
                ->Configure(i, MilliSeconds(static_cast<int64_t>(deadlineMs)), mq);
            g_n2Hold.push_back(hq.Get(i));
        }
    }

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);
    Ipv4Address groundAddr = ifaces.GetAddress(nUav); // ground is the last node

    if (arpPrefill)
    {
        // Give every node the hardware address of every other node up front,
        // so no packet ever waits for neighbour discovery.
        NeighborCacheHelper neighborCache;
        neighborCache.PopulateNeighborCache();
    }

    // Two-way C2: which drones carry C2, and their addresses. Set up before routing starts.
    g_groundIdx = nUav;
    if (g_c2Downlink && nC2 > 0)
    {
        for (uint32_t k = 0; k < nC2; ++k)
        {
            uint32_t idx = (nCam + k) % nUav;
            g_c2DestNode.push_back(idx);
            g_c2DestAddr.push_back(ifaces.GetAddress(idx));
            g_ipToDl[ifaces.GetAddress(idx).Get()] = k;
            g_nodeToDl[idx] = k;
        }
        g_c2DestBelief.assign(nC2, C2Fix());
        g_n2NextHopDl.assign(nUav + 1, std::vector<int>(nC2, -2));
        std::cerr << "  C2 downlink on: ground -> " << nC2 << " drones, " << c2Bytes
                  << " B every " << c2IntervalMs << " ms, destination position = " << c2DlPos
                  << (g_c2DlIdealPos ? "" : " (from uplink reports)") << std::endl;
        if ((routing == "N2" || routing == "PRED") && !g_c2DlIdealPos && g_c2Copies == 0)
        {
            std::cerr << "  *** c2Downlink=1 with c2DlPos=tracked needs c2Copies=1: the ground\n"
                         "  *** station learns where each drone is from the numbered uplink\n"
                         "  *** reports, which only the c2Copies=1 app sends. Add --c2Copies=1\n"
                         "  *** (or use --c2DlPos=ideal) or no command will be routed."
                      << std::endl;
        }
    }

    if (routing == "N2" || routing == "PRED")
    {
        g_n2Nodes = all;
        g_n2Ifaces = ifaces;
        g_n2Ground = groundAddr;
        g_n2PredS = predMs / 1000.0;
        g_n2NextHop.assign(nUav, -2); // -2 = no route chosen yet
        g_n2NextHop2.assign(nUav, -1);
        g_c2Count.assign(nUav, 0);
        g_c2Activity.assign(nUav, 0);
        for (uint32_t i = 0; i < devices.GetN(); ++i)
            g_n2Macs.push_back(devices.Get(i)->GetAddress());
        g_n2Seen.assign(all.GetN(), std::vector<N2Belief>(all.GetN()));
        g_n2Gps = CreateObject<NormalRandomVariable>();
        g_n2Gps->SetAttribute("Mean", DoubleValue(0.0));
        g_n2Gps->SetAttribute("Variance", DoubleValue(g_n2GpsErr * g_n2GpsErr));
        g_n2Uni = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> brng = CreateObject<UniformRandomVariable>();
        for (uint32_t i = 0; i < all.GetN(); ++i)
        {
            Ptr<Socket> s = Socket::CreateSocket(all.Get(i), UdpSocketFactory::GetTypeId());
            s->SetAllowBroadcast(true);
            s->Connect(InetSocketAddress(Ipv4Address("10.1.255.255"), 9001));
            Simulator::Schedule(Seconds(1.0 + brng->GetValue(0.0, beaconMs / 1000.0)),
                                &N2SendBeacon, i, s, beaconMs / 1000.0, brng);
        }
        Simulator::Schedule(Seconds(1.0), &N2UpdateRoutes, beaconMs / 1000.0);
        if (g_predMode)
        {
            std::cerr << "  PRED baseline on: link-expiration prediction from the same beacons"
                      << " (loss " << g_n2BeaconLoss << ", GPS error " << g_n2GpsErr << " m),"
                      << " usable while LET > " << g_letThreshS << " s.\n"
                      << "  Deliberately WITHOUT: late binding, "
                      << (g_predHold ? "and WITH hold+deadline, " : "hold+deadline, ")
                      << "and a C2 relay chain."
                      << std::endl;
        }
        std::cerr << "  N2 on: beacons every " << beaconMs << " ms, predict " << predMs
                  << " ms ahead, safe range " << g_n2Hear * g_n2SafeFrac << " m, info=" << n2Info
                  << " (beacon loss " << g_n2BeaconLoss << ", GPS error " << g_n2GpsErr
                  << " m), hold=" << (n2Hold ? 1 : 0) << ", deadline=" << deadlineMs << " ms"
                  << std::endl;
        if (g_n1Zone)
        {
            std::cerr << "  N1 v2 funnel-zone TDMA on: superframe " << n1SfMs << " ms, "
                      << g_n1AFrac * 100 << "% reserved for drones sending directly to ground"
                      << std::endl;
        }
    }

    // ---------------- 6. Traffic: camera drones -> ground ----------------
    uint16_t port = 9000;
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sink.Install(ground.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    double trafficStart = 20.0; // give routing time to converge first
    for (uint32_t i = 0; i < nCam; ++i)
    {
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(groundAddr, port));
        onoff.SetConstantRate(DataRate(flowRate), 1000); // 1000-byte packets
        ApplicationContainer app = onoff.Install(uavs.Get(i));
        app.Start(Seconds(trafficStart + 0.1 * i)); // small offsets so flows don't start together
        app.Stop(Seconds(simTime - 1.0));
    }

    // C2 telemetry: small periodic messages from nC2 drones (not the camera drones) to the ground
    uint16_t c2Port = 9100;
    PacketSinkHelper c2Sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), c2Port));
    ApplicationContainer c2SinkApp = c2Sink.Install(ground.Get(0));
    c2SinkApp.Start(Seconds(0.0));
    c2SinkApp.Stop(Seconds(simTime));
    double c2Expected = 0.0;
    if (nC2 > 0 && g_c2Copies > 0)
    {
        // own C2 app: numbered messages, so the ground can keep the first of two copies
        Ptr<Socket> rx = Socket::CreateSocket(ground.Get(0), UdpSocketFactory::GetTypeId());
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9101));
        rx->SetRecvCallback(MakeCallback(&C2Receive));
        for (uint32_t k = 0; k < nC2; ++k)
        {
            uint32_t srcIdx = (nCam + k) % nUav;
            Ptr<Node> src = uavs.Get(srcIdx);
            Ptr<Socket> s1 = Socket::CreateSocket(src, UdpSocketFactory::GetTypeId());
            s1->Connect(InetSocketAddress(groundAddr, 9101));
            Ptr<Socket> s2;
            if (g_c2Route == 1)
            {
                s1->SetIpTos(g_c2DupTos); // the only copy follows the C2 relay chain
            }
            if (g_c2Copies == 2)
            {
                s2 = Socket::CreateSocket(src, UdpSocketFactory::GetTypeId());
                s2->SetIpTos(g_c2DupTos); // marks the second copy
                s2->Connect(InetSocketAddress(groundAddr, 9101));
            }
            double st = trafficStart + 0.0137 * k;
            Simulator::Schedule(Seconds(st), &C2SendTick, srcIdx, s1, s2, 0u, c2Bytes,
                                MicroSeconds(static_cast<int64_t>(c2IntervalMs * 1000)),
                                Seconds(simTime - 1.0));
            c2Expected += std::ceil((simTime - 1.0 - st) * 1000.0 / c2IntervalMs);
        }
        std::cerr << "  C2 (own app): " << nC2 << " drones x " << c2Bytes << " B every "
                  << c2IntervalMs << " ms, copies=" << g_c2Copies << ", N1 C2 mode=" << g_n1C2
                  << std::endl;
    }
    else if (nC2 > 0)
    {
        uint64_t c2bps = static_cast<uint64_t>(c2Bytes * 8 * 1000.0 / c2IntervalMs);
        for (uint32_t k = 0; k < nC2; ++k)
        {
            OnOffHelper c2("ns3::UdpSocketFactory", InetSocketAddress(groundAddr, c2Port));
            c2.SetConstantRate(DataRate(c2bps), c2Bytes);
            ApplicationContainer a = c2.Install(uavs.Get((nCam + k) % nUav));
            double st = trafficStart + 0.0137 * k;
            a.Start(Seconds(st));
            a.Stop(Seconds(simTime - 1.0));
            c2Expected += (simTime - 1.0 - st) * 1000.0 / c2IntervalMs;
        }
        std::cerr << "  C2: " << nC2 << " drones x " << c2Bytes << " B every " << c2IntervalMs
                  << " ms; priority=" << (g_c2Prio ? 1 : 0) << ", N1 C2 mode=" << g_n1C2;
        if (g_n1C2 == 1)
            std::cerr << " (shared window " << c2WinMs << " ms every " << n1SfMs << " ms)";
        if (g_n1C2 == 2)
            std::cerr << " (own slots every " << n1SfMs << " ms, up to " << g_c2MaxFrac * 100
                      << "% of the cycle)";
        std::cerr << std::endl;
    }

    // ---------------- 6b. Downlink C2: ground station -> each C2 drone ----------------
    // Same message size and period as the uplink report, same 40 ms deadline. Both
    // directions share the one channel and the one protected window, which is the point
    // of the test: the window that was sized for uplink alone now has to carry both.
    if (g_c2Downlink && nC2 > 0)
    {
        for (uint32_t k = 0; k < nC2; ++k)
        {
            uint32_t dstIdx = g_c2DestNode[k];
            Ptr<Socket> rx = Socket::CreateSocket(uavs.Get(dstIdx), UdpSocketFactory::GetTypeId());
            rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9102));
            rx->SetRecvCallback(MakeBoundCallback(&C2ReceiveDl, k));

            Ptr<Socket> tx = Socket::CreateSocket(ground.Get(0), UdpSocketFactory::GetTypeId());
            if (g_c2Route == 1)
            {
                tx->SetIpTos(g_c2DupTos); // commands follow the short-link C2 chain too
            }
            tx->Connect(InetSocketAddress(g_c2DestAddr[k], 9102));
            // Start half a C2 period after the uplink flows, so the ground has at least one
            // position report for each drone before it starts sending commands.
            double st = trafficStart + 0.5 + 0.0137 * k;
            Simulator::Schedule(Seconds(st), &C2SendTickDl, k, tx, 0u, c2Bytes,
                                MicroSeconds(static_cast<int64_t>(c2IntervalMs * 1000)),
                                Seconds(simTime - 1.0));
            g_c2DlExpected += std::ceil((simTime - 1.0 - st) * 1000.0 / c2IntervalMs);
        }
    }

    g_connNodes = all;
    g_connCams = nCam;
    Simulator::Schedule(Seconds(trafficStart), &LogConnectivity, 0.1, g_n2Hear);

    // ---------------- 7. Measurement ----------------
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                                  MakeCallback(&AppTxCounter));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/DroppedMpdu",
                    MakeCallback(&MacDrop));
    // ARP drops (FailSafe: if a path does not exist in this ns-3 version, it is skipped, not fatal)
    bool okArpQ = Config::ConnectWithoutContextFailSafe("/NodeList/*/$ns3::ArpL3Protocol/Drop",
                                                        MakeCallback(&ArpQueueDrop));
    bool okArpT = Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/Drop", MakeCallback(&ArpTimeoutDrop));
    if (!okArpQ || !okArpT)
    {
        std::cerr << "  (note: an ARP drop counter could not be connected: queue="
                  << okArpQ << " timeout=" << okArpT << ")" << std::endl;
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001)); // 1 ms histogram bins

    Simulator::Schedule(Seconds(5.0), &PrintProgress, 5.0);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ---------------- 8. Results ----------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    uint64_t totalTx = 0;
    uint64_t totalRx = 0;
    double totalDelay = 0.0;
    std::vector<uint64_t> binCounts; // combined delay histogram across camera flows
    double binWidth = 0.001;
    uint64_t ipNoRoute = 0, ipTtl = 0, ipQueue = 0, ipRouteErr = 0, ipOther = 0;
    uint64_t c2Rx = 0;
    double c2DelaySum = 0.0;
    std::vector<uint64_t> c2Bins;

    for (const auto& kv : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        if (t.destinationAddress == groundAddr && t.destinationPort == 9100)
        {
            const FlowMonitor::FlowStats& cs = kv.second;
            c2Rx += cs.rxPackets;
            c2DelaySum += cs.delaySum.GetSeconds();
            const Histogram& ch = cs.delayHistogram;
            if (c2Bins.size() < ch.GetNBins())
                c2Bins.resize(ch.GetNBins(), 0);
            for (uint32_t b = 0; b < ch.GetNBins(); ++b)
                c2Bins[b] += ch.GetBinCount(b);
            continue;
        }
        if (t.destinationAddress != groundAddr || t.destinationPort != port)
        {
            continue; // count only camera->ground data flows, not routing packets
        }
        const FlowMonitor::FlowStats& fs = kv.second;
        totalTx += fs.txPackets;
        totalRx += fs.rxPackets;
        totalDelay += fs.delaySum.GetSeconds();

        // IP-layer drops along the path, by reason
        for (uint32_t r = 0; r < fs.packetsDropped.size(); ++r)
        {
            uint64_t c = fs.packetsDropped[r];
            if (r == Ipv4FlowProbe::DROP_NO_ROUTE)
                ipNoRoute += c;
            else if (r == Ipv4FlowProbe::DROP_TTL_EXPIRE)
                ipTtl += c;
            else if (r == Ipv4FlowProbe::DROP_QUEUE || r == Ipv4FlowProbe::DROP_QUEUE_DISC)
                ipQueue += c;
            else if (r == Ipv4FlowProbe::DROP_ROUTE_ERROR)
                ipRouteErr += c;
            else
                ipOther += c;
        }

        const Histogram& h = fs.delayHistogram;
        if (binCounts.size() < h.GetNBins())
        {
            binCounts.resize(h.GetNBins(), 0);
        }
        for (uint32_t b = 0; b < h.GetNBins(); ++b)
        {
            binCounts[b] += h.GetBinCount(b);
        }
    }

    // Packets the cameras SHOULD have produced (the true offered load).
    double pktPerSec = DataRate(flowRate).GetBitRate() / (8.0 * 1000.0); // 1000-byte packets
    double expected = 0.0;
    for (uint32_t i = 0; i < nCam; ++i)
    {
        expected += pktPerSec * ((simTime - 1.0) - (trafficStart + 0.1 * i));
    }

    double meanDelayMs = totalRx > 0 ? 1000.0 * totalDelay / totalRx : 0.0;
    double pdr = totalTx > 0 ? 100.0 * totalRx / totalTx : 0.0;
    double e2ePdr = expected > 0 ? 100.0 * totalRx / expected : 0.0;
    double leftPct = expected > 0 ? 100.0 * totalTx / expected : 0.0;
    double noRouteSrc = expected - totalTx; // never left the source drone

    auto percentileMs = [&](double q) {
        uint64_t target = static_cast<uint64_t>(std::ceil(q * totalRx));
        uint64_t cumulative = 0;
        for (uint32_t b = 0; b < binCounts.size(); ++b)
        {
            cumulative += binCounts[b];
            if (cumulative >= target && totalRx > 0)
            {
                return 1000.0 * (b + 1) * binWidth;
            }
        }
        return 0.0;
    };
    double connPct = g_connSamples > 0 ? 100.0 * g_connHits / g_connSamples : 0.0;
    double ofPossible = connPct > 0 ? 100.0 * e2ePdr / connPct : 0.0;
    double dl = deadlineMs > 0 ? deadlineMs : 100.0; // report "within 100 ms" if no deadline set
    uint64_t inTime = 0;
    for (uint32_t b = 0; b < binCounts.size(); ++b)
    {
        if ((b + 1) * binWidth * 1000.0 <= dl + 1e-9)
            inTime += binCounts[b];
    }
    double inTimePct = expected > 0 ? 100.0 * inTime / expected : 0.0;
    // C2 statistics against the 3GPP direct-steering target (40 ms, 99.9%)
    if (g_c2Copies > 0)
    {
        // replace the FlowMonitor numbers by unique messages (first copy to arrive)
        c2Rx = g_c2Rec.firstDelay.size();
        c2DelaySum = 0.0;
        c2Bins.assign(1, 0);
        for (auto& kv : g_c2Rec.firstDelay)
        {
            c2DelaySum += kv.second;
            uint32_t b = static_cast<uint32_t>(kv.second / binWidth);
            if (b >= c2Bins.size())
                c2Bins.resize(b + 1, 0);
            ++c2Bins[b];
        }
    }
    double c2DelivPct = c2Expected > 0 ? 100.0 * c2Rx / c2Expected : 0.0;
    double c2MeanMs = c2Rx > 0 ? 1000.0 * c2DelaySum / c2Rx : 0.0;
    uint64_t c2In40 = 0, c2Cum = 0;
    double c2P99 = 0.0, c2Max = 0.0;
    for (uint32_t b = 0; b < c2Bins.size(); ++b)
    {
        if ((b + 1) * binWidth * 1000.0 <= 40.0 + 1e-9)
            c2In40 += c2Bins[b];
        c2Cum += c2Bins[b];
        if (c2P99 == 0.0 && c2Rx > 0 && c2Cum >= static_cast<uint64_t>(std::ceil(0.99 * c2Rx)))
            c2P99 = 1000.0 * (b + 1) * binWidth;
        if (c2Bins[b] > 0)
            c2Max = 1000.0 * (b + 1) * binWidth;
    }
    double c2In40Pct = c2Expected > 0 ? 100.0 * c2In40 / c2Expected : 0.0;

    // Downlink C2 (ground -> drone), measured the same way and reported separately.
    uint64_t c2DlRx = g_c2RecDl.firstDelay.size();
    double c2DlDelaySum = 0.0;
    std::vector<uint64_t> c2DlBins(1, 0);
    for (auto& kv : g_c2RecDl.firstDelay)
    {
        c2DlDelaySum += kv.second;
        uint32_t b = static_cast<uint32_t>(kv.second / binWidth);
        if (b >= c2DlBins.size())
            c2DlBins.resize(b + 1, 0);
        ++c2DlBins[b];
    }
    double c2DlDelivPct = g_c2DlExpected > 0 ? 100.0 * c2DlRx / g_c2DlExpected : 0.0;
    double c2DlMeanMs = c2DlRx > 0 ? 1000.0 * c2DlDelaySum / c2DlRx : 0.0;
    uint64_t c2DlIn40 = 0, c2DlCum = 0;
    double c2DlP99 = 0.0, c2DlMax = 0.0;
    for (uint32_t b = 0; b < c2DlBins.size(); ++b)
    {
        if ((b + 1) * binWidth * 1000.0 <= 40.0 + 1e-9)
            c2DlIn40 += c2DlBins[b];
        c2DlCum += c2DlBins[b];
        if (c2DlP99 == 0.0 && c2DlRx > 0 &&
            c2DlCum >= static_cast<uint64_t>(std::ceil(0.99 * c2DlRx)))
            c2DlP99 = 1000.0 * (b + 1) * binWidth;
        if (c2DlBins[b] > 0)
            c2DlMax = 1000.0 * (b + 1) * binWidth;
    }
    double c2DlIn40Pct = g_c2DlExpected > 0 ? 100.0 * c2DlIn40 / g_c2DlExpected : 0.0;

    double p95Ms = percentileMs(0.95);
    double p99Ms = percentileMs(0.99);
    double maxMs = 0.0;
    for (uint32_t b = 0; b < binCounts.size(); ++b)
    {
        if (binCounts[b] > 0)
        {
            maxMs = 1000.0 * (b + 1) * binWidth;
        }
    }

    std::cout << "\n===== SwarmNet-IND (" << routing << " over 802.11a, MAC = "
              << (macMode == "N1" ? "N1 altitude-banded TDMA" : "CSMA-CA baseline")
              << (arpPrefill ? ", ARP pre-filled" : "") << ") =====\n"
              << "UAVs: " << nUav << "  camera UAVs: " << nCam << "  run: " << run << "\n"
              << "Packets expected (offered load): " << expected << "\n"
              << "Packets that left the source: " << totalTx << " (" << leftPct
              << " % -- a route existed)\n"
              << "Packets received at ground: " << totalRx << "\n"
              << "End-to-end delivery (of expected): " << e2ePdr << " %\n"
              << "Network PDR (of those that left): " << pdr << " %\n"
              << "Mean delay: " << meanDelayMs << " ms\n"
              << "95th-percentile delay: " << p95Ms << " ms\n"
              << "99th-percentile delay: " << p99Ms << " ms\n"
              << "Maximum delay: " << maxMs << " ms\n"
              << "Physically connected to ground (camera drones, % of time): " << connPct << " %\n"
              << "Delivery as share of what is physically possible: " << ofPossible << " %\n"
              << "Delivered within " << dl << " ms (of expected): " << inTimePct << " %\n"
              << (nC2 > 0 ? "" : "")
              << "\n--- Command-and-control (C2) messages ---\n"
              << "C2 expected: " << c2Expected << "  received: " << c2Rx << "\n"
              << "C2 delivered (any delay): " << c2DelivPct << " %\n"
              << "C2 delivered within 40 ms (3GPP target 99.9 %): " << c2In40Pct << " %\n"
              << "C2 mean / 99th-percentile / max delay: " << c2MeanMs << " / " << c2P99 << " / "
              << c2Max << " ms\n"
              << "C2 discarded as too old: " << g_c2Stale << "\n"
              << "C2 copies sent per message: " << (g_c2Copies == 2 ? 2 : 1)
              << "   duplicate copies that arrived (ignored): " << g_c2Rec.dupes << "\n"
              << "C2 own relay chain (short links): " << g_c2Route << " (max link "
              << g_c2SafeFrac * g_n2Hear << " m)\n"
              << "--- Where were C2 messages lost? ---\n"
              << "C2 not sent, no route at source:        " << g_c2SrcFail << "\n"
              << "C2 held with no relay, then too old:     " << g_c2Stale << "\n"
              << "C2 Wi-Fi gave up, receiver IN range:     " << g_c2RetryNear << "\n"
              << "C2 Wi-Fi gave up, receiver OUT of range: " << g_c2RetryFar << "\n"
              << "   ...receiver <300 m: " << g_c2Bin[0] << "   300-500 m: " << g_c2Bin[1]
              << "   500-650 m: " << g_c2Bin[2] << "   >650 m: " << g_c2Bin[3] << "\n"
              << "C2 waited too long in Wi-Fi queue:       " << g_c2MacExpired << "\n"
              << "C2 Wi-Fi queue full:                     " << g_c2MacFull << "\n"
              << "\n--- Why were packets lost? ---\n"
              << "No route at the source drone:            " << noRouteSrc << "\n"
              << "Dropped at IP layer - no route en route: " << ipNoRoute << "\n"
              << "Dropped at IP layer - TTL expired:       " << ipTtl << "\n"
              << "Dropped at IP layer - queue full:        " << ipQueue << "\n"
              << "Dropped at IP layer - route error:       " << ipRouteErr << "\n"
              << "Dropped at IP layer - other:             " << ipOther << "\n"
              << "Wi-Fi gave up, receiver IN range  (collision/interference): " << g_macRetryNear << "\n"
              << "Wi-Fi gave up, receiver OUT of range (link broken by move): " << g_macRetryFar << "\n"
              << "   ...of which receiver was  <300 m: " << g_dropBin[0]
              << "   300-500 m: " << g_dropBin[1] << "   500-650 m: " << g_dropBin[2]
              << "   >650 m: " << g_dropBin[3] << "\n"
              << "   (drops at short distance = collisions; drops near the edge = weak links)\n"
              << "Wi-Fi queue full:                        " << g_macQueueFull << "\n"
              << "Wi-Fi packet waited too long:            " << g_macExpired << "\n"
              << "ARP waiting queue full (neighbour discovery): " << g_arpQueueDrop << "\n"
              << "ARP gave up, neighbour never answered:        " << g_arpTimeoutDrop << "\n"
              << "(Wi-Fi counts are per hop, so they can exceed end-to-end losses.)\n";
    if (g_c2Downlink)
    {
        std::cout << "\n--- Downlink C2 (ground station -> drone) ---\n"
                  << "Destination position used: " << c2DlPos
                  << (g_c2DlIdealPos ? " (exact; optimistic upper bound)"
                                     : " (fix carried by the drone's own uplink report)")
                  << "\n"
                  << "Downlink C2 expected: " << g_c2DlExpected << "  received: " << c2DlRx << "\n"
                  << "Downlink C2 delivered (any delay): " << c2DlDelivPct << " %\n"
                  << "Downlink C2 within 40 ms (3GPP target 99.9 %): " << c2DlIn40Pct << " %\n"
                  << "Downlink C2 mean / 99th-percentile / max delay: " << c2DlMeanMs << " / "
                  << c2DlP99 << " / " << c2DlMax << " ms\n"
                  << "Downlink not sent, no route at the ground:  " << g_c2DlSrcFail << "\n"
                  << "Downlink held with no relay, then too old:  " << g_c2StaleDl << "\n"
                  << "Route not computed, destination fix missing or stale: " << g_c2DlNoPos << "\n"
                  << "Route computed with no safe relay (fallback used):    " << g_c2DlNoRelay
                  << "\n"
                  << "Downlink next-hop switches:                 " << g_c2DlSwitches << "\n"
                  << "(The per-hop Wi-Fi C2 counters above pool BOTH directions: a dropped\n"
                  << " frame is counted by size, which is the same in each direction.)\n";
    }
    if (routing == "N2" || routing == "PRED")
    {
        std::cout << (g_predMode ? "\n--- PRED baseline behaviour ---\n"
                                 : "\n--- N2 behaviour ---\n")
                  << "Next-hop switches (make-before-break): " << g_n2Switches << "\n"
                  << "Times no SAFE neighbour existed:       " << g_n2Fallback << "\n"
                  << "Times no neighbour at all (data held):  " << g_n2NoNeighbour << "\n"
                  << "Data discarded as too old (deadline):  " << g_n2DeadlineDrops << "\n";
        if (g_predMode)
        {
            std::cout << "Routes kept although nothing qualified: " << g_predStaleKeep
                      << "   (LET threshold " << g_letThreshS << " s, hold "
                      << (g_predHold ? "on" : "off") << ", late binding off)\n";
        }
    }

    std::ofstream csv("swarm-baseline.csv", std::ios::app);
    csv << routing << "," << nUav << "," << nCam << "," << run << "," << areaSize << ","
        << (arpPrefill ? 1 : 0) << ","
        << expected << "," << totalTx << "," << totalRx << "," << e2ePdr << "," << pdr << ","
        << meanDelayMs << "," << p95Ms << "," << p99Ms << "," << maxMs << ","
        << noRouteSrc << "," << ipNoRoute << "," << ipTtl << "," << ipQueue << ","
        << ipRouteErr << "," << ipOther << ","
        << g_macRetryNear << "," << g_macRetryFar << "," << g_macQueueFull << "," << g_macExpired
        << "," << g_arpQueueDrop << "," << g_arpTimeoutDrop << "," << macMode << ","
        << (macMode == "N1" ? bands : 0) << "," << (macMode == "N1" ? cols : 0) << ","
        << (macMode == "N1" ? slotMs : 0.0) << "," << g_dropBin[0] << "," << g_dropBin[1] << ","
        << g_dropBin[2] << "," << g_dropBin[3] << "," << n2Info << "," << g_n2BeaconLoss << ","
        << g_n2GpsErr << "," << deadlineMs << "," << (n2Hold ? 1 : 0) << "," << connPct << ","
        << ofPossible << "," << inTimePct << "," << g_n2DeadlineDrops << "," << (g_n1Zone ? 1 : 0)
        << "," << n1SfMs << "," << g_n1AFrac << "," << nC2 << "," << (g_c2Prio ? 1 : 0) << ","
        << g_n1C2 << "," << c2WinMs << "," << c2Expected << "," << c2Rx << ","
        << c2DelivPct << "," << c2In40Pct << "," << c2MeanMs << "," << c2P99 << "," << c2Max << "," << g_c2Copies << ","
        << g_c2Rec.dupes << "," << g_c2Route << "," << g_c2SafeFrac << "," << g_c2SrcFail << ","
        << g_c2Stale << "," << g_c2RetryNear << "," << g_c2RetryFar << "," << g_c2MacExpired
        // ---- version 12 columns, appended so CSV columns 1-67 keep their old meaning ----
        // (the sweep scripts prepend a tag, so awk field number = CSV column + 1)
        << "," << (g_c2Downlink ? 1 : 0)   // 68 downlink on
        << "," << c2DlPos                  // 69 destination position mode
        << "," << g_c2DlExpected           // 70 downlink C2 expected
        << "," << c2DlRx                   // 71 downlink C2 received
        << "," << c2DlDelivPct             // 72 downlink delivered, any delay, %
        << "," << c2DlIn40Pct              // 73 downlink within 40 ms, %
        << "," << c2DlMeanMs               // 74 downlink mean delay, ms
        << "," << c2DlP99                  // 75 downlink 99th-percentile delay, ms
        << "," << c2DlMax                  // 76 downlink max delay, ms
        << "," << g_c2DlSrcFail            // 77 not sent, no route at the ground
        << "," << g_c2StaleDl              // 78 held, then discarded as too old
        << "," << g_c2DlNoPos              // 79 destination fix missing or stale
        << "," << g_c2DlNoRelay            // 80 no safe relay (fallback used)
        << "," << g_c2DlSwitches           // 81 downlink next-hop switches
        // ---- version 13 columns (PRED baseline) ----
        << "," << (g_predMode ? 1 : 0)     // 82 PRED baseline
        << "," << g_letThreshS             // 83 PRED link-expiration threshold, s
        << "," << (g_predHold ? 1 : 0)     // 84 PRED with hold + deadline
        << "," << g_predStaleKeep          // 85 routes kept although nothing qualified
        << "\n";
    csv.close();

    Simulator::Destroy();
    return 0;
}
