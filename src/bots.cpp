#include "bots.h"

BotNode::BotNode(const PSX::NavFrame& frame)
{
    m_pos = ConvertPSXVec3(frame.pos, FP_ONE_GEO);
    m_rot.x = ConvertBotAngle(frame.rot[0]);
    m_rot.y = ConvertBotAngle(frame.rot[1]);
    m_rot.z = ConvertBotAngle(frame.rot[2]);
    m_terrain = (frame.flags & PSXBotNodeFlags::TERRAIN_MASK) >> 3;
    m_pathChangeIndex = static_cast<int>(frame.pathChangeOpCode & 0x3FF) ;
    m_pathChange = static_cast<int>(frame.pathChangeOpCode >> 10);
    m_checkpoint = frame.goBackCount;

    uint16_t flags = frame.flags;
    if (flags & PSXBotNodeFlags::TURBO_PAD_HIGH) m_flags.turboPad = true;
    if (flags & PSXBotNodeFlags::SKIDMARKS_FRONT) m_flags.skidmarkFront = true;
    if (flags & PSXBotNodeFlags::SKIDMARKS_BACK) m_flags.skidmarkBack = true;
    if (flags & PSXBotNodeFlags::TURBO_PAD_LOW) m_flags.turboPadLow = true;
    if (flags & PSXBotNodeFlags::MASK_GRAB_STP) m_flags.maskGrabSTP = true;
    if (flags & PSXBotNodeFlags::JUMP) m_flags.jump = true;
    if (flags & PSXBotNodeFlags::DRIFT_LEFT) m_flags.driftLeft = true;
    if (flags & PSXBotNodeFlags::DRIFT_RIGHT) m_flags.driftRight = true;
    if (flags & PSXBotNodeFlags::ENGINE_ECHO) m_flags.echo = true;
    if (flags & PSXBotNodeFlags::MID_AIR) m_flags.midAir = true;
    if (flags & PSXBotNodeFlags::SINK_KART) m_flags.sink = true;

    uint8_t flags2 = frame.specialBits;
    if (flags2 & PSXBotNodeFlags2::MOON_GRAV) m_flags.lowGrav = true;

    if (flags2 & PSXBotNodeFlags2::USE_RAMPHYS) 
    {
        m_ramPhysID = flags2 & PSXBotNodeFlags2::SPECIAL_MASK;
        m_specialBits = BotSpecialBits::RAM_PHYS;
    }
    else if (flags2 & PSXBotNodeFlags2::USE_REFLECTION) 
    {
        m_splitLineID = flags2 & PSXBotNodeFlags2::SPECIAL_MASK;
        m_specialBits = BotSpecialBits::REFLECTION;
    }
    else 
    {
        m_transparency = flags2 & PSXBotNodeFlags2::SPECIAL_MASK;
        m_specialBits = BotSpecialBits::TRANSPARENCY;
    }
}


std::vector<uint8_t> BotNode::Serialize(const Vec3& nextPos, std::vector<Instance>& instances) const
{
    PSX::NavFrame frame = {};
    std::vector<uint8_t> buffer(sizeof(frame));
    frame.pos = ConvertVec3(m_pos, FP_ONE_GEO);
    frame.rot[0] = ConvertBotAngle(m_rot.x);
    frame.rot[1] = ConvertBotAngle(m_rot.y);
    frame.rot[2] = ConvertBotAngle(m_rot.z);
    frame.rot[3] = -frame.rot[0]; // Not sure what this is
    frame.distXYZ = ConvertFloat((m_pos - nextPos).Length(), FP_ONE_GEO);
    frame.distXZ = ConvertFloat((m_pos - nextPos).LengthHorizontal(), FP_ONE_GEO);
    frame.pathChangeOpCode = (static_cast<uint16_t>(m_pathChange) << 10) | (static_cast<uint16_t>(m_pathChangeIndex));
    frame.goBackCount = m_checkpoint;

    if (m_flags.turboPad) frame.flags |= PSXBotNodeFlags::TURBO_PAD_HIGH;
    if (m_flags.skidmarkFront) frame.flags |= PSXBotNodeFlags::SKIDMARKS_FRONT;
    if (m_flags.skidmarkBack) frame.flags |= PSXBotNodeFlags::SKIDMARKS_BACK;
    if (m_flags.turboPadLow) frame.flags |= PSXBotNodeFlags::TURBO_PAD_LOW;
    if (m_flags.maskGrabSTP) frame.flags |= PSXBotNodeFlags::MASK_GRAB_STP;
    if (m_flags.jump) frame.flags |= PSXBotNodeFlags::JUMP;
    if (m_flags.driftLeft) frame.flags |= PSXBotNodeFlags::DRIFT_LEFT;
    if (m_flags.driftRight) frame.flags |= PSXBotNodeFlags::DRIFT_RIGHT;
    if (m_flags.echo) frame.flags |= PSXBotNodeFlags::ENGINE_ECHO;
    if (m_flags.midAir) frame.flags |= PSXBotNodeFlags::MID_AIR;
    if (m_flags.sink) frame.flags |= PSXBotNodeFlags::SINK_KART;
    frame.flags &= ~PSXBotNodeFlags::TERRAIN_MASK;
    frame.flags |= (static_cast<uint16_t>(m_terrain) << 3) & PSXBotNodeFlags::TERRAIN_MASK;

    if (m_flags.lowGrav) frame.specialBits |= PSXBotNodeFlags2::MOON_GRAV;
    if (m_specialBits == BotSpecialBits::RAM_PHYS)
        frame.specialBits |= static_cast<uint8_t>(m_ramPhysID) & PSXBotNodeFlags2::SPECIAL_MASK;
    else if (m_specialBits == BotSpecialBits::REFLECTION)
        frame.specialBits |= static_cast<uint8_t>(m_splitLineID) & PSXBotNodeFlags2::SPECIAL_MASK;
    else
        frame.specialBits |= static_cast<uint8_t>(m_transparency) & PSXBotNodeFlags2::SPECIAL_MASK;

    for (Instance& inst : instances)
    {
        if (inst.GetHitbox().enabled)
        {
            BoundingBox bbox = inst.ComputeBBox();
            if (bbox.Distance(m_pos) < EPSILON)
                frame.specialBits |= PSXBotNodeFlags2::INSTANCE_COLL;
        }
    }

    std::memcpy(buffer.data(), &frame, sizeof(frame));
    return buffer;
}

BotPath::BotPath(const PSX::NavHeader& header, const std::vector<PSX::NavFrame>& frames)
{
    //m_offLastPoint = header.offLastPoint;
    std::copy(std::begin(header.physUnk), std::end(header.physUnk), std::begin(m_physUnk));

    m_nodes.reserve(frames.size());
    for (const auto& frame : frames)
    {
        m_nodes.emplace_back(frame);
    }   
}

void BotPath::Clear()
{
    m_nodes.clear();
}

bool BotPath::IsValid()
{
    return m_nodes.size() > 1;
}


bool BotPath::GeneratePath(std::vector<Vec3>& nodesPos, std::vector<Quadblock>& quadblocks)
{
    m_nodes.clear();
    if (nodesPos.empty()) { return false; }


    const size_t nodeCount = nodesPos.size();
    constexpr float GROUND_THRESHOLD = 8.0f;
    constexpr float NEARBY_THRESHOLD = 2.0f;
    constexpr float REVERB_THRESHOLD = 50.0f; // broader search for reverb, not just ground
    constexpr float SHARP_TURN_THRESHOLD = 15.0f; // degrees, for drift detection
    constexpr float DRIFT_BONUS_YAW = 45; // degrees, for drift yaw addition
    constexpr float SKIDMARK_LENGTH = 15.0f; // degrees, for drift yaw addition
    constexpr float BOT_SPEED = 25.0f;
    constexpr float SHARP_TURN_CIRCLE_SECONDS = 10.0f;
    constexpr float SHARP_TURN_DEG_PER_UNIT = 360.0f / (BOT_SPEED * SHARP_TURN_CIRCLE_SECONDS); // 2.4 deg/unit
    constexpr float DRIFT_MIN_DISTANCE = 25.0f;   // at least 2s worth of distance
    constexpr int   DRIFT_ANTICIPATION_NODES = 2; // start drift this many nodes before the sharp section

    
    // --- Helper: any nearby quad matching a predicate ---
    auto HasNearbyQuad = [&](const Vec3& pos, float radius,
        const std::function<bool(const Quadblock&)>& predicate) -> bool
        {
            for (const Quadblock& quad : quadblocks)
            {
                const BoundingBox& bb = quad.GetBoundingBox();
                // Closest point on the AABB to pos � clamp each axis independently
                float cx = std::clamp(pos.x, bb.min.x, bb.max.x);
                float cy = std::clamp(pos.y, bb.min.y, bb.max.y);
                float cz = std::clamp(pos.z, bb.min.z, bb.max.z);
                float dx = pos.x - cx;
                float dy = pos.y - cy;
                float dz = pos.z - cz;
                float distSq = dx * dx + dy * dy + dz * dz;
                if (distSq <= radius * radius && predicate(quad)) { return true; }
            }
            return false;
        };

    //Helper : return an angle in between -180 and 180, modulo 360
    auto NormalizeAngle = [](float angle)
        {
            angle = std::fmod(angle + 180.0f, 360.0f);
            if (angle < 0)
                angle += 360.0f;
            return angle - 180.0f;
        };

    m_nodes.resize(nodeCount);
    const std::vector<Vec3> nodesRot = ComputeYaw(nodesPos, true);

    // Pass 1 : Detect AirTime + Snap to Ground + construct up vec list
    std::vector<const Quadblock*> groundQuads(nodeCount);
    std::vector<bool> grounded(nodeCount);
    std::vector<Vec3> upVec(nodeCount);
    std::vector<Vec3> forwardVec(nodeCount);
    std::vector<float> segmentDist(nodeCount);
    const Vec3 upGlobal = Vec3(0.0f, 1.0f, 0.0f);
    for (size_t i = 0; i < nodeCount; i++)
    {
        Vec3 pos = nodesPos[i];
        Vec3 rot = nodesRot[i];
        float bestDist = GROUND_THRESHOLD;

        grounded[i] = false;
        upVec[i] = { 0.0f, 1.0f, 0.0f };
        for (const Quadblock& quad : quadblocks)
        {
            if (!(quad.GetFlags() & QuadFlags::GROUND))
                continue;

            const BoundingBox& bb = quad.GetBoundingBox();
            if (pos.x < bb.min.x || pos.x > bb.max.x) continue;
            if (pos.z < bb.min.z || pos.z > bb.max.z) continue;

            float dist = 0.0f;
            Vec3 normal;
            if (!quad.IntersectRay(pos, upGlobal, dist, normal))
                continue;
            if (std::abs(dist) > bestDist)
                continue;
            bestDist = std::abs(dist);
            quad.SnapPoint(pos, rot, upGlobal);
            groundQuads[i] = &quad;
            upVec[i] = normal;
            grounded[i] = true;
        }
            
        m_nodes[i].SetPos(pos);
        m_nodes[i].SetRot(rot);
    }

    /// --- Pre-pass: compute yaw ---
    std::vector<float> yaws(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        const Vec3& curr = nodesPos[i];
        const Vec3& next = nodesPos[(i + 1) % nodeCount];
        const Vec3 delta = next - curr;
        forwardVec[i] = delta - upVec[i] * (upVec[i].Dot(delta));
        forwardVec[i].Normalize();
        yaws[i] = std::atan2(delta.x, delta.z) * (180.0f / MATH_PI);
        segmentDist[i] = delta.Length();
    }


    // --- Pre-pass: drift ---
    std::vector<float> angularVel(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        float dist = segmentDist[i];
        if (dist < 1e-6f) { angularVel[i] = 0.0f; continue; }
        float yawDelta = NormalizeAngle(yaws[(i + 1) % nodeCount] - yaws[i]);
        angularVel[i] = yawDelta / dist; // degrees per unit of distance
    }

    std::vector<int> driftDir(nodeCount, 0); // -1 = right, 0 = none, +1 = left
    for (size_t i = 0; i < nodeCount; i++)
    {
        if (!grounded[i]) { continue; }
        if (std::abs(angularVel[i]) >= SHARP_TURN_DEG_PER_UNIT)
            driftDir[i] = (angularVel[i] > 0.0f) ? 1 : -1;
    }

    struct DriftChunk {
        size_t start, end; // inclusive indices
        int dir;           // -1 or +1
    };

    // Helper: sum of segmentDist[start..end] inclusive
    auto chunkDist = [&](size_t start, size_t end) {
        float d = 0.f;
        for (size_t i = start; i <= end; i++) d += segmentDist[i];
        return d;
        };

    // Helper: distance of the gap between two chunks (exclusive indices between them)
    auto gapDist = [&](const DriftChunk& a, const DriftChunk& b) {
        float d = 0.f;
        for (size_t i = a.end + 1; i < b.start; i++) d += segmentDist[i];
        return d;
        };

    // Helper: rebuild DriftChunk list from current driftDir array
    auto buildChunks = [&]() {
        std::vector<DriftChunk> chunks;
        size_t i = 0;
        while (i < nodeCount) {
            if (driftDir[i] != 0) {
                size_t s = i;
                while (i < nodeCount && driftDir[i] == driftDir[s]) i++;
                chunks.push_back({ s, i - 1, driftDir[s] });
            }
            else {
                i++;
            }
        }
        return chunks;
        };

    const float MIN_DRIFT_DIST = 15.0f;
    const float MIN_GAP_DIST = 15.0f;

    // --- Step 1: Merge same-direction chunks that are too close ---
    auto chunks = buildChunks();
    for (size_t i = 0; i + 1 < chunks.size(); i++) 
    {
        auto& a = chunks[i];
        auto& b = chunks[i + 1];
        if (a.dir == b.dir && gapDist(a, b) < MIN_GAP_DIST) 
        {
            for (size_t j = a.end + 1; j < b.start; j++) 
                driftDir[j] = a.dir;
        }
    }

    // --- Step 2: Remove drift chunks shorter than MIN_DRIFT_DIST ---
    chunks = buildChunks();
    for (auto& c : chunks) {
        if (chunkDist(c.start, c.end) < MIN_DRIFT_DIST) {
            for (size_t i = c.start; i <= c.end; i++) driftDir[i] = 0;
        }
    }



    // --- Step 3: Trim first chunk when different-direction chunks are too close ---
    bool changed = true;
    while (changed) {
        changed = false;
        chunks = buildChunks();
        for (size_t i = 0; i + 1 < chunks.size(); i++) {
            auto& a = chunks[i];
            auto& b = chunks[i + 1];
            if (a.dir != b.dir && gapDist(a, b) < MIN_GAP_DIST) {
                float deficit = MIN_GAP_DIST - gapDist(a, b);
                // Trim from the end of chunk A, node by node
                size_t j = a.end;
                float trimmed = 0.f;
                while (j >= a.start && trimmed < deficit) {
                    trimmed += segmentDist[j];
                    driftDir[j] = 0;
                    if (j == 0) break;
                    j--;
                }
                changed = true;
                break;
            }
        }
    }

    // --- Step 4: After trimming, some chunks may now be too short � repeat step 2 ---
    chunks = buildChunks();
    for (auto& c : chunks) {
        if (chunkDist(c.start, c.end) < MIN_DRIFT_DIST) {
            for (size_t i = c.start; i <= c.end; i++) driftDir[i] = 0;
        }
    }




    const float DRIFT_ANGLE_DEG = 30.0f;
    const float DRIFT_ANGLE_RAD = DRIFT_ANGLE_DEG * (MATH_PI / 180.0f);
    for (size_t i = 0; i < nodeCount; i++)
    {
        //Rotate forward to simulate drift.
        /*Vec3& forward = forwardVec[i];
        Vec3& up = upVec[i];
        float angle = driftDir[i] * DRIFT_ANGLE_RAD;
        forwardVec[i] = forward * std::cos(angle) + (up.Cross(forward)) * std::sin(angle);*/

        if (driftDir[i] == 1)
        {
            BotFlags flags{};
            flags.driftRight = true;
            m_nodes[i].SetFlags(flags);
        }
        if (driftDir[i] == -1)
        {
            BotFlags flags{};
            flags.driftRight = true;
            m_nodes[i].SetFlags(flags);
        }
    }

    uint8_t lastckpt = 0;
    for (size_t i = 0; i < nodeCount; i++)
    {
        BotNode& node = m_nodes[i];
        node.SetPathChange(3); // no path change
        node.SetPathChangeIndex(static_cast<int>((i + 4) % nodeCount));
        const Quadblock* groundQuad = groundQuads[i];
        bool isGrounded = grounded[i];

        // --- Rotation ---
        Vec3& forward = forwardVec[i];
        Vec3& up = upVec[i];
        Vec3 right = forward.Cross(up);
        
        // --- Terrain & go back count from ground quad ---
        
        if (groundQuad)
        {
            int cur_ckpt = groundQuad->GetCheckpoint();
            if (cur_ckpt >=  0) { lastckpt = static_cast<uint8_t>(std::clamp(cur_ckpt, 0, 255)); }
            node.SetCheckpoint(lastckpt);
            // Terrain from the quad directly underfoot
            node.SetTerrain(groundQuad->GetTerrain());

        }
        else
        {
            node.SetCheckpoint(lastckpt);
            node.SetTerrain(TerrainType::ASPHALT);
        }

        // --- Flags ---

        const Vec3& pos = nodesPos[i];
        BotFlags flags = node.GetFlags();

        // MID_AIR: not grounded
        if (!isGrounded)
        {
            flags.midAir = true;
        }

        // JUMP: last grounded node before becoming airborne
        if (isGrounded)
        {
            bool nextAirborne = !grounded[(i + 1) % nodeCount];
            if (nextAirborne) { flags.jump = true; }
        }

        // SINK_KART: ground quad has water / fast water / mud terrain
        if (groundQuad)
        {
            uint8_t t = groundQuad->GetTerrain();
            if (t == TerrainType::WATER ||
                t == TerrainType::FAST_WATER ||
                t == TerrainType::MUD)
            {
                flags.sink = true;
            }
        }

        //ECHO 
        if (groundQuad)
        {
            if (groundQuad->GetFlags() & QuadFlags::REVERB)
                flags.echo = true;
        }

        //MOON GRAV 
        if (groundQuad)
        {
            if (groundQuad->GetFlags() & QuadFlags::MOON_GRAVITY)
                flags.lowGrav = true;
        }

        // TURBO_PAD_HIGH: nearby quad with TRIGGER_SCRIPT and Dirt terrain
        bool onTurboPad = HasNearbyQuad(pos, NEARBY_THRESHOLD, [](const Quadblock& q)
            {
                return (q.GetFlags() & QuadFlags::TRIGGER_SCRIPT) &&
                    (q.GetTerrain() == TerrainType::DIRT);
            });
        if (onTurboPad) { flags.turboPad = true; }

        // TURBO_PAD_LOW: nearby quad with TRIGGER_SCRIPT and Grass terrain (super turbo pad)
        bool onSuperTurboPad = HasNearbyQuad(pos, NEARBY_THRESHOLD, [](const Quadblock& q)
            {
                return (q.GetFlags() & QuadFlags::TRIGGER_SCRIPT) &&
                    (q.GetTerrain() == TerrainType::GRASS);
            });
        if (onSuperTurboPad) { flags.turboPadLow = true; }

        // SKIDMARKS_BACK: when drifting
        bool drifting = flags.driftLeft || flags.driftRight;
        if (drifting) { flags.skidmarkBack = true; }

        // SKIDMARKS_FRONT: drifting, or on a turbo/super turbo pad,
        // or the ~10 units after landing (transitioning from air to ground)
        bool prevAirborne = !grounded[(i + nodeCount - 1) % nodeCount];
        bool justLanded = isGrounded && prevAirborne;

        // Count how many nodes ago we landed to cover the ~10 unit window
        bool withinLandingWindow = false;
        if (isGrounded)
        {
            float distSinceLanding = 0.0f;
            for (size_t k = 1; k < nodeCount && distSinceLanding < SKIDMARK_LENGTH; k++)
            {
                size_t idx = (i + nodeCount - k) % nodeCount;
                if (!grounded[idx] || (m_nodes[idx].GetFlags().turboPad || m_nodes[idx].GetFlags().turboPadLow)) { withinLandingWindow = true; break; }
                size_t idxNext = (idx + 1) % nodeCount;
                Vec3 d = {
                    nodesPos[idxNext].x - nodesPos[idx].x,
                    nodesPos[idxNext].y - nodesPos[idx].y,
                    nodesPos[idxNext].z - nodesPos[idx].z
                };
                distSinceLanding += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            }
        }

        if (drifting || onTurboPad || onSuperTurboPad || withinLandingWindow)
        {
            flags.skidmarkFront = true;
        }

        node.SetFlags(flags);
    }
    return true;
}




std::vector<uint8_t> BotPath::Serialize(std::vector<Instance>& instances) const
{
    // Crash if called with invalid nodes. Never serialize empty path.
    PSX::NavHeader header = {};
    std::vector<uint8_t> buffer(sizeof(header));
    header.magic = BOT_PATH_MAGIC;
    header.numPoints = static_cast<uint16_t>(m_nodes.size()-1);
    header.unk1 = 0;
    header.posY = ConvertFloat(m_nodes[0].GetPos().y, FP_ONE_GEO);
    header.offLastPoint = 0;//m_offLastPoint;
    std::copy(std::begin(m_physUnk), std::end(m_physUnk), std::begin(header.physUnk)); // can't be removed (on crash cove, ramp fails), need to be understood
    std::memcpy(buffer.data(), &header, sizeof(header));

    for (int i = 0; i < m_nodes.size() - 1 ; i++)
    {
        int next_id = i == (m_nodes.size() - 2) ? 0 : i + 1; //2nd to last's next is the first. Last is handled differently
        const BotNode& node = m_nodes[i];
        const Vec3& nextPos = m_nodes[next_id].GetPos();
        auto nodeBytes = node.Serialize(nextPos, instances);
        buffer.insert(buffer.end(), nodeBytes.begin(), nodeBytes.end());
    }
    //Placeholder behavior for the last. Need to investigate how it works. It doesn't seem to be the distance to first.
    const BotNode& node = m_nodes[m_nodes.size() - 1];
    const Vec3& nextPos = m_nodes[0].GetPos();
    auto nodeBytes = node.Serialize(nextPos, instances);
    buffer.insert(buffer.end(), nodeBytes.begin(), nodeBytes.end());
    return buffer;
}


std::vector<Vec3> GenerateLateralPath(const std::vector<BotNode>& nodes, float lateralOffset, std::vector<Quadblock>& quadblocks)
{
    if (nodes.size() < 2)
    {
        std::vector<Vec3> fallback;
        fallback.reserve(nodes.size());
        for (const BotNode& node : nodes)
            fallback.push_back(node.GetPos());
        return fallback;
    }

    float sign = lateralOffset < 0 ? -1.0f : 1.0f;
    const Vec3 up(0.0f, 1.0f, 0.0f);

    // Check if a given XZ position falls within the XZ bounds of any quadblock with the given checkpoint ID
    auto isAboveAnyQuadblock = [&](const Vec3& testPos, int checkpointID, float& height) -> bool
        {
            for (const Quadblock& quad : quadblocks)
            {
                if (quad.GetCheckpoint() > checkpointID + 1 || quad.GetCheckpoint() < checkpointID - 1)
                    continue;
                Vec3 _;
                if (quad.IntersectRay(testPos, up, height, _))
                    return true;
            }
            return false;
        };

    std::vector<Vec3> result;
    result.reserve(nodes.size());

    float currLateralOffset = lateralOffset;
    constexpr float reductionFactor = 0.8f;
    constexpr int   maxAttempts = 10;

    for (int i = 0; i < nodes.size(); i++)
    {
        const Vec3 nodePos = nodes[i].GetPos();
        const int  checkpointID = static_cast<int>(nodes[i].GetCheckpoint());

        Vec3 forward = nodes[(i == nodes.size() - 1) ? 0 : i + 1].GetPos() - nodes[i].GetPos();
        forward.Normalize();

        Vec3 right = forward.Cross(up);
        right.Normalize();
        if (right.Length() < EPSILON)
            right = Vec3(1.0f, 0.0f, 0.0f);
        float _ = 0.0f;
        if (!isAboveAnyQuadblock(nodePos, checkpointID, _))
        {
            result.push_back(nodePos + right * currLateralOffset);
            continue;
        }

        float tempLateralOffset = sign * std::fmin(std::abs(currLateralOffset) / reductionFactor, std::abs(lateralOffset));
        Vec3 candidatePos;
        float target_height = 0.0f;

        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            candidatePos = nodePos + right * tempLateralOffset;
            if (isAboveAnyQuadblock(candidatePos, checkpointID, target_height))
            {
                currLateralOffset = tempLateralOffset;
                //candidatePos.y = target_height;
                break;
            }   
            tempLateralOffset *= reductionFactor;
        }
        result.push_back(candidatePos);
    }
    return result;
}
