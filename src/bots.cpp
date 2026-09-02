#include "bots.h"
#include "settings.h"


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
        m_shadow = flags2 & PSXBotNodeFlags2::SPECIAL_MASK;
        m_specialBits = BotSpecialBits::SHADOW;
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
        frame.specialBits |= static_cast<uint8_t>(m_shadow) & PSXBotNodeFlags2::SPECIAL_MASK;

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


bool BotPath::GeneratePath(std::vector<Vec3>& nodesPos, const std::vector<Quadblock>& quadblocks, int pathID)
{
    std::vector<size_t> groundQuadIndexes;
    for (size_t i = 0; i < quadblocks.size(); i++)
    {
        if (quadblocks[i].GetFlags() & QuadFlags::GROUND)
            groundQuadIndexes.push_back(i);
    }
    const Vec3 upGlobal = Vec3(0.0f, 1.0f, 0.0f);



    if (BotPathSettings::normalizeNodeDist)
        nodesPos = NormalizePos(nodesPos, BotPathSettings::nodeDistance, true);

    if (!BotPathSettings::useManualPath)
    {
        const size_t nodeCount = nodesPos.size();
        std::vector<Vec3> lateralPos;
        std::vector<Vec3> nodesRot = ComputeYaw(nodesPos, true);
        float lateralOffset = BotPathSettings::sidewayOffset * (pathID - 1);
        float currLateralOffset = lateralOffset;
        for (size_t i = 0; i < nodeCount; i++)
        {
            int quadID = SnapToClosestQuad(quadblocks, groundQuadIndexes, nodesPos[i], nodesRot[i], upGlobal, BotPathSettings::negSnapDist, BotPathSettings::posSnapDist);
            Vec3 forward = nodesPos[(i + 1) % nodeCount] - nodesPos[i];
            forward.Normalize();
            Vec3 right = forward.Cross(upGlobal);
            right.Normalize();
            Vec3 currPos = nodesPos[i] + right * currLateralOffset;
            if (quadID != -1)
            {
                int k = 0;
                while (-1 == SnapToClosestQuad(quadblocks, groundQuadIndexes, currPos, nodesRot[i], upGlobal, BotPathSettings::negSnapDist, BotPathSettings::posSnapDist) && k < 10)
                {
                    k++;
                    currLateralOffset *= 0.85f;
                    currPos = nodesPos[i] + right * currLateralOffset;
                }
            }
            lateralPos.push_back(currPos);
            if (currLateralOffset * currLateralOffset < lateralOffset * lateralOffset)
                currLateralOffset /= 0.85f;
        }
        nodesPos = NormalizePos(lateralPos, BotPathSettings::nodeDistance, true);
    }


    const size_t nodeCount = nodesPos.size();
    constexpr float SKIDMARK_LENGTH = 15.0f; // degrees, for drift yaw addition
    constexpr float BOT_SPEED = 25.0f;
    constexpr float SHARP_TURN_CIRCLE_SECONDS = 10.0f;
    constexpr float SHARP_TURN_DEG_PER_UNIT = 360.0f / (BOT_SPEED * SHARP_TURN_CIRCLE_SECONDS); // 2.4 deg/unit


    //Helper : return an angle in between -180 and 180, modulo 360
    auto NormalizeAngle = [](float angle)
        {
            angle = std::fmod(angle + 180.0f, 360.0f);
            if (angle < 0)
                angle += 360.0f;
            return angle - 180.0f;
        };

    m_nodes.clear();
    if (nodesPos.empty()) { return false; }
    m_nodes.resize(nodeCount);
    const std::vector<Vec3> nodesRot = ComputeYaw(nodesPos, true);

    // Pass 1 : Detect AirTime + Snap to Ground
    std::vector<const Quadblock*> groundQuads(nodeCount);
    
    for (size_t i = 0; i < nodeCount; i++)
    {
        Vec3 pos = nodesPos[i];
        Vec3 rot = nodesRot[i];
        int quadID = SnapToClosestQuad(quadblocks, groundQuadIndexes, pos, rot, upGlobal, BotPathSettings::negSnapDist, BotPathSettings::posSnapDist);
        groundQuads[i] = quadID == -1 ? nullptr : groundQuads[i] = &quadblocks[quadID];
        m_nodes[i].SetPos(pos);
        m_nodes[i].SetRot(rot);          
    }
    
    // Pass : Distance + AngularVel
    std::vector<float> segmentDist(nodeCount);
    std::vector<float> angularVel(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        const BotNode& curr = m_nodes[i];
        const BotNode& next = m_nodes[(i + 1) % nodeCount];
        float dist = (next.GetPos() - curr.GetPos()).Length();
        float yawDelta = NormalizeAngle(next.GetRot().y - curr.GetRot().y);
        segmentDist[i] = dist;
        angularVel[i] = yawDelta / dist;
    }

    // Pass : drift
    std::vector<int> driftDir(nodeCount, 0); // -1 = right, 0 = none, +1 = left
    for (size_t i = 0; i < nodeCount; i++)
    {
        if (!groundQuads[i]) { continue; }
        if (std::abs(angularVel[i]) >= SHARP_TURN_DEG_PER_UNIT)
            driftDir[i] = (angularVel[i] > 0.0f) ? 1 : -1;
    }

    struct DriftChunk {
        size_t start, end; // inclusive indices
        int dir;           // -1 or +1
    };

    auto chunkDist = [&](size_t start, size_t end) {
        float d = 0.f;
        for (size_t i = start; i <= end; i++) d += segmentDist[i];
        return d;
        };

    auto gapDist = [&](const DriftChunk& a, const DriftChunk& b) {
        float d = 0.f;
        for (size_t i = a.end + 1; i < b.start; i++) d += segmentDist[i];
        return d;
        };

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

    const float MIN_DRIFT_DIST = 10.0f;
    const float MIN_GAP_DIST = 15.0f;

    // Merge close same dir chuncks
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
    //Remove short dritfs
    chunks = buildChunks();
    for (auto& c : chunks) {
        if (chunkDist(c.start, c.end) < MIN_DRIFT_DIST) {
            for (size_t i = c.start; i <= c.end; i++) driftDir[i] = 0;
        }
    }
    // Reduce drift size when 2 very close opposite drift
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
    //Remove short dritfs again
    chunks = buildChunks();
    for (auto& c : chunks) {
        if (chunkDist(c.start, c.end) < MIN_DRIFT_DIST) {
            for (size_t i = c.start; i <= c.end; i++) driftDir[i] = 0;
        }
    }


    // Pass flags and settings
    uint8_t lastckpt = 0;
    for (size_t i = 0; i < nodeCount; i++)
    {
        BotNode& node = m_nodes[i];
        node.SetPathChange(3); // no path change
        node.SetPathChangeIndex(static_cast<int>(i));

        BotFlags flags = node.GetFlags();
        if (driftDir[i] == -1)
            flags.driftLeft = true;      
        if (driftDir[i] == 1)
            flags.driftRight = true;
        if (driftDir[i] != 0)
        {
            flags.skidmarkBack = true;
            flags.skidmarkFront = true;
        }        
        
        if (groundQuads[i])
        {
            if (groundQuads[i]->GetCheckpoint() >=  0) 
                lastckpt = static_cast<uint8_t>(groundQuads[i]->GetCheckpoint());
            node.SetTerrain(groundQuads[i]->GetTerrain());
            if (!groundQuads[(i + 1) % nodeCount])
                flags.jump = true;
            uint8_t t = groundQuads[i]->GetTerrain();
            if (t == TerrainType::WATER || t == TerrainType::FAST_WATER || t == TerrainType::MUD)
                flags.sink = true;
            if (groundQuads[i]->GetFlags() & QuadFlags::REVERB)
                flags.echo = true;
            if (groundQuads[i]->GetFlags() & QuadFlags::MOON_GRAVITY)
                flags.lowGrav = true;
            if (groundQuads[i]->GetFlags() & QuadFlags::REFLECTION_1)
            {
                node.SetReflection(0);
                node.SetSpecialMode(BotSpecialBits::REFLECTION);
            }
            if (groundQuads[i]->GetFlags() & QuadFlags::REFLECTION_2)
            {
                node.SetReflection(1);
                node.SetSpecialMode(BotSpecialBits::REFLECTION);
            }
            // Shadow
            int sumColor = 0;
            for (const Vertex& vert : groundQuads[i]->GetVertices())
            {
                Color col = vert.GetColor(true);
                sumColor += col.r + col.g + col.b;
            }
            float ratio = static_cast<float>(sumColor) / (9.0f * 255.0f * 3.0f);
            if (ratio > 0.5f)
                node.SetShadow(0);
            else
                node.SetShadow(static_cast<int>(15.0f - ratio * 30.0f));
        }
        else
        {
            node.SetTerrain(TerrainType::ASPHALT);
            flags.midAir = true;
        }
        node.SetCheckpoint(lastckpt);

        for (const Quadblock& quad : quadblocks)
        {
            float dist = quad.GetBoundingBox().Distance(node.GetPos());
            if (dist < 2.0f)
            {
                if (quad.GetFlags() & QuadFlags::TRIGGER_SCRIPT && quad.GetTerrain() == TerrainType::DIRT)
                {
                    flags.turboPad = true;
                    flags.skidmarkFront = true;
                }                
                if (quad.GetFlags() & QuadFlags::TRIGGER_SCRIPT && quad.GetTerrain() == TerrainType::GRASS)
                {
                    flags.turboPadLow = true;
                    flags.skidmarkFront = true;
                }
            }
        }

        if (groundQuads[i] && !groundQuads[(i + nodeCount - 1) % nodeCount]) // just landed, skidmark for 10u
        {
            flags.skidmarkFront = true; 
            float distSinceLanding = 0.0f;
            size_t k = 0;
            while (distSinceLanding < SKIDMARK_LENGTH && k < 15)
            {
                BotNode& nextNode = m_nodes[(i + k) % nodeCount];
                BotFlags nextFlags = nextNode.GetFlags();
                nextFlags.skidmarkFront = true;
                nextNode.SetFlags(nextFlags);
                distSinceLanding += (nextNode.GetPos() - node.GetPos()).Length();
                k++;
            }
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
    header.numPoints = static_cast<uint16_t>(m_nodes.size() - 1);
    header.unk1 = 0;
    header.posY = ConvertFloat(m_nodes[0].GetPos().y, FP_ONE_GEO);
    header.offLastPoint = 0;//m_offLastPoint;
    std::copy(std::begin(m_physUnk), std::end(m_physUnk), std::begin(header.physUnk)); // can't be removed (on crash cove, ramp fails), need to be understood
    std::memcpy(buffer.data(), &header, sizeof(header));

    for (int i = 0; i < m_nodes.size() - 1; i++)
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
