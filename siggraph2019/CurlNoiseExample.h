///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2019 DNEG
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DNEG nor the names
// of its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointGroup.h>

#include <openvdb_ax/codegen/OpenSimplexNoise.h>

namespace detail
{

struct NoiseParameters
{
    NoiseParameters(const float amplitude,
                    const float persistence,
                    const float lacunarity,
                    const int octaves,
                    const openvdb::Vec3f& frequency,
                    const openvdb::Vec3f& offset)
        : mAmplitude(amplitude)
        , mPersistence(persistence)
        , mLacunarity(lacunarity)
        , mOctaves(octaves)
        , mFrequency(frequency)
        , mOffset(offset) {}

    const float mAmplitude;
    const float mPersistence;
    const float mLacunarity;
    const int mOctaves;
    const openvdb::Vec3f mFrequency;
    const openvdb::Vec3f mOffset;
};

struct SimplexNoise
{
    // Open simplex noise - Visually axis-decorrelated coherent noise algorithm
    // based on the simplectic honeycomb.
    // See https://gist.github.com/KdotJPG/b1270127455a94ac5d19

    static double noise(double x, double y, double z)
    {
        static const OSN::OSNoise noiseGenerator = OSN::OSNoise();
        const double result = noiseGenerator.eval<double>(x, y, z);

        // adjust result so that it lies between 0 and 1, since Noise::eval returns
        // a number between -1 and 1
        return (result + 1.0) * 0.5;
    }
};

template <typename NoiseT>
openvdb::Vec3d curlNoise(const openvdb::Vec3d& in)
{
    float delta = 0.0001f;
    float a, b;

    openvdb::Vec3d out;

    // noise coordinates for vector potential components.
    float p[3][3] = {
        { static_cast<float>((in)[0]) + 000.0f, static_cast<float>((in)[1]) + 000.0f, static_cast<float>((in)[2]) + 000.0f }, // x
        { static_cast<float>((in)[0]) + 256.0f, static_cast<float>((in)[1]) - 256.0f, static_cast<float>((in)[2]) + 256.0f }, // y
        { static_cast<float>((in)[0]) - 512.0f, static_cast<float>((in)[1]) + 512.0f, static_cast<float>((in)[2]) - 512.0f }, // z
    };

    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
    // Compute curl.x
    a = (NoiseT::noise(p[2][0], p[2][1] + delta, p[2][2]) - NoiseT::noise(p[2][0], p[2][1] - delta, p[2][2])) / (2.0f * delta);
    b = (NoiseT::noise(p[1][0], p[1][1], p[1][2] + delta) - NoiseT::noise(p[1][0], p[1][1], p[1][2] - delta)) / (2.0f * delta);
    out[0] = a - b;

    // Compute curl.y
    a = (NoiseT::noise(p[0][0], p[0][1], p[0][2] + delta) - NoiseT::noise(p[0][0], p[0][1], p[0][2] - delta)) / (2.0f * delta);
    b = (NoiseT::noise(p[2][0] + delta, p[2][1], p[2][2]) - NoiseT::noise(p[2][0] - delta, p[2][1], p[2][2])) / (2.0f * delta);
    out[1] = a - b;

    // Compute curl.z
    a = (NoiseT::noise(p[1][0] + delta, p[1][1], p[1][2]) - NoiseT::noise(p[1][0] - delta, p[1][1], p[1][2])) / (2.0f * delta);
    b = (NoiseT::noise(p[0][0], p[0][1] + delta, p[0][2]) - NoiseT::noise(p[0][0], p[0][1] - delta, p[0][2])) / (2.0f * delta);
    out[2] = a - b;
    OPENVDB_NO_TYPE_CONVERSION_WARNING_END

    return out;
}

} // namespace detail

const char* CurlNoiseAX = R"AX(

if (!ingroup("escaped")) {

    float amplitude = 1.0f; //$amplitude;
    float persistence = 1.0f; //$persistence;
    float lacunarity = 1.0f; //$lacunarity;
    vec3f freq = {3.0f,3.0f,3.0f}; //vec3f$frequency;
    vec3f offset = {0.0f,0.0f,0.0f}; //vec3f$offset;
    int octaves = 3; // $octaves;
    vec3f noise = 0.0f;

    for (int octave = 0; octave < octaves; ++octave) {
        vec3f noisePos = v@P * freq + offset;
        noise += curlsimplexnoise(noisePos) * amplitude;

        amplitude *= persistence;
        freq *= lacunarity;
    }

    v@v += length(v@v) * noise;
}

)AX";


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


void applyCurlNoise(openvdb::points::PointDataGrid::Ptr grid)
{
    if (!grid) return;
    auto leaf = grid->tree().cbeginLeaf();
    if (!leaf) return;

    if (!leaf->hasAttribute("v")) {
        openvdb::points::appendAttribute<openvdb::Vec3f>(grid->tree(), "v");
    }

    openvdb::points::AttributeSet::DescriptorPtr desc = leaf->attributeSet().descriptorPtr();

    openvdb::points::AttributeSet::Descriptor::GroupIndex groupIndex;
    bool hasGroup = false;
    if (desc->hasGroup("escaped")) {
        groupIndex = leaf->attributeSet().groupIndex("escaped");
        hasGroup = true;
    }
    const size_t ppos = desc->find("P"); // always exists on VDB points
    const size_t vpos = desc->find("v");

    const openvdb::math::Transform& transform = grid->transform();
    detail::NoiseParameters noiseParams(1.0f, 1.0f, 1.0f, 3, openvdb::Vec3f(3.0f), openvdb::Vec3f(0.0f));

    auto scaledCurlNoiseOp = [ppos, vpos, hasGroup, &noiseParams, &groupIndex, &transform]
        (openvdb::points::PointDataTree::LeafNodeType& leaf, size_t)
    {
        // Attribute handles
        openvdb::points::AttributeHandle<openvdb::Vec3f>::Ptr posHandle =
            openvdb::points::AttributeHandle<openvdb::Vec3f>::create(leaf.attributeArray(ppos));
        openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::Ptr vHandle =
            openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(leaf.attributeArray(vpos));

        // Group Filter
        openvdb::points::GroupFilter filter(groupIndex);
        if (hasGroup) filter.reset(leaf);

        for (auto voxel = leaf.beginValueOn(); voxel; ++voxel) {

            const openvdb::Coord voxelCoord = voxel.getCoord();

            auto iter = leaf.beginIndexVoxel(voxelCoord);
            if (!iter) continue;

            const openvdb::Vec3f voxelPos = voxelCoord.asVec3s();

            for (; iter; ++iter) {
                const openvdb::Index idx = *iter;

                if (hasGroup && filter.valid(iter)) continue;

                float amplitude = noiseParams.mAmplitude;
                openvdb::Vec3f frequency = noiseParams.mFrequency;

                openvdb::Vec3f noise(0.0, 0.0, 0.0);
                openvdb::Vec3f posWS = transform.indexToWorld(posHandle->get(idx) + voxelPos);
                openvdb::Vec3d newNoise;

                for (int octave = 0; octave < noiseParams.mOctaves; ++octave) {
                    noise += detail::curlNoise<detail::SimplexNoise>(posWS * frequency + noiseParams.mOffset) * amplitude;

                    amplitude *= noiseParams.mPersistence;
                    frequency *= noiseParams.mLacunarity;
                }

                openvdb::Vec3f vel = vHandle->get(idx);
                vel += vel.length() * noise;
                vHandle->set(idx, vel);
            }
        }
    };

    openvdb::tree::LeafManager<openvdb::points::PointDataTree> leafManager(grid->tree());
    leafManager.foreach(scaledCurlNoiseOp);
}
